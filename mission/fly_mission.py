"""Otonom görev: waypoint'leri sırayla gez, hedef ara, veri topla, iniş noktasına in.

Kullanım:
  python mission/fly_mission.py                     # veri toplama (etiketleri Unreal verir)
  python mission/fly_mission.py --model best.pt     # eğitilmiş YOLO ile canlı tespit
"""
import argparse
import time
from datetime import datetime

import cv2
import numpy as np

from sim_client import (ROOT, airsim, connect, get_boxes, get_rgb, load_mission,
                        randomize_weather, safe_land, setup_detection, wait_until_still)


class Recorder:
    """Görüntüleri ve YOLO formatındaki etiketleri dataset/raw/<oturum>/ altına yazar."""

    def __init__(self):
        self.dir = ROOT / "dataset" / "raw" / datetime.now().strftime("%Y%m%d_%H%M%S")
        (self.dir / "images").mkdir(parents=True)
        (self.dir / "labels").mkdir(parents=True)
        self.count = 0

    def save(self, img, boxes):
        if img.mean() < 2:
            return  # siyah kare (kamera render etmemiş), veri setini kirletmesin
        if getattr(self, "_last", None) is not None and np.array_equal(img, self._last):
            return  # donmuş kamera: aynı kare tekrar geldi
        self._last = img
        h, w = img.shape[:2]
        lines = []
        for x1, y1, x2, y2, name in boxes:
            x1, x2 = np.clip([x1, x2], 0, w)
            y1, y2 = np.clip([y1, y2], 0, h)
            if (x2 - x1) < 12 or (y2 - y1) < 12:
                continue  # kenarda kırıntı kalmış ya da çok küçük: öğrenilemez, skoru haksız düşürür
            cls = 1 if "Mannequin" in name else 0   # 0: araba, 1: kişi (mesh adına göre)
            lines.append(f"{cls} {(x1 + x2) / 2 / w:.6f} {(y1 + y2) / 2 / h:.6f} "
                         f"{(x2 - x1) / w:.6f} {(y2 - y1) / h:.6f}")
        name = f"{self.count:06d}"
        # cv2.imwrite Türkçe karakterli yollara (Masaüstü) sessizce yazamıyor, bu yüzden imencode
        (self.dir / "images" / f"{name}.jpg").write_bytes(cv2.imencode(".jpg", img)[1].tobytes())
        (self.dir / "labels" / f"{name}.txt").write_text("\n".join(lines))
        self.count += 1


def detect(model, img, boxes_gt):
    """Model varsa onun tahmini, yoksa Unreal'in gerçek kutuları kullanılır."""
    if model is None:
        return boxes_gt
    res = model.predict(img, conf=0.4, verbose=False)[0]
    return [(*b.tolist(), "car") for b in res.boxes.xyxy.cpu()]


def fly_path(client, points, z, speed, cfg, rec, model, found):
    """Noktaları tek bir akıcı yol olarak uçar; yol boyunca görüntü alır ve tespit yapar."""
    path = [airsim.Vector3r(x, y, z) for x, y in points]
    client.moveOnPathAsync(path, speed, 600, airsim.DrivetrainType.MaxDegreeOfFreedom,
                           airsim.YawMode(False, 0), -1, 1)
    x, y = points[-1]
    cam = cfg["camera"]
    while True:
        pos = client.getMultirotorState().kinematics_estimated.position
        if np.hypot(pos.x_val - x, pos.y_val - y) < 1.5 and abs(pos.z_val - z) < 1.0:
            break
        img = get_rgb(client, cam)
        gt = get_boxes(client, cam)
        if rec:
            rec.save(img, gt)
        dets = detect(model, img, gt)
        if dets:
            found.append((round(pos.x_val, 1), round(pos.y_val, 1), len(dets)))
            print(f"  hedef görüldü @ ({pos.x_val:.1f}, {pos.y_val:.1f}): {len(dets)} adet")
        show(img, dets)
        time.sleep(cfg["capture_interval_s"])
    wait_until_still(client)


SHOW_PREVIEW = False  # --goster ile açılır


def show(img, dets):
    """Kamera önizlemesi: küçük pencere, tespitler kırmızı kutu."""
    if not SHOW_PREVIEW:
        return
    vis = cv2.resize(img, (480, 270))
    sx, sy = 480 / img.shape[1], 270 / img.shape[0]
    for x1, y1, x2, y2, *_ in dets:
        cv2.rectangle(vis, (int(x1 * sx), int(y1 * sy)), (int(x2 * sx), int(y2 * sy)), (0, 0, 255), 1)
    cv2.imshow("drone", vis)
    cv2.waitKey(1)


def hide_preview():
    """Uzun bekleyen adımlardan (alçalma, iniş) önce pencereyi kapat.

    Pencere birkaç saniye yenilenmezse Windows onu 'yanıt vermiyor' diye dondurur;
    sonraki show() çağrısı pencereyi yeniden açar.
    """
    if SHOW_PREVIEW:
        cv2.destroyAllWindows()
        cv2.waitKey(1)


def precision_land(client, cfg):
    """Helipad'i alt kamerada ortalayarak alçalır, sonra iner.

    Alt kamerada görüntünün üstü drone'un önüne bakar:
      piksel x+ (sağ)  -> gövde y+ (sağ)
      piksel y+ (aşağı) -> gövde x- (geri)
    """
    cam = cfg["camera"]
    setup_detection(client, cam, cfg["helipad_mesh_patterns"], 40)
    misses = 0
    seen = False
    for _ in range(200):
        img = get_rgb(client, cam)
        pads = get_boxes(client, cam)
        alt = -client.getMultirotorState().kinematics_estimated.position.z_val
        if not pads:
            misses += 1  # filtre değişince ilk kareler boş gelebiliyor; hemen vazgeçme
            if misses > 15:
                # alçakta helipad kadraja sığmayıp kaybolabilir; zaten ortalanmışsa sorun değil
                print("  helipad hizalandı, iniliyor" if seen else "  helipad görünmüyor, düz iniş yapılıyor")
                break
            time.sleep(0.1)
            continue
        misses = 0
        seen = True
        h, w = img.shape[:2]
        x1, y1, x2, y2, _ = pads[0]
        ex = ((x1 + x2) / 2 - w / 2) / (w / 2)  # -1..1
        ey = ((y1 + y2) / 2 - h / 2) / (h / 2)
        vz = 0.8 if abs(ex) < 0.1 and abs(ey) < 0.1 else 0.0  # ortalanınca alçal
        client.moveByVelocityBodyFrameAsync(-1.5 * ey, 1.5 * ex, vz, 0.3).join()
        show(img, pads)
        if alt < 2.0:
            break
    hide_preview()
    safe_land(client)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", default=None)
    ap.add_argument("--model", default=None, help="Eğitilmiş YOLO .pt dosyası")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--goster", action="store_true", help="Kamera önizleme penceresini aç (FPS düşürür)")
    args = ap.parse_args()
    global SHOW_PREVIEW
    SHOW_PREVIEW = args.goster

    cfg = load_mission(args.mission)
    model = None
    if args.model:
        from ultralytics import YOLO
        model = YOLO(args.model)

    client = connect()
    rec = None if args.no_record else Recorder()
    try:
        found = fly_once(client, cfg, rec, model, cfg["waypoints"], cfg["altitude"],
                         np.random.default_rng())
    finally:
        client.enableApiControl(False)
        cv2.destroyAllWindows()

    print(f"Bitti. {len(found)} karede hedef görüldü.")
    if rec:
        print(f"{rec.count} görüntü kaydedildi: {rec.dir}")


def fly_once(client, cfg, rec, model, waypoints, z, rng, before_landing=None):
    """Tek tur: kalk, waypoint'leri gez (kaydederek), [ek iş], helipad'e hassas in."""
    setup_detection(client, cfg["camera"], cfg["target_mesh_patterns"], cfg["detection_radius_m"])
    found = []
    v = cfg["speed"]
    client.enableApiControl(True)
    client.armDisarm(True)
    print("Kalkış...")
    client.takeoffAsync().join()
    client.moveToZAsync(z, 3).join()

    if cfg["randomize_weather"]:
        randomize_weather(client, rng)
    print(f"{len(waypoints)} waypoint geziliyor ({-z:.0f} m irtifa)")
    fly_path(client, waypoints, z, v, cfg, rec, model, found)
    if before_landing is not None:
        before_landing(client)

    lx, ly = cfg["landing"]
    print(f"İniş noktasına gidiliyor: ({lx}, {ly})")
    fly_path(client, [(lx, ly)], z, v, cfg, None, model, found)
    hide_preview()
    from aruco_land import aruco_land  # döngüsel import olmasın diye burada
    aruco_land(client, cfg["camera"])   # simülatör bilgisi olmadan, işaretçiyle iniş
    return found


if __name__ == "__main__":
    main()
