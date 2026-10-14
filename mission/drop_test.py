"""Paket bırakma testi (aşama 1: havada durarak bırakma).

Her deneme:
  1. Arabaların yeri rastgele değişir, bir hedef araba seçilir.
  2. Drone hedefin yaklaşık konumuna gider (GPS hatası gibi birkaç metre sapmayla).
  3. Alt kamerada arabayı görüp görüntünün ortasına alarak tam üstüne hizalanır.
  4. Havada durur ve paketi bırakır; paket fizik motoruyla düşer.
  5. Paketin durduğu yer ölçülür: arabanın merkezine uzaklık ve arabanın üstünde mi.

Kullanım:
  python mission/drop_test.py                          # 5 deneme, 10 m
  python mission/drop_test.py --deneme 10 --irtifa 20
  python mission/drop_test.py --model C:/UE/drone_runs/car/weights/best.pt   # hedefi YOLO bulsun
"""
import argparse
import csv
import json
import math
import time
from datetime import datetime

import numpy as np
from cosysairsim.utils import quaternion_to_euler_angles

import fly_mission
from collect_data import scatter_cars
from fly_mission import hide_preview, show
from sim_client import (ROOT, airsim, connect, get_boxes, get_rgb, load_mission, safe_land,
                        setup_detection, wait_until_still)

PACKAGE_SIZE = (0.4, 0.3, 0.25)  # metre
GPS_ERROR_M = 3.0                # hedefin yaklaşık konumundaki rastgele hata


def target_box(boxes, target_name, w, h):
    """Hedef arabanın kutusu. Unreal etiketlerinde isimle, YOLO'da görüntü merkezine en yakın kutu."""
    if target_name:
        # isim "SM_Car_0_<aktör>_<id>" biçiminde; "_" eki Actor_2'nin Actor_25'e uymasını önler
        boxes = [b for b in boxes if f"{target_name}_" in b[4] or b[4] == target_name]
    if not boxes:
        return None
    return min(boxes, key=lambda b: ((b[0] + b[2]) / 2 - w / 2) ** 2 + ((b[1] + b[3]) / 2 - h / 2) ** 2)


def detect(client, cam, model, target_name):
    img = get_rgb(client, cam)
    if model is None:
        boxes = get_boxes(client, cam)
    else:
        res = model.predict(img, conf=0.3, verbose=False)[0]
        boxes = [(*b.tolist(), "car") for b in res.boxes.xyxy.cpu()]
        target_name = None
    return img, target_box(boxes, target_name, img.shape[1], img.shape[0])


def center_over_target(client, cam, model, target_name, alt, timeout=40):
    """Arabayı kameranın tam ortasına alana kadar küçük hız komutlarıyla kayar (kapalı döngü).

    Kamera 90° yatay görüş açısıyla dik aşağı bakıyor: yükseklik H iken görüntünün
    yarı genişliği H metre, yarı yüksekliği H * (h / w) metre eder.
    Görüntünün üstü drone'un önü (kuzey/x+), sağı drone'un sağı (doğu/y+).

    Teşhis (diag_centering.py) gösterdi ki bu çevirme sapmayı ~1.3-1.7 kat büyük
    hesaplıyor (araba tavanı kameraya yakın olduğu için kutu dışa kayıyor) ve
    moveToPosition hedefe ~0.8 m kala bitiyor. Kapalı döngü ikisinden de etkilenmez:
    araba görüntü merkezine geldikçe hata sıfıra gider.
    """
    t0 = time.time()
    good = 0
    while time.time() - t0 < timeout:
        img, box = detect(client, cam, model, target_name)
        if box is None:
            client.hoverAsync()
            return False
        show(img, [box])
        h, w = img.shape[:2]
        ex = ((box[0] + box[2]) / 2 - w / 2) / (w / 2)
        ey = ((box[1] + box[3]) / 2 - h / 2) / (h / 2)
        pos = client.getMultirotorState().kinematics_estimated.position
        height = GROUND_Z - pos.z_val
        dx, dy = -ey * height * h / w, ex * height
        err = math.hypot(dx, dy)
        good = good + 1 if err < 0.08 else 0
        if good >= 4:  # art arda birkaç karede ortada: yerleşti
            client.hoverAsync()
            return True
        gain = 0.6  # hesaplanan sapma büyük çıktığı için temkinli kazanç
        vx = float(np.clip(gain * dx, -1.5, 1.5))
        vy = float(np.clip(gain * dy, -1.5, 1.5))
        client.moveByVelocityZAsync(vx, vy, -alt, 0.5)
        time.sleep(0.15)
    client.hoverAsync()
    return True


def wait_until_rest(client, name, cam, timeout=8):
    """Paket durana kadar bekler (önizlemede düşüşü gösterir), son konumunu döndürür."""
    last = None
    still_since = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        if fly_mission.SHOW_PREVIEW:
            show(get_rgb(client, cam), [])
        p = client.simGetObjectPose(name).position
        cur = np.array([p.x_val, p.y_val, p.z_val])
        if last is not None and np.linalg.norm(cur - last) < 0.005:
            still_since = still_since or time.time()
            if time.time() - still_since > 0.5 and time.time() - t0 > 1.0:
                break
        else:
            still_since = None
        last = cur
        time.sleep(0.1)
    return last


def score(client, car, pkg_pos, car_height):
    """Paket ile araba merkezi arasındaki yatay mesafe ve arabanın üstünde olup olmadığı."""
    cp = client.simGetObjectPose(car)
    sc = client.simGetObjectScale(car)
    yaw = quaternion_to_euler_angles(cp.orientation)[2]
    dx, dy = pkg_pos[0] - cp.position.x_val, pkg_pos[1] - cp.position.y_val
    # araba eksenlerine göre: lx uzunluk boyunca, ly genişlik boyunca
    lx = dx * math.cos(yaw) + dy * math.sin(yaw)
    ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
    on_roof = pkg_pos[2] < GROUND_Z - sc.z_val * 0.8  # zeminden yüksekte duruyorsa arabanın üstünde
    return math.hypot(dx, dy), on_roof, (lx, ly)


def main():
    global GROUND_Z
    ap = argparse.ArgumentParser()
    ap.add_argument("--deneme", type=int, default=5)
    ap.add_argument("--irtifa", type=float, default=10.0, help="Bırakma yüksekliği (m)")
    ap.add_argument("--model", default=None, help="Hedefi YOLO ile bul (yoksa Unreal etiketleri)")
    ap.add_argument("--gizle", action="store_true", help="Kamera önizleme penceresini açma")
    args = ap.parse_args()
    fly_mission.SHOW_PREVIEW = not args.gizle

    cfg = load_mission()
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    GROUND_Z = scene["ground_z_ned"]
    model = None
    if args.model:
        from ultralytics import YOLO
        model = YOLO(args.model)

    client = connect()
    rng = np.random.default_rng()
    cam = cfg["camera"]
    setup_detection(client, cam, cfg["target_mesh_patterns"], cfg["detection_radius_m"])
    results = []
    alt = args.irtifa - GROUND_Z  # zemin NED'de +GROUND_Z'de; -alt zeminden irtifa kadar yukarısı

    try:
        client.takeoffAsync().join()
        for i in range(1, args.deneme + 1):
            scatter_cars(client, scene, cfg["landing"], rng)
            car = str(rng.choice(scene["cars"]))
            cp = client.simGetObjectPose(car).position
            ax = cp.x_val + rng.uniform(-GPS_ERROR_M, GPS_ERROR_M)
            ay = cp.y_val + rng.uniform(-GPS_ERROR_M, GPS_ERROR_M)
            print(f"\n=== Deneme {i}/{args.deneme}: hedef {car} ===")
            client.moveToPositionAsync(ax, ay, -alt, 5).join()

            if not center_over_target(client, cam, model, None if model else car, alt):
                print("  hedef kamerada görülmedi, atlanıyor")
                results.append({"deneme": i, "sonuc": "gorulmedi"})
                continue
            wait_until_still(client, max_speed=0.05)

            d = client.getMultirotorState().kinematics_estimated.position
            cp = client.simGetObjectPose(car).position
            print(f"  hizalandı: drone arabanın merkezinden "
                  f"{math.hypot(d.x_val - cp.x_val, d.y_val - cp.y_val):.2f} m uzakta")
            pkg = client.simSpawnObject(
                "Package", "SM_Package",
                airsim.Pose(airsim.Vector3r(d.x_val, d.y_val, d.z_val + 0.7), airsim.Quaternionr()),
                airsim.Vector3r(*PACKAGE_SIZE), physics_enabled=True)
            print(f"  paket bırakıldı ({args.irtifa:.0f} m)")
            rest = wait_until_rest(client, pkg, cam)
            dist, on_roof, (lx, ly) = score(client, car, rest, scene["car_height_m"])
            print(f"  merkeze uzaklık: {dist:.2f} m  |  {'ARABANIN ÜSTÜNDE ✔' if on_roof else 'ıska ✘'}"
                  f"  (boyuna {lx:+.2f} m, enine {ly:+.2f} m)")
            results.append({"deneme": i, "sonuc": "isabet" if on_roof else "iska",
                            "merkeze_uzaklik_m": round(dist, 3), "boyuna_m": round(lx, 3),
                            "enine_m": round(ly, 3)})
            time.sleep(1.5)
            client.simDestroyObject(pkg)
            hide_preview()  # sonraki uçuş boyunca pencere donmasın

        lx, ly = cfg["landing"]
        client.moveToPositionAsync(lx, ly, -alt, 5).join()
        safe_land(client)
    finally:
        client.enableApiControl(False)

    measured = [r for r in results if "merkeze_uzaklik_m" in r]
    hits = sum(r["sonuc"] == "isabet" for r in results)
    print(f"\n=== SONUÇ ({args.irtifa:.0f} m, {'YOLO' if model else 'Unreal etiketi'}) ===")
    print(f"İsabet: {hits}/{len(results)}")
    if measured:
        d = [r["merkeze_uzaklik_m"] for r in measured]
        print(f"Merkeze uzaklık: ortalama {np.mean(d):.2f} m, en kötü {max(d):.2f} m")

    out = ROOT / "results"
    out.mkdir(exist_ok=True)
    f = out / f"drop_{datetime.now():%Y%m%d_%H%M%S}_{args.irtifa:.0f}m.csv"
    with f.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=["deneme", "sonuc", "merkeze_uzaklik_m", "boyuna_m", "enine_m"])
        wr.writeheader()
        wr.writerows(results)
    print(f"Kaydedildi: {f}")


GROUND_Z = 0.0

if __name__ == "__main__":
    main()
