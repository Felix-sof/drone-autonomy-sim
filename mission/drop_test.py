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
MODEL_CONF = 0.3                 # YOLO ile hizalanırken kabul edilen en düşük güven
MAX_MISSES = 10                  # hedef bu kadar ardışık karede görünmezse vazgeç (tespit titrer)
CAR_SIZE_M = (1.98, 5.24, 1.47)  # Fab araba modeli, mesh ekseninde (x=genişlik, y=uzunluk, z)
GPS_ERROR_M = 3.0                # hedefin yaklaşık konumundaki rastgele hata


def target_box(boxes, target_name, w, h, near=None, max_jump=None):
    """Hedef arabanın kutusu.

    Unreal etiketlerinde isimle bulunur. YOLO'da isim yok: `near` (bir önceki karede takip
    edilen nokta, yoksa görüntü merkezi) noktasına en yakın kutu seçilir; `max_jump` pikselden
    uzaktaysa hiç seçilmez. Böylece hedef bir kare kaybolunca drone yandaki arabaya kaymaz
    (otonom görevde aynı arabaya üç paket bırakılmasının sebebi buydu).
    """
    if target_name:
        # isim "SM_Car_0_<aktör>_<id>" biçiminde; "_" eki Actor_2'nin Actor_25'e uymasını önler
        boxes = [b for b in boxes if f"{target_name}_" in b[4] or b[4] == target_name]
    if not boxes:
        return None
    nx, ny = near if near is not None else (w / 2, h / 2)
    best = min(boxes, key=lambda b: ((b[0] + b[2]) / 2 - nx) ** 2 + ((b[1] + b[3]) / 2 - ny) ** 2)
    if max_jump is not None and math.hypot((best[0] + best[2]) / 2 - nx, (best[1] + best[3]) / 2 - ny) > max_jump:
        return None
    return best


def detect(client, cam, model, target_name, near=None):
    img = get_rgb(client, cam)
    if model is None:
        boxes = get_boxes(client, cam)
        return img, target_box(boxes, target_name, img.shape[1], img.shape[0])
    res = model.predict(img, conf=MODEL_CONF, verbose=False)[0]
    boxes = [(*b.tolist(), "car") for b in res.boxes.xyxy.cpu()]
    # takip: kilitlendikten sonra hedef kare başına en fazla görüntü genişliğinin %12'si kayabilir
    jump = 0.12 * img.shape[1] if near is not None else None
    return img, target_box(boxes, None, img.shape[1], img.shape[0], near, jump)


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
    misses = 0
    track = None  # YOLO'da kilitlenilen arabanın görüntüdeki son konumu
    while time.time() - t0 < timeout:
        img, box = detect(client, cam, model, target_name, near=track)
        if box is None:
            # YOLO güveni eşik etrafında titreyebiliyor: tek karede kaybolunca hemen vazgeçme
            client.hoverAsync()
            misses += 1
            if misses > MAX_MISSES:
                return False
            time.sleep(0.1)
            continue
        misses = 0
        show(img, [box])
        h, w = img.shape[:2]
        ex = ((box[0] + box[2]) / 2 - w / 2) / (w / 2)
        ey = ((box[1] + box[3]) / 2 - h / 2) / (h / 2)
        # drone hedefe doğru kaydıkça hedef görüntü merkezine yaklaşır; takip noktası kutunun merkezi
        track = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
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


LAST_IMPACT_SPEED = None   # son wait_until_rest çağrısında ölçülen çarpma hızı (m/s, düşey)


def wait_until_rest(client, name, cam, timeout=8):
    """Paketi izler: ilk çarpma noktasını ve durduğu yeri döndürür (önizlemede düşüşü gösterir).

    Konum sık örneklenir (~50 Hz) ki 10 m'lik ~1.4 s'lik düşüşte çarpma anı kaçmasın;
    kamera karesi daha seyrek alınır. İlk çarpma: hızla düşerken (>3 m/s) aşağı hızın
    birden kesildiği an.
    """
    last, last_t = None, None
    still_since = None
    impact = None
    falling_fast = False
    hist = []          # (zaman, z) farklı fizik kareleri; çarpma hızı birkaç kare üzerinden
    v_impact = None

    def speed_before(t_end, z_end):
        # okuma anı fizik karesine denk gelmediği için tek aralık gürültülü (yukarı kaçıyor):
        # çarpmadan önceki ~0.15 s'nin ortalama hızı; serbest düşüşte ortalama, pencerenin ORTASINDAKİ
        # hızdır: çarpma anına g*Δt/2 eklenir (sabit ivmede tam doğru)
        old = [hz for hz in hist if hz[0] <= t_end - 0.15]
        if not old:
            return None
        dt = t_end - old[-1][0]
        return (z_end - old[-1][1]) / dt + 9.81 * dt / 2

    t0 = time.time()
    next_frame = 0.0
    while time.time() - t0 < timeout:
        now = time.time()
        if fly_mission.SHOW_PREVIEW and now >= next_frame:
            show(get_rgb(client, cam), [])
            next_frame = time.time() + 0.15
        p = client.simGetObjectPose(name).position
        cur = np.array([p.x_val, p.y_val, p.z_val])
        if last is not None and np.array_equal(cur, last):
            # aynı fizik karesi tekrar okundu (Unreal ~45 Hz günceller, biz 50 Hz okuyoruz):
            # hız 0 sanılıp "çarptı" denmesin; durma süresi yine de sayılsın
            if still_since is None:
                still_since = now
            elif falling_fast and impact is None and now - still_since > 0.1:
                impact = last          # düşerken birden kıpırdamaz oldu: sekmeden durdu
                v_impact = speed_before(last_t, last[2])
            elif now - still_since > 0.5 and now - t0 > 1.0:
                break
            time.sleep(0.02)
            continue
        if last is not None:
            vz = (cur[2] - last[2]) / max(now - last_t, 1e-3)  # NED: + aşağı
            if vz > 3.0:
                falling_fast = True
            elif falling_fast and impact is None and vz < 1.0:
                impact = last  # hızın kesildiği andan hemen önceki konum
                v_impact = speed_before(last_t, last[2])
            if impact is None:
                hist.append((now, cur[2]))
            if np.linalg.norm(cur - last) < 0.003:
                still_since = still_since or now
                if now - still_since > 0.5 and now - t0 > 1.0:
                    break
            else:
                still_since = None
        last, last_t = cur, now
        time.sleep(0.02)
    global LAST_IMPACT_SPEED
    LAST_IMPACT_SPEED = None if v_impact is None else round(v_impact, 1)
    return last, (impact if impact is not None else last)


def score(client, car, pkg_pos, car_height):
    """Paket ile araba merkezi arasındaki yatay mesafe ve arabanın üstünde olup olmadığı."""
    cp = client.simGetObjectPose(car)
    sc = client.simGetObjectScale(car)
    yaw = quaternion_to_euler_angles(cp.orientation)[2]
    dx, dy = pkg_pos[0] - cp.position.x_val, pkg_pos[1] - cp.position.y_val
    # araba eksenlerine göre: lx uzunluk boyunca, ly genişlik boyunca
    lx = dx * math.cos(yaw) + dy * math.sin(yaw)
    ly = -dx * math.sin(yaw) + dy * math.cos(yaw)
    # gerçek modelde ölçek ~1 ve yükseklik sahneden; kutuda ölçek doğrudan yükseklik (metre)
    height = car_height * sc.z_val if sc.z_val < 1.3 else sc.z_val
    on_roof = pkg_pos[2] < GROUND_Z - height * 0.6  # zeminden yüksekte duruyorsa arabanın üstünde
    return math.hypot(dx, dy), on_roof, (lx, ly)


def on_car(client, car, p, margin=0.3):
    """p (x, y) noktası arabanın üstten görünen ayak izinin içinde mi (yön ve ölçek dahil)."""
    cp = client.simGetObjectPose(car)
    s = client.simGetObjectScale(car).x_val
    yaw = quaternion_to_euler_angles(cp.orientation)[2]
    dx, dy = p[0] - cp.position.x_val, p[1] - cp.position.y_val
    lx = dx * math.cos(yaw) + dy * math.sin(yaw)    # mesh x ekseni (genişlik)
    ly = -dx * math.sin(yaw) + dy * math.cos(yaw)   # mesh y ekseni (uzunluk)
    return abs(lx) <= CAR_SIZE_M[0] * s / 2 + margin and abs(ly) <= CAR_SIZE_M[1] * s / 2 + margin


def classify_impact(client, cars, target, impact):
    """İlk temas neye geldi: 'hedef', 'baska_araba', 'agac_vb' (yüksekte ama araba değil) ya da 'yer'."""
    elevated = impact[2] < GROUND_Z - 0.4
    if elevated and on_car(client, target, impact):
        return "hedef"
    if elevated:
        if any(on_car(client, c, impact) for c in cars if c != target):
            return "baska_araba"
        return "agac_vb"
    return "yer"


def save_debug(client, cam, model, car, img, path):
    """Teşhis karesi: YOLO kutuları mavi, gerçek hedef yeşil."""
    import cv2
    vis = img.copy()
    res = model.predict(img, conf=MODEL_CONF, verbose=False)[0]
    for (x1, y1, x2, y2), cf in zip(res.boxes.xyxy.cpu().tolist(), res.boxes.conf.cpu().tolist()):
        cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
        cv2.putText(vis, f"{cf:.2f}", (int(x1), int(y1) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    for x1, y1, x2, y2, name in get_boxes(client, cam):
        if f"{car}_" in name:
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    h, w = vis.shape[:2]
    cv2.drawMarker(vis, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(cv2.imencode(".jpg", cv2.resize(vis, (960, 540)))[1].tobytes())


def main():
    global GROUND_Z
    ap = argparse.ArgumentParser()
    ap.add_argument("--deneme", type=int, default=5)
    ap.add_argument("--irtifa", type=float, default=10.0, help="Bırakma yüksekliği (m)")
    ap.add_argument("--model", default=None, help="Hedefi YOLO ile bul (yoksa Unreal etiketleri)")
    ap.add_argument("--gizle", action="store_true", help="Kamera önizleme penceresini açma")
    args = ap.parse_args()
    fly_mission.SHOW_PREVIEW = not args.gizle
    debug_dir = ROOT / "results" / "debug" / f"drop_{datetime.now():%Y%m%d_%H%M%S}"

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
    # zemin NED'de +GROUND_Z'de; -x zeminden o kadar yukarısı. Ormanda hedefe ağaçların
    # üstünden (seyir) gidilir, açıklığın üstünde dikey alçalıp bırakılır.
    alt = args.irtifa - GROUND_Z
    cruise = max(scene.get("cruise_alt_m", 0), args.irtifa) - GROUND_Z
    target_name = lambda c: None if model else c  # noqa: E731

    try:
        client.takeoffAsync().join()
        client.moveToZAsync(-cruise, 3).join()
        for i in range(1, args.deneme + 1):
            scatter_cars(client, scene, cfg["landing"], rng)
            car = str(rng.choice(scene["cars"]))
            cp = client.simGetObjectPose(car).position
            ax = cp.x_val + rng.uniform(-GPS_ERROR_M, GPS_ERROR_M)
            ay = cp.y_val + rng.uniform(-GPS_ERROR_M, GPS_ERROR_M)
            print(f"\n=== Deneme {i}/{args.deneme}: hedef {car} ===")
            t_start = time.time()
            client.moveToPositionAsync(ax, ay, -cruise, 5).join()

            # önce seyir irtifasında kabaca, alçaldıktan sonra hassas hizalan
            seen = center_over_target(client, cam, model, target_name(car), cruise)
            if seen and cruise > alt:
                client.moveToZAsync(-alt, 2).join()
                seen = center_over_target(client, cam, model, target_name(car), alt)
            if not seen:
                print("  hedef kamerada görülmedi, atlanıyor")
                if model is not None:
                    save_debug(client, cam, model, car, get_rgb(client, cam), debug_dir / f"{i:02d}_gorulmedi.jpg")
                results.append({"deneme": i, "sonuc": "gorulmedi"})
                client.moveToZAsync(-cruise, 3).join()
                continue
            wait_until_still(client, max_speed=0.05)

            d = client.getMultirotorState().kinematics_estimated.position
            cp = client.simGetObjectPose(car).position
            align = math.hypot(d.x_val - cp.x_val, d.y_val - cp.y_val)
            print(f"  hizalandı: drone hedef arabanın merkezinden {align:.2f} m uzakta")
            if model is not None:
                save_debug(client, cam, model, car, get_rgb(client, cam), debug_dir / f"{i:02d}_hizalama.jpg")
            pkg = client.simSpawnObject(
                "Package", "SM_Package",
                airsim.Pose(airsim.Vector3r(d.x_val, d.y_val, d.z_val + 0.7), airsim.Quaternionr()),
                airsim.Vector3r(*PACKAGE_SIZE), physics_enabled=True)
            print(f"  paket bırakıldı ({args.irtifa:.0f} m)")
            rest, impact = wait_until_rest(client, pkg, cam)
            # ANA ÖLÇÜT ilk çarpma noktası: hedef arabanın ayak izine mi geldi.
            # Ormanda "yerden yüksekte" tek başına yetmez: ağaç ya da başka araba olabilir.
            hit_dist, _, _ = score(client, car, impact, scene["car_height_m"])
            kind = classify_impact(client, scene["cars"], car, impact)
            hit = kind == "hedef"
            # ek bilgi: paket arabanın üstünde mi kaldı (sekme/kayma dahil)
            dist, on_roof, (lx, ly) = score(client, car, rest, scene["car_height_m"])
            label = {"hedef": "HEDEF ARABAYA ÇARPTI ✔", "baska_araba": "başka arabaya çarptı ✘",
                     "agac_vb": "ağaca/başka nesneye çarptı ✘", "yer": "yere düştü ✘"}[kind]
            print(f"  İSABET: {label} (ilk temas hedef merkezine {hit_dist:.2f} m)"
                  f"  |  ek: {'üstünde kaldı' if on_roof else 'sekip düştü'}"
                  f" (durduğu yer merkeze {dist:.2f} m)")
            results.append({"deneme": i, "sonuc": "isabet" if hit else "iska", "ilk_temas": kind,
                            "hizalama_m": round(align, 3), "ustunde_kaldi": on_roof,
                            "vurus": hit, "vurus_merkeze_m": round(hit_dist, 3),
                            "merkeze_uzaklik_m": round(dist, 3), "boyuna_m": round(lx, 3),
                            "enine_m": round(ly, 3), "sure_s": round(time.time() - t_start, 1)})
            time.sleep(1.5)
            client.simDestroyObject(pkg)
            hide_preview()  # sonraki uçuş boyunca pencere donmasın
            client.moveToZAsync(-cruise, 3).join()  # ağaçların üstüne geri çık

        lx, ly = cfg["landing"]
        client.moveToPositionAsync(lx, ly, -cruise, 5).join()
        safe_land(client)
    finally:
        client.enableApiControl(False)

    measured = [r for r in results if "merkeze_uzaklik_m" in r]
    hits = sum(r["sonuc"] == "isabet" for r in results)
    stayed = sum(bool(r.get("ustunde_kaldi")) for r in results)
    print(f"\n=== SONUÇ ({args.irtifa:.0f} m, {'YOLO' if model else 'Unreal etiketi'}) ===")
    print(f"İsabet (ilk temas hedef arabada): {hits}/{len(results)}")
    kinds = [r.get("ilk_temas", r["sonuc"]) for r in results]
    print("Dağılım: " + ", ".join(f"{k}={kinds.count(k)}" for k in
                                  ("hedef", "baska_araba", "agac_vb", "yer", "gorulmedi")))
    print(f"Ek: paket arabanın üstünde kaldı: {stayed}/{len(results)}")
    if measured:
        d = [r["vurus_merkeze_m"] for r in measured]
        print(f"İlk temasın merkeze uzaklığı: ortalama {np.mean(d):.2f} m, en kötü {max(d):.2f} m")
        print(f"Deneme başına süre (uçuş+hizalama+bırakma): "
              f"ortalama {np.mean([r['sure_s'] for r in measured]):.0f} s")

    out = ROOT / "results"
    out.mkdir(exist_ok=True)
    f = out / f"drop_{datetime.now():%Y%m%d_%H%M%S}_{args.irtifa:.0f}m.csv"
    with f.open("w", newline="", encoding="utf-8") as fh:
        wr = csv.DictWriter(fh, fieldnames=["deneme", "sonuc", "ilk_temas", "hizalama_m", "vurus",
                                            "vurus_merkeze_m", "ustunde_kaldi",
                                            "merkeze_uzaklik_m", "boyuna_m", "enine_m", "sure_s"])
        wr.writeheader()
        wr.writerows(results)
    print(f"Kaydedildi: {f}")


GROUND_Z = 0.0

if __name__ == "__main__":
    main()
