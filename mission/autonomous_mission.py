"""Rotasız tam otonom görev: ara -> YOLO ile bul -> hizalan -> paket bırak -> aramaya devam -> in.

Drone'a hedef konumu verilmez. Görev alanını ağaçların üstünden şerit şerit (çim biçme
deseni) tarar; alt kameradaki her karede eğitilmiş YOLO modeli araba arar. Yeni bir araba
görünce aramayı keser, arabanın tahmini konumuna gider, kamerayla tam üstüne hizalanır,
paketi bırakır ve aramaya kaldığı yerden devam eder. İstenen sayıda teslimattan sonra ya da
alan bitince helipad'e iner.

Değerlendirme (drone'un kararlarını etkilemez): Unreal'deki gerçek araba konumlarıyla
her paketin ilk teması bir arabaya mı geldi, aynı arabaya iki kez mi bırakıldı, yanlış
alarm (araba olmayan yere bırakma) oldu mu.

Kullanım:
  python mission/autonomous_mission.py --model C:/UE/drone_runs/car_forest/weights/best.pt
  python mission/autonomous_mission.py --model ... --teslimat 5 --irtifa 25
"""
import argparse
import json
import math
import time
from datetime import datetime

import numpy as np

import drop_test
import fly_mission
from collect_data import area, scatter_cars
from aruco_land import aruco_land
from drop_test import PACKAGE_SIZE, center_over_target, classify_impact, wait_until_rest
from sim_client import (ROOT, airsim, connect, get_rgb, load_mission, safe_land,
                        wait_until_still)

LANE_SPACING_M = 35   # 25 m'den kamera ~50 m genişliğinde şerit görür; kenarlarda örtüşme kalsın
MIN_CONF = 0.5        # aramada "araba var" demek için gereken güven
NEW_TARGET_M = 8.0    # daha önce gidilen bir hedefe bu kadar yakın tespitler yok sayılır
MAX_APPROACHES = 12   # güvenlik: bir görevde en fazla bu kadar hedefe yaklaşılır


def lawnmower(a, spacing):
    """Alanı x boyunca gidip gelen şeritlerle kaplayan nokta listesi."""
    pts, flip = [], False
    y = a[2] + spacing / 3
    while y <= a[3] + 1e-6:
        xs = (a[0], a[1]) if not flip else (a[1], a[0])
        pts += [(xs[0], y), (xs[1], y)]
        flip = not flip
        y += spacing
    return pts


def detections_world(model, img, pos, ground_z):
    """YOLO tespitlerini kabaca dünya koordinatına çevirir (yaklaşma için; kesin hizalama kamerayla)."""
    h, w = img.shape[:2]
    height = ground_z - pos.z_val
    res = model.predict(img, conf=MIN_CONF, verbose=False)[0]
    out = []
    for (x1, y1, x2, y2), conf in zip(res.boxes.xyxy.cpu().tolist(), res.boxes.conf.cpu().tolist()):
        ex = ((x1 + x2) / 2 - w / 2) / (w / 2)
        ey = ((y1 + y2) / 2 - h / 2) / (h / 2)
        out.append((pos.x_val - ey * height * h / w, pos.y_val + ex * height, conf))
    return out, res


def nearest_car(client, cars, x, y):
    best = None
    for name in cars:
        p = client.simGetObjectPose(name).position
        d = math.hypot(p.x_val - x, p.y_val - y)
        if best is None or d < best[1]:
            best = (name, d)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--teslimat", type=int, default=3, help="Kaç pakete kadar bırakılsın")
    ap.add_argument("--irtifa", type=float, default=25.0, help="Arama ve bırakma irtifası (m)")
    ap.add_argument("--hiz", type=float, default=5.0)
    ap.add_argument("--goster", action="store_true")
    args = ap.parse_args()
    fly_mission.SHOW_PREVIEW = args.goster

    from ultralytics import YOLO
    model = YOLO(args.model)
    cfg = load_mission()
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    G = scene["ground_z_ned"]
    drop_test.GROUND_Z = G
    cam = cfg["camera"]
    z = G - max(args.irtifa, scene.get("cruise_alt_m", 0))
    alt = -z  # center_over_target -alt irtifasında hizalanır

    client = connect()
    rng = np.random.default_rng()
    scatter_cars(client, scene, cfg["landing"], rng)  # her görev yeni bir sahne
    route = lawnmower(area(scene), LANE_SPACING_M)
    print(f"Arama deseni: {len(route) // 2} şerit, {len(route)} nokta, irtifa {args.irtifa:.0f} m")

    visited = []      # gidilen hedeflerin tahmini konumları
    dropped = []      # paket bırakılan yerler (aynı arabaya ikinci paket bırakılmasın)
    approaches = 0
    landing_err = None
    deliveries = []
    t_start = time.time()
    try:
        client.takeoffAsync().join()
        client.moveToZAsync(z, 3).join()
        idx = 0
        while idx < len(route) and len(deliveries) < args.teslimat:
            client.moveOnPathAsync([airsim.Vector3r(x, y, z) for x, y in route[idx:]], args.hiz, 600,
                                   airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0), -1, 1)
            target = None
            while target is None:
                pos = client.getMultirotorState().kinematics_estimated.position
                # sıradaki noktaya varınca rotada ilerle
                while idx < len(route) and math.hypot(route[idx][0] - pos.x_val, route[idx][1] - pos.y_val) < 3:
                    idx += 1
                if idx >= len(route):
                    break
                img = get_rgb(client, cam)
                dets, res = detections_world(model, img, pos, G)
                fly_mission.show(img, [(*b, "car") for b in res.boxes.xyxy.cpu().tolist()])
                fresh = [d for d in dets if all(math.hypot(d[0] - vx, d[1] - vy) > NEW_TARGET_M
                                                for vx, vy in visited)]
                if fresh:
                    target = max(fresh, key=lambda d: d[2])
            if target is None:
                break

            tx, ty, conf = target
            visited.append((tx, ty))
            approaches += 1
            if approaches > MAX_APPROACHES:
                print(f"  {MAX_APPROACHES} yaklaşma sınırı aşıldı, görev sonlandırılıyor")
                break
            print(f"\nAraba bulundu (güven {conf:.2f}) ~({tx:.1f}, {ty:.1f}) -> yaklaşılıyor")
            client.moveToPositionAsync(tx, ty, z, 4).join()
            if not center_over_target(client, cam, model, None, alt):
                print("  yaklaşınca kayboldu, aramaya devam")
                continue
            wait_until_still(client, max_speed=0.05)
            d = client.getMultirotorState().kinematics_estimated.position
            # tahmini konum da listede kalsın: tahmin ile hizalanılan araba farklıysa
            # (kenardaki tespitlerde tahmin metrelerce sapabiliyor) aynı tespite sonsuza dek dönülmesin
            visited.append((d.x_val, d.y_val))
            if any(math.hypot(d.x_val - px, d.y_val - py) < NEW_TARGET_M for px, py in dropped):
                print("  bu araba zaten teslim aldı, bırakılmadı; aramaya devam")
                continue
            dropped.append((d.x_val, d.y_val))
            pkg = client.simSpawnObject(
                "Package", "SM_Package",
                airsim.Pose(airsim.Vector3r(d.x_val, d.y_val, d.z_val + 0.7), airsim.Quaternionr()),
                airsim.Vector3r(*PACKAGE_SIZE), physics_enabled=True)
            _, impact = wait_until_rest(client, pkg, cam)
            # hedefi drone kendi seçti; değerlendirmede ilk temasa en yakın gerçek araba hedef sayılır
            car, dist = nearest_car(client, scene["cars"], impact[0], impact[1])
            kind = classify_impact(client, scene["cars"], car, impact)  # hedef / agac_vb / yer
            hit = kind == "hedef"
            deliveries.append({"no": len(deliveries) + 1, "isabet": hit, "ilk_temas": kind,
                               "en_yakin_araba": car, "merkeze_m": round(dist, 2), "guven": round(conf, 2),
                               "zaman_s": round(time.time() - t_start, 1)})
            label = {"hedef": "ARABAYA ÇARPTI ✔", "agac_vb": "ağaca/nesneye çarptı ✘", "yer": "yere düştü ✘",
                     "baska_araba": "ARABAYA ÇARPTI ✔"}[kind]
            print(f"  paket {len(deliveries)}: {label} (en yakın araba merkezine {dist:.2f} m)")
            client.simDestroyObject(pkg)
            fly_mission.hide_preview()

        print("\nGörev tamam, iniş noktasına dönülüyor")
        lx, ly = cfg["landing"]
        client.moveToPositionAsync(lx, ly, z, args.hiz).join()
        landed_on_marker = aruco_land(client, cam)
        dp = client.getMultirotorState().kinematics_estimated.position
        landing_err = math.hypot(dp.x_val - lx, dp.y_val - ly)  # helipad iniş noktasında
        print(f"İniş: {'işaretçiyle' if landed_on_marker else 'işaretçisiz'}, helipad merkezine {landing_err:.2f} m")
    finally:
        client.enableApiControl(False)

    total = time.time() - t_start
    hits = sum(d["isabet"] for d in deliveries)
    unique = len({d["en_yakin_araba"] for d in deliveries if d["isabet"]})
    print(f"\n=== OTONOM GÖREV SONUCU ===")
    print(f"Teslimat: {len(deliveries)}  |  isabet: {hits}/{len(deliveries)}  |  farklı araba: {unique}"
          f"  |  yanlış alarm (araba olmayan yere): {len(deliveries) - hits}")
    print(f"Toplam süre (kalkıştan inişe): {total:.0f} s")
    out = ROOT / "results" / f"autonomous_{datetime.now():%Y%m%d_%H%M%S}.json"
    out.write_text(json.dumps({"irtifa_m": args.irtifa, "model": args.model, "sure_s": round(total, 1),
                               "inis_hatasi_m": None if landing_err is None else round(landing_err, 2),
                               "teslimatlar": deliveries}, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Kaydedildi: {out}")


if __name__ == "__main__":
    main()
