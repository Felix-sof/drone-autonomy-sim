"""Kayıt için demo turu: 5 biyomu gezer, canlı tespiti gösterir, bir teslimat noktasına bırakıp iner.

  bash mission/sim.sh airsim
  python mission/demo_tour.py --model C:/UE/drone_runs/carperson3/weights/best.pt

Ekranda iki pencere: Unreal (drone'u arkadan takip eden kamera) ve "drone kamerası" önizlemesi
(model kutuları: kırmızı araba, sarı kişi; sol üstte biyom ve irtifa). İkisini yan yana kaydet.
Sadece AirSim (SimpleFlight) modunda çalışır; görev mantığı autonomy.py ile aynıdır.
"""
import argparse
import json
import math
import time

import cv2
import numpy as np

import drop_test
import perception
from aruco_land import aruco_land_vehicle
from autonomy import search_and_deliver
from collect_data import area, scatter_cars, scatter_people
from payload import SimPayload
from perception import CarDetector
from sim_client import ROOT, airsim, get_rgb, load_mission
from vehicle import AirSimVehicle
from world import BIOMES, randomize_world

WIN = "drone kamerasi"
COLORS = {0: (0, 0, 255), 1: (0, 220, 255)}   # BGR: araba kırmızı, kişi sarı


def overlay(img, boxes, lines, width=960):
    """Önizleme: küçültülmüş kare + tespit kutuları + bilgi satırları."""
    s = width / img.shape[1]
    vis = cv2.resize(img, (width, int(img.shape[0] * s)))
    for x1, y1, x2, y2, conf, cls in boxes:
        c = COLORS.get(int(cls), (255, 255, 255))
        cv2.rectangle(vis, (int(x1 * s), int(y1 * s)), (int(x2 * s), int(y2 * s)), c, 2)
        cv2.putText(vis, f"{'kisi' if int(cls) == 1 else 'araba'} {conf:.2f}", (int(x1 * s), int(y1 * s) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1)
    for i, t in enumerate(lines):
        cv2.putText(vis, t, (12, 30 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(vis, t, (12, 30 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.imshow(WIN, vis)
    cv2.waitKey(1)


def fly_and_show(client, vehicle, det, cam, pts, alt, speed, title):
    """Noktaları sırayla uçar; yol boyunca canlı tespiti gösterir."""
    for n, e in pts:
        vehicle.start_goto(n, e, -alt, speed)
        while vehicle.distance_to(n, e) > 2.0:
            img = get_rgb(client, cam)
            boxes = det.detect(img, conf=0.35, classes=[0, 1])
            people = sum(1 for b in boxes if int(b[5]) == 1)
            overlay(img, boxes, [title, f"irtifa {vehicle.height_agl():.0f} m | araba {len(boxes) - people} | kisi {people}"])
    vehicle.hover()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--irtifa", type=float, default=25.0)
    ap.add_argument("--hiz", type=float, default=6.0)
    ap.add_argument("--biyomlar", default=",".join(BIOMES), help="virgüllü liste")
    ap.add_argument("--teslimat", type=int, default=1, help="turun sonunda kaç teslimat noktasına bırakılsın")
    args = ap.parse_args()

    cfg = load_mission()
    perception.CAMERA_STABILIZED = cfg.get("camera_gimbal", False)
    perception.CAMERA_YAW_LOCK = None if cfg.get("camera_yaw_lock_deg") is None else math.radians(cfg["camera_yaw_lock_deg"])
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    G = scene["ground_z_ned"]
    drop_test.GROUND_Z = G
    cam = cfg["camera"]
    client = airsim.MultirotorClient()
    client.confirmConnection()
    vehicle = AirSimVehicle(client, ground_z=G)
    det = CarDetector(args.model, imgsz=1280, default_classes=(0, 1))
    rng = np.random.default_rng()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 960, 540)

    try:
        vehicle.arm_takeoff(args.irtifa)
        a = area(scene)
        for b in args.biyomlar.split(","):
            randomize_world(client, scene, rng, b, landing=cfg["landing"])
            placed = scatter_cars(client, scene, cfg["landing"], rng)
            scatter_people(client, scene, rng, list(placed or []))
            time.sleep(1.0)
            # biyom başına kısa bir tur: alanın içinden rastgele 3 nokta
            pts = [(rng.uniform(a[0] + 10, a[1] - 10), rng.uniform(a[2] + 10, a[3] - 10)) for _ in range(3)]
            fly_and_show(client, vehicle, det, cam, pts, args.irtifa, args.hiz, f"biyom: {b}")

        if args.teslimat > 0:   # son biyomda: görev çekirdeğiyle teslimat + işaretçiyle iniş
            import fly_mission   # çekirdeğin önizlemesini aynı büyük pencereye yönlendir (kayıt kopmasın)
            fly_mission.SHOW_PREVIEW = True
            fly_mission.show = lambda img, dets: overlay(
                img, [(*d[:4], d[4] if len(d) > 4 and isinstance(d[4], float) else 1.0,
                       d[5] if len(d) > 5 else 0) for d in dets],
                ["teslimat gorevi (25 m)", f"irtifa {vehicle.height_agl():.0f} m"])
            fly_mission.hide_preview = lambda: None
            payload = SimPayload(client)
            results, marker, total, abort = search_and_deliver(
                vehicle, lambda: get_rgb(client, cam), CarDetector(args.model, imgsz=960), payload, a,
                alt=args.irtifa, deliveries=args.teslimat, speed=5.0, landing=tuple(cfg["landing"]),
                on_drop=lambda pkg, mid: {"sonuc_metni": f"isaretci #{mid}"})
            print(f"Teslimat: {len(results)}, iniş {'işaretçiyle' if marker else 'işaretçisiz'}")
        else:
            vehicle.goto(cfg["landing"][0], cfg["landing"][1], -args.irtifa, args.hiz)
            aruco_land_vehicle(vehicle, lambda: get_rgb(client, cam))
    finally:
        vehicle.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
