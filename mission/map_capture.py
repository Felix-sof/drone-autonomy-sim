"""Görev planlayıcı için harita: drone'u alanın ortasında yükseğe çıkarıp aşağı bakan kamerayla tek kare çeker.

  bash mission/sim.sh airsim
  python mission/map_capture.py                 # mission/map/harita.jpg + harita.json
  python tools/gorev_planlayici.py              # haritadan planlayıcı sayfasını üretir

Kamera gimbal'lı ve kuzeye kilitli (görüntünün üstü kuzey), 90° yatay görüş: piksel (px, py) yerde
  e = merkez_e + (px - W/2) * H / f,   n = merkez_n - (py - H_px/2) * H / f,   f = (W/2) / tan(45°) = W/2
(H = zeminden yükseklik). harita.json bu bilgileri taşır; planlayıcı tıklamaları NED metreye çevirir.
Gerçek drone'da aynı dosya bir uydu/ortofoto görüntüsü ve onun koordinat bilgisiyle üretilebilir.
"""
import argparse
import json
import math
import time

import cv2

from collect_data import area
from sim_client import ROOT, airsim, get_rgb, load_mission
from vehicle import AirSimVehicle


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yukseklik", type=float, default=150.0, help="çekim yüksekliği (m); 90° görüşte yerde 2·H genişlik")
    args = ap.parse_args()
    cfg = load_mission()
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    a = area(scene)
    cn, ce = (a[0] + a[1]) / 2, (a[2] + a[3]) / 2
    client = airsim.MultirotorClient()
    client.confirmConnection()
    v = AirSimVehicle(client, ground_z=scene["ground_z_ned"])
    out = ROOT / "mission" / "map"
    out.mkdir(parents=True, exist_ok=True)
    try:
        v.arm_takeoff(25.0)
        v.goto(cn, ce, -args.yukseklik, 8.0)
        v.hover()
        time.sleep(4.0)                          # gimbal ve görüntü otursun
        img = get_rgb(client, cfg["camera"])
        n, e, _ = v.position()
        h = v.height_agl()
        H_px, W = img.shape[:2]
        (out / "harita.jpg").write_bytes(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 88])[1].tobytes())
        meta = {"merkez_n": round(n, 3), "merkez_e": round(e, 3), "yukseklik_m": round(h, 2), "hfov_deg": 90.0,
                "genislik_px": W, "yukseklik_px": H_px, "yaw_kilit_deg": cfg.get("camera_yaw_lock_deg", 0.0),
                "harita_alani": list(a), "inis": list(cfg["landing"]),
                "not": "piksel -> NED: e = merkez_e + (px - W/2)*H/f, n = merkez_n - (py - H_px/2)*H/f, f = W/2"}
        (out / "harita.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        f = (W / 2) / math.tan(math.radians(45))
        print(f"harita: {W}x{H_px} px, merkez ({n:.1f}, {e:.1f}), {h:.0f} m -> yerde {W * h / f:.0f} x {H_px * h / f:.0f} m")
    finally:
        v.goto(cfg["landing"][0], cfg["landing"][1], -40.0, 8.0)
        v.land()
        v.release()


if __name__ == "__main__":
    main()
