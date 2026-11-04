"""Rüzgar tahmini kalibrasyonu: drone havada dururken bilinen rüzgarlarda eğimini ölçer.

  bash mission/sim.sh airsim
  python mission/wind_calibrate.py            # AirSim uçuş kontrolcüsü

Model (ballistics.wind_from_tilt): tan(eğim) = k_d * v^2. Her rüzgar için k_d hesaplanır, ortalaması
mission.json "wind_k_d" önerisi olarak yazdırılır; ayrıca o k_d ile tahmin edilen rüzgar gerçeğiyle
karşılaştırılır (hız ve yön hatası). Gerçek drone'da aynı ölçüm PX4 ile, bilinen rüzgarlı bir günde ya da
yerdeki anemometreyle yapılır.
"""
import argparse
import json
import math
import time

import ballistics
from sim_client import ROOT, airsim, load_mission
from vehicle import AirSimVehicle

WINDS = [(0, 3), (0, 5), (0, 8), (5, 0), (-4, 3), (3, -6)]   # (kuzey, doğu) m/s


def mean_attitude(vehicle, seconds=4.0):
    rs, ps, ys = [], [], []
    t0 = time.time()
    while time.time() - t0 < seconds:
        r, p, y = vehicle.attitude()
        rs.append(r)
        ps.append(p)
        ys.append(y)
        time.sleep(0.05)
    return sum(rs) / len(rs), sum(ps) / len(ps), math.atan2(sum(map(math.sin, ys)), sum(map(math.cos, ys)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--irtifa", type=float, default=25.0)
    ap.add_argument("--bekle", type=float, default=8.0, help="her rüzgarda oturma süresi (s)")
    args = ap.parse_args()
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    load_mission()
    client = airsim.MultirotorClient()
    client.confirmConnection()
    v = AirSimVehicle(client, ground_z=scene["ground_z_ned"])
    rows = []
    try:
        v.arm_takeoff(args.irtifa)
        for wn, we in WINDS:
            client.simSetWind(airsim.Vector3r(wn, we, 0))
            v.goto(0, 0, -args.irtifa, 3.0)      # aynı noktada dur (kontrolcü rüzgara karşı eğilir)
            v.hover()
            time.sleep(args.bekle)
            r, p, y = mean_attitude(v)
            n, e, tilt = ballistics.tilt_vector_ne(r, p, y)
            speed = math.hypot(wn, we)
            k = ballistics.calibrate_k(tilt, speed)
            rows.append((wn, we, math.degrees(tilt), k, r, p, y))
            print(f"rüzgar ({wn:+.0f},{we:+.0f}) {speed:.1f} m/s: eğim {math.degrees(tilt):.2f}°, k_d {k:.5f}, "
                  f"eğim yönü (n,e)=({n:+.3f},{e:+.3f})")
        k_mean = sum(r[3] for r in rows) / len(rows)
        print(f"\nönerilen wind_k_d = {k_mean:.5f}")
        print("bu k_d ile tahmin (gerçek -> tahmin):")
        for wn, we, _, _, r, p, y in rows:
            en, ee = ballistics.wind_from_tilt(r, p, y, k_mean)
            err_s = math.hypot(en, ee) - math.hypot(wn, we)
            err_d = math.degrees(math.atan2(math.sin(math.atan2(ee, en) - math.atan2(we, wn)),
                                            math.cos(math.atan2(ee, en) - math.atan2(we, wn))))
            print(f"  ({wn:+.1f},{we:+.1f}) -> ({en:+.1f},{ee:+.1f})  hız hatası {err_s:+.2f} m/s, yön hatası {err_d:+.0f}°")
    finally:
        client.simSetWind(airsim.Vector3r(0, 0, 0))
        v.land()
        v.release()


if __name__ == "__main__":
    main()
