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
    ap.add_argument("--matris", action="store_true",
                    help="8 yön x (5, 8) m/s ölç, yöne bağlı 2x2 matris kalibrasyonu yap (wind_from_tilt_matrix)")
    ap.add_argument("--kaydet", action="store_true", help="sonucu mission.json'a yaz (wind_k_d / wind_matrix)")
    ap.add_argument("--tablo", action="store_true",
                    help="36 yön (10°'de bir) x 5 m/s ölç, tablo (look-up) kalibrasyonu yap; --kaydet ile wind_lut")
    args = ap.parse_args()
    winds = WINDS
    if args.tablo:
        winds = [(round(5 * math.cos(math.radians(a)), 2), round(5 * math.sin(math.radians(a)), 2))
                 for a in range(0, 360, 10)]
    elif args.matris:
        winds = [(round(s * math.cos(math.radians(a)), 2), round(s * math.sin(math.radians(a)), 2))
                 for s in (5.0, 8.0) for a in range(0, 360, 30)]   # 45°'nin katları sin4θ'yı ölçemez
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    load_mission()
    client = airsim.MultirotorClient()
    client.confirmConnection()
    v = AirSimVehicle(client, ground_z=scene["ground_z_ned"])
    rows = []
    try:
        v.arm_takeoff(args.irtifa)
        for wn, we in winds:
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
        if args.matris:
            samples = [(ballistics.tilt_feature(r, p, y), (wn, we)) for wn, we, _, _, r, p, y in rows]
            M = ballistics.fit_wind_matrix(samples)
            print(f"yöne bağlı matris M = [[{M[0][0]:.2f}, {M[0][1]:.2f}], [{M[1][0]:.2f}, {M[1][1]:.2f}]]")
            e1, e2 = [], []
            for i, ((un, ue), (wn, we)) in enumerate(samples):   # bir-dışarıda (leave-one-out) doğrulama
                Mi = ballistics.fit_wind_matrix(samples[:i] + samples[i + 1:])
                mn, me = Mi[0][0] * un + Mi[0][1] * ue, Mi[1][0] * un + Mi[1][1] * ue
                sn, se = ballistics.wind_from_tilt(rows[i][4], rows[i][5], rows[i][6], k_mean)
                e1.append(math.hypot(sn - wn, se - we))
                e2.append(math.hypot(mn - wn, me - we))
            e3 = []
            hrows = [(r, p, y, wn, we) for wn, we, _, _, r, p, y in rows]
            for i in range(len(hrows)):
                prm_i = ballistics.fit_wind_harmonic(hrows[:i] + hrows[i + 1:])
                hn, he = ballistics.wind_from_tilt_harmonic(*hrows[i][:3], prm_i)
                e3.append(math.hypot(hn - hrows[i][3], he - hrows[i][4]))
            prm = ballistics.fit_wind_harmonic(hrows)
            print(f"rüzgar vektörü hatası (görülmemiş rüzgarlarda): tek katsayı ort {sum(e1) / len(e1):.2f} / en kötü "
                  f"{max(e1):.2f}  |  matris ort {sum(e2) / len(e2):.2f} / en kötü {max(e2):.2f}  |  açıya bağlı ort "
                  f"{sum(e3) / len(e3):.2f} / en kötü {max(e3):.2f} m/s")
            print(f"açıya bağlı parametreler: {prm}")
            out = ROOT / "results" / f"wind_calib_{time.strftime('%Y%m%d_%H%M%S')}.json"
            out.write_text(json.dumps({"rows": hrows, "harmonic": prm, "matrix": M, "k_d": k_mean}, indent=1),
                           encoding="utf-8")
        if args.tablo:
            trows = [(r, p, y, wn, we) for wn, we, _, _, r, p, y in rows]
            lut = ballistics.fit_wind_lut(trows)
            e_k, e_l = [], []
            for i in range(len(trows)):          # bir-dışarıda: o açı tablodan çıkarılıp komşulardan tahmin
                lut_i = ballistics.fit_wind_lut(trows[:i] + trows[i + 1:])
                ln, le = ballistics.wind_from_tilt_lut(*trows[i][:3], lut_i)
                kn, ke = ballistics.wind_from_tilt(*trows[i][:3], k_mean)
                e_l.append(math.hypot(ln - trows[i][3], le - trows[i][4]))
                e_k.append(math.hypot(kn - trows[i][3], ke - trows[i][4]))
            print(f"tablo: {len(lut['theta_e'])} nokta; görülmemiş açılarda hata: tek katsayı ort {sum(e_k) / len(e_k):.2f}"
                  f" / en kötü {max(e_k):.2f}  |  tablo ort {sum(e_l) / len(e_l):.2f} / en kötü {max(e_l):.2f} m/s")
            out = ROOT / "results" / f"wind_lut_{time.strftime('%Y%m%d_%H%M%S')}.json"
            out.write_text(json.dumps({"rows": trows, "lut": lut}, indent=1), encoding="utf-8")
        if args.kaydet:
            p_cfg = ROOT / "mission" / "mission.json"
            cfg = json.loads(p_cfg.read_text(encoding="utf-8"))
            cfg["wind_k_d"] = round(k_mean, 6)
            if args.matris:
                cfg["wind_harmonic"] = prm          # açıya bağlı (X gövde) kalibrasyon
            if args.tablo:
                cfg["wind_lut"] = lut               # tablo kalibrasyonu (öncelikli)
            p_cfg.write_text(json.dumps(cfg, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print("mission.json güncellendi")
    finally:
        client.simSetWind(airsim.Vector3r(0, 0, 0))
        v.land()
        v.release()


if __name__ == "__main__":
    main()
