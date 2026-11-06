"""Otonom görevi simülasyonda çalıştırır: uçuşu AirSim ya da PX4 yapar, görev çekirdeği aynı.

  python mission/run_autonomy.py --model C:/UE/drone_runs/car_forest2/weights/best.pt              # AirSim
  python mission/run_autonomy.py --model ... --backend px4                                        # PX4 SITL

PX4 modunda: airsim/settings_px4.json yüklü olmalı, Unreal Play açık, PX4 SITL WSL'de çalışıyor.
Kamera karesi, paketin fiziği ve isabet değerlendirmesi her iki modda AirSim'den gelir;
gerçek drone'da bunların yerini kamera sürücüsü ve GripperPayload alır.
"""
import argparse
import json
import math
from datetime import datetime

import numpy as np

import drop_test
import fly_mission
from autonomy import search_and_deliver
from collect_data import area, scatter_cars, scatter_people
from drop_test import classify_impact, wait_until_rest
from payload import SimPayload
from perception import CarDetector
from sim_client import ROOT, airsim, get_rgb, load_mission


def nearest_car(client, cars, x, y):
    return min(((c, math.hypot(client.simGetObjectPose(c).position.x_val - x,
                               client.simGetObjectPose(c).position.y_val - y)) for c in cars), key=lambda t: t[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--backend", choices=["airsim", "px4"], default="airsim")
    ap.add_argument("--px4-address", default="udp://:14550")
    ap.add_argument("--teslimat", type=int, default=3)
    ap.add_argument("--irtifa", type=float, default=25.0)
    ap.add_argument("--hiz", type=float, default=5.0)
    ap.add_argument("--goster", action="store_true")
    ap.add_argument("--ruzgar", default="0,0", help="sabit rüzgar, m/s: kuzey,doğu (ör. 6,3)")
    ap.add_argument("--isaretcisiz", type=float, default=0.0,
                    help="dropzone modunda araçların bu oranının teslimat işaretçisini kaldır (negatif test)")
    ap.add_argument("--mod", choices=["roof", "dropzone", "sar"], default="roof",
                    help="roof (varsayılan): aracın tavanına; dropzone: aracın yanındaki ArUco teslimat noktasına; "
                         "sar: kişi ara, güvenli mesafede yardım paketi bırak (iki sınıflı model)")
    ap.add_argument("--hedef", choices=["araba", "insan", "ikisi"], default=None,
                    help="ne aranacak: araba (--mod roof/dropzone), insan (= --mod sar), ikisi (karma görev: "
                         "araba bulunca tavana, kişi bulunca 5 m+ uzağa yardım paketi; öncelik kişide)")
    ap.add_argument("--imgsz", type=int, default=960, help="tespit çözünürlüğü (sar için 1280 önerilir)")
    ap.add_argument("--teslim", choices=["ip", "birak"], default="birak",
                    help="sar modunda: birak = 5 m+ uzağa serbest bırakma (varsayılan), ip = kişinin yanına ipli uzatma (sonraki aşama)")
    ap.add_argument("--paket-fizigi", choices=["basit", "gercekci"], default="basit",
                    help="gercekci: düşen pakete hava direnci + rüzgar (AirSim'in fiziği uygulamıyor)")
    ap.add_argument("--ruzgar-telafi", choices=["yok", "bilinen", "egim"], default="yok",
                    help="dropzone'da bırakma noktasını rüzgara karşı kaydır: bilinen = simülasyonun gerçek "
                         "rüzgarı (hesap testi), egim = drone'un eğiminden tahmin (gerçek hayattaki yol)")
    ap.add_argument("--biyom", default="sabit",
                    help="görev dünyası: sabit (kurulumdaki yaz ormanı) | rastgele | yaz_ormani | kis_ormani | "
                         "seyrek_orman | col | kayalik")
    ap.add_argument("--birakma-irtifa", type=float, default=None,
                    help="isteğe bağlı: hedefin üstünde bu irtifaya (m) alçalıp bırak (ör. 5). "
                         "Verilmezse seyir irtifasından (en yüksekten) bırakılır")
    args = ap.parse_args()
    if args.hedef in ("insan", "ikisi") and args.imgsz == 960:
        args.imgsz = 1280          # kişiler küçük: 1280 px tespit
    if args.hedef == "insan":
        args.mod = "sar"
    elif args.hedef == "araba" and args.mod == "sar":
        ap.error("--hedef araba ile --mod sar birlikte olmaz")
    fly_mission.SHOW_PREVIEW = args.goster
    drop_alt = args.birakma_irtifa
    if drop_alt is None:
        drop_alt = 0.0   # varsayılan: seyir irtifasından bırak (alçalma yok)
    drop_alt = drop_alt if drop_alt > 0 else None

    cfg = load_mission()
    import perception
    perception.CAMERA_STABILIZED = cfg.get("camera_gimbal", False)
    import autonomy
    autonomy.RELEASE_BIAS_NE = tuple(cfg.get("release_bias_ne_m", (0.0, 0.0)))
    perception.CAMERA_YAW_LOCK = (None if cfg.get("camera_yaw_lock_deg") is None
                                  else __import__("math").radians(cfg["camera_yaw_lock_deg"]))
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    G = scene["ground_z_ned"]
    drop_test.GROUND_Z = G
    cam = cfg["camera"]

    client = airsim.MultirotorClient()
    client.confirmConnection()
    if args.backend == "airsim":
        from vehicle import AirSimVehicle
        vehicle = AirSimVehicle(client, ground_z=G)
    else:
        from vehicle import Px4Vehicle
        print(f"PX4'e bağlanılıyor ({args.px4_address})...")
        vehicle = Px4Vehicle(args.px4_address)

    rng_scene = np.random.default_rng()
    if args.biyom != "sabit":
        from world import randomize_world
        randomize_world(client, scene, rng_scene, None if args.biyom == "rastgele" else args.biyom,
                        landing=cfg["landing"])
    placed = scatter_cars(client, scene, cfg["landing"], rng_scene)  # her görev yeni sahne
    if scene.get("people"):
        scatter_people(client, scene, rng_scene, list(placed or []))
    removed = []
    if args.isaretcisiz > 0 and "dropzones" in scene:   # bu araçlara teslimat yapılmamalı
        rng_rm = np.random.default_rng()
        for i in rng_rm.choice(len(scene["dropzones"]), size=round(len(scene["dropzones"]) * args.isaretcisiz),
                               replace=False):
            client.simSetObjectPose(scene["dropzones"][i], airsim.Pose(airsim.Vector3r(0, 0, 50), airsim.Quaternionr()), True)
            removed.append(int(i) + 1)
        print(f"İşaretçisi kaldırılan araçlar (teslimat YAPILMAMALI): {sorted(removed)}")
    wn, we = (float(x) for x in args.ruzgar.split(","))
    client.simSetWind(airsim.Vector3r(wn, we, 0))   # drone dinamiğini etkiler (paket fiziğini değil)
    if wn or we:
        print(f"Rüzgar: kuzey {wn} m/s, doğu {we} m/s ({math.hypot(wn, we):.1f} m/s)")
    detector = CarDetector(args.model, imgsz=args.imgsz)
    from payload import TetherPayload
    if args.mod == "sar" and args.teslim == "ip":
        payload = TetherPayload(client, G)
    elif args.paket_fizigi == "gercekci":
        from drop_test import on_car
        from payload import DragPayload
        roof = scene["car_height_m"]
        payload = DragPayload(client, G, (wn, we), car_hit=lambda x, y, z: z > G - roof - 0.05 and any(
            on_car(client, c, (x, y, z)) for c in scene["cars"]))
    else:
        payload = SimPayload(client)
    import autonomy
    if args.ruzgar_telafi == "bilinen":
        autonomy.WIND_COMP = ("known", (wn, we))
    elif args.ruzgar_telafi == "egim":
        # yöne bağlı kalibrasyon varsa onu kullan (wind_calibrate.py --matris), yoksa tek katsayı
        if cfg.get("wind_lut"):
            autonomy.WIND_COMP = ("tilt_lut", cfg["wind_lut"])
        elif cfg.get("wind_harmonic"):
            autonomy.WIND_COMP = ("tilt_harmonic", cfg["wind_harmonic"])
        elif cfg.get("wind_matrix"):
            autonomy.WIND_COMP = ("tilt_matrix", cfg["wind_matrix"])
        else:
            autonomy.WIND_COMP = ("tilt", cfg.get("wind_k_d", 0.00065))

    def on_drop(pkg, marker_id=None):  # sadece değerlendirme: Unreal'in gerçek konumlarıyla
        drone = client.simGetVehiclePose().position     # bırakma anındaki gerçek drone konumu
        if getattr(payload, "impact", None) is not None:   # gerçekçi fizik: çarpma noktası hesaptan
            impact, drop_test.LAST_IMPACT_SPEED = payload.impact, payload.last_speed
        else:
            _, impact = wait_until_rest(client, pkg, cam)
        client.simDestroyObject(pkg)
        if args.mod == "dropzone":
            # teslimat noktası: levhanın (2.67 m kare) içine düştü mü, araca çarptı mı
            pad = client.simGetObjectPose(scene["dropzones"][marker_id - 1]).position
            dx, dy = impact[0] - pad.x_val, impact[1] - pad.y_val
            half = scene["dropzone_marker_m"] / 2 + 0.335
            on_pad = abs(dx) <= half and abs(dy) <= half
            car, _ = nearest_car(client, scene["cars"], impact[0], impact[1])
            hit_car = classify_impact(client, scene["cars"], car, impact) in ("hedef", "baska_araba")
            ok = on_pad and not hit_car
            text = ("TESLİMAT NOKTASINA ✔" if ok else ("araca çarptı ✘" if hit_car else "noktanın dışına ✘"))
            # hata ayrıştırması: drone işaretçiye göre neredeydi (hizalama) + paket nereye düştü (düşüş)
            return {"carpma_ofset_ne_m": [round(dx, 3), round(dy, 3)],
                    "drone_ofset_ne_m": [round(drone.x_val - pad.x_val, 3), round(drone.y_val - pad.y_val, 3)],
                    "dusus_kaymasi_ne_m": [round(impact[0] - drone.x_val, 3), round(impact[1] - drone.y_val, 3)],
                    "isabet": ok, "araba": car, "isaretci_merkezine_m": round(math.hypot(dx, dy), 2),
                    "carpma_hizi_ms": drop_test.LAST_IMPACT_SPEED,
                    "araca_carpti": hit_car, "sonuc_metni": f"{text} (işaretçi #{marker_id} merkezine "
                                                         f"{math.hypot(dx, dy):.2f} m)"}
        car, dist = nearest_car(client, scene["cars"], impact[0], impact[1])
        kind = classify_impact(client, scene["cars"], car, impact)
        text = {"hedef": "ARABAYA ÇARPTI ✔", "baska_araba": "ARABAYA ÇARPTI ✔",
                "agac_vb": "ağaca/nesneye çarptı ✘", "yer": "yere düştü ✘"}[kind]
        return {"isabet": kind in ("hedef", "baska_araba"), "ilk_temas": kind, "araba": car,
                "merkeze_m": round(dist, 2), "sonuc_metni": f"{text} (merkeze {dist:.2f} m)"}

    def body_distance(pp, x, y):
        """(x, y)'nin kişinin gövdesine yatay uzaklığı: ayaktaysa dik eksen, yatıyorsa ayak->baş çizgisi."""
        pose = client.simGetObjectPose(pp)
        p, q = pose.position, pose.orientation
        w, qx, qy, qz = q.w_val, q.x_val, q.y_val, q.z_val
        # NED "yukarı" (0,0,-1) vektörünü kişinin yönelimiyle döndür: yatanda yatay olur
        ux = -(2 * (qx * qz + w * qy))
        uy = -(2 * (qy * qz - w * qx))
        L = 1.75 * math.hypot(ux, uy)     # gövdenin yere izdüşümü (ayakta ~0)
        if L < 0.2:
            return math.hypot(x - p.x_val, y - p.y_val)
        ax, ay = ux / math.hypot(ux, uy), uy / math.hypot(ux, uy)
        t = max(0.0, min(L, (x - p.x_val) * ax + (y - p.y_val) * ay))
        return math.hypot(x - (p.x_val + t * ax), y - (p.y_val + t * ay))

    def on_tether_sar(pkg):   # ipli uzatma: paket kişinin kol mesafesinde mi, üstünde mi
        pos = client.simGetObjectPose(pkg).position
        client.simDestroyObject(pkg)
        dists = [body_distance(pp, pos.x_val, pos.y_val) for pp in scene["people"]]
        nearest = min(dists)
        reach = 0.3 <= nearest <= 1.6
        text = ("KOL MESAFESİNE ✔" if reach else
                ("kişinin üstüne çok yakın ✘" if nearest < 0.3 else "uzanamayacağı yerde ✘"))
        return {"isabet": reach, "tehlikeli": nearest < 0.3, "kisiye_m": round(nearest, 2),
                "carpma_hizi_ms": payload.last_speed, "ip_m": payload.line_m,
                "sonuc_metni": f"{text} (gövdeye {nearest:.2f} m, iniş hızı {payload.last_speed} m/s)"}

    def on_drop_sar(pkg):  # sadece değerlendirme: kişilere gerçek mesafe
        import sar
        _, impact = wait_until_rest(client, pkg, cam)
        client.simDestroyObject(pkg)
        dists = [math.hypot(client.simGetObjectPose(pp).position.x_val - impact[0],
                            client.simGetObjectPose(pp).position.y_val - impact[1]) for pp in scene["people"]]
        nearest = min(dists)
        car, _ = nearest_car(client, scene["cars"], impact[0], impact[1])
        kind = classify_impact(client, scene["cars"], car, impact)   # yer / ağaç / araç
        dangerous = nearest < sar.SAFE_MIN_M
        ok = (not dangerous) and nearest <= 12.0 and kind == "yer"
        text = ("TEHLİKELİ: kişiye çok yakın ✘" if dangerous else
                ("GÜVENLİ TESLİMAT ✔" if ok else ("ağaca/araca düştü ✘" if kind != "yer" else "kişiye çok uzak ✘")))
        return {"isabet": ok, "tehlikeli": dangerous, "carpma_hizi_ms": drop_test.LAST_IMPACT_SPEED,
                "kisiye_m": round(nearest, 2), "zemin": kind,
                "araba": car, "sonuc_metni": f"{text} (en yakın kişiye {nearest:.1f} m)"}

    if args.hedef == "ikisi":
        from mixed import mixed_mission

        def on_drop_mixed(pkg, kind):
            return on_drop(pkg) if kind == "araba" else on_drop_sar(pkg)
        try:
            results, marker, total, abort_reason = mixed_mission(
                vehicle, lambda: get_rgb(client, cam), detector, payload, area(scene),
                alt=args.irtifa, deliveries=args.teslimat, speed=args.hiz,
                landing=tuple(cfg["landing"]), on_drop=on_drop_mixed)
        finally:
            vehicle.release()
        cars = [r for r in results if r.get("hedef_turu") == "araba"]
        ppl = [r for r in results if r.get("hedef_turu") == "kisi"]
        print(f"\n=== KARMA GÖREV ({args.backend.upper()}) ===")
        print(f"Araba: {len(cars)} paket, tavana isabet {sum(r.get('isabet', False) for r in cars)}/{len(cars)}  |  "
              f"Kişi: {len(ppl)} yardım paketi, güvenli {sum(r.get('isabet', False) for r in ppl)}/{len(ppl)}, "
              f"TEHLİKELİ {sum(r.get('tehlikeli', False) for r in ppl)}")
        print(f"Toplam süre: {total:.0f} s" + (f"  |  İPTAL: {abort_reason}" if abort_reason else ""))
        out = ROOT / "results" / f"karma_{args.backend}_{datetime.now():%Y%m%d_%H%M%S}.json"
        out.write_text(json.dumps({"backend": args.backend, "model": args.model, "sure_s": round(total, 1),
                                   "biyom": scene.get("biome", "sabit"), "paketler": results},
                                  indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Kaydedildi: {out}")
        return

    if args.mod == "sar":
        import sar
        try:
            results, marker, total, abort_reason = sar.sar_mission(
                vehicle, lambda: get_rgb(client, cam), detector, payload, area(scene),
                alt=args.irtifa, packages=args.teslimat, speed=args.hiz,
                landing=tuple(cfg["landing"]),
                on_drop=on_tether_sar if args.teslim == "ip" else on_drop_sar, drop_alt=drop_alt,
                method="tether" if args.teslim == "ip" else "drop")
        finally:
            vehicle.release()
        p = client.simGetVehiclePose().position
        landing_err = math.hypot(p.x_val - cfg["landing"][0], p.y_val - cfg["landing"][1])
        ok = sum(r.get("isabet", False) for r in results)
        bad = sum(r.get("tehlikeli", False) for r in results)
        print(f"\n=== SAR GÖREVİ ({args.backend.upper()}) ===")
        print(f"Yardım paketi ({'ipli uzatma' if args.teslim == 'ip' else 'serbest bırakma'}): {len(results)}  |  "
              f"başarılı: {ok}/{len(results)}  |  TEHLİKELİ: {bad}")
        if results and args.teslim == "ip":
            ds = [r["kisiye_m"] for r in results]
            print(f"Paket-gövde mesafesi: ort {sum(ds) / len(ds):.2f} m, en az {min(ds):.2f}, en çok {max(ds):.2f}")
        print(f"İniş: {'ArUco işaretçisiyle' if marker else 'işaretçisiz'}, helipad merkezine {landing_err:.2f} m")
        print(f"Toplam süre: {total:.0f} s" + (f"  |  İPTAL: {abort_reason}" if abort_reason else ""))
        out = ROOT / "results" / f"sar_{args.backend}_{datetime.now():%Y%m%d_%H%M%S}.json"
        out.write_text(json.dumps({"backend": args.backend, "model": args.model, "sure_s": round(total, 1),
                                   "birakma_irtifa_m": drop_alt, "teslim": args.teslim,
                                   "biyom": scene.get("biome", "sabit"),
                                   "paketler": results}, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Kaydedildi: {out}")
        return

    try:
        results, marker, total, abort_reason = search_and_deliver(
            vehicle, lambda: get_rgb(client, cam), detector, payload, area(scene),
            alt=args.irtifa, deliveries=args.teslimat, speed=args.hiz,
            landing=tuple(cfg["landing"]), on_drop=on_drop, mode=args.mod, drop_alt=drop_alt)
    finally:
        vehicle.release()

    p = client.simGetVehiclePose().position   # iniş değerlendirmesi (AirSim koordinatı)
    landing_err = math.hypot(p.x_val - cfg["landing"][0], p.y_val - cfg["landing"][1])
    hits = sum(r.get("isabet", False) for r in results)
    unique = len({r.get("isaretci") or r["araba"] for r in results if r.get("isabet")})
    drop_txt = f"{drop_alt:.0f} m" if drop_alt else f"{args.irtifa:.0f} m serbest"
    print(f"\n=== OTONOM GÖREV ({args.backend.upper()}, mod: {args.mod}, bırakma: {drop_txt}) ===")
    speeds = [r["carpma_hizi_ms"] for r in results if r.get("carpma_hizi_ms") is not None]
    if speeds:
        print(f"Paket çarpma hızı: ort {sum(speeds) / len(speeds):.1f} m/s, en fazla {max(speeds):.1f} m/s")
    print(f"Teslimat: {len(results)}  |  isabet: {hits}/{len(results)}  |  farklı araba: {unique}"
          f"  |  yanlış alarm: {len(results) - hits}")
    print(f"İniş: {'ArUco işaretçisiyle' if marker else 'işaretçisiz'}, helipad merkezine {landing_err:.2f} m")
    print(f"Toplam süre: {total:.0f} s" + (f"  |  İPTAL: {abort_reason}" if abort_reason else ""))
    if removed:
        wrong = [r for r in results if r.get("isaretci") in removed]
        print(f"İşaretçisiz araca yapılan teslimat: {len(wrong)} (0 olmalı)")
    out = ROOT / "results" / f"autonomy_{args.backend}_{datetime.now():%Y%m%d_%H%M%S}.json"
    client.simSetWind(airsim.Vector3r(0, 0, 0))
    out.write_text(json.dumps({"backend": args.backend, "ruzgar": args.ruzgar, "model": args.model, "irtifa_m": args.irtifa,
                               "birakma_irtifa_m": drop_alt, "mod": args.mod,
                               "paket_fizigi": args.paket_fizigi, "ruzgar_telafi": args.ruzgar_telafi,
                               "sure_s": round(total, 1), "inis_hatasi_m": round(landing_err, 2),
                               "inis_isaretci": marker, "iptal": abort_reason, "teslimatlar": results},
                              indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Kaydedildi: {out}")


if __name__ == "__main__":
    main()
