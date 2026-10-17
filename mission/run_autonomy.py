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
    ap.add_argument("--mod", choices=["dropzone", "roof", "sar"], default="dropzone",
                    help="dropzone: aracın yanındaki ArUco teslimat noktasına; roof: araç üstüne; "
                         "sar: kişi ara, güvenli mesafede yardım paketi bırak (iki sınıflı model)")
    ap.add_argument("--imgsz", type=int, default=960, help="tespit çözünürlüğü (sar için 1280 önerilir)")
    ap.add_argument("--biyom", default="sabit",
                    help="görev dünyası: sabit (kurulumdaki yaz ormanı) | rastgele | yaz_ormani | kis_ormani | "
                         "seyrek_orman | col | kayalik")
    ap.add_argument("--birakma-irtifa", type=float, default=None,
                    help="hedefin üstünde bu irtifaya (m) alçalıp bırak; varsayılan dropzone 5, sar 8; "
                         "0 = seyir irtifasından serbest bırak (eski davranış)")
    args = ap.parse_args()
    fly_mission.SHOW_PREVIEW = args.goster
    drop_alt = args.birakma_irtifa
    if drop_alt is None:
        drop_alt = 8.0 if args.mod == "sar" else (5.0 if args.mod == "dropzone" else 0.0)
    drop_alt = drop_alt if drop_alt > 0 else None

    cfg = load_mission()
    import perception
    perception.CAMERA_STABILIZED = cfg.get("camera_gimbal", False)
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
    payload = SimPayload(client)

    def on_drop(pkg, marker_id=None):  # sadece değerlendirme: Unreal'in gerçek konumlarıyla
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
            return {"isabet": ok, "araba": car, "isaretci_merkezine_m": round(math.hypot(dx, dy), 2),
                    "carpma_hizi_ms": drop_test.LAST_IMPACT_SPEED,
                    "araca_carpti": hit_car, "sonuc_metni": f"{text} (işaretçi #{marker_id} merkezine "
                                                         f"{math.hypot(dx, dy):.2f} m)"}
        car, dist = nearest_car(client, scene["cars"], impact[0], impact[1])
        kind = classify_impact(client, scene["cars"], car, impact)
        text = {"hedef": "ARABAYA ÇARPTI ✔", "baska_araba": "ARABAYA ÇARPTI ✔",
                "agac_vb": "ağaca/nesneye çarptı ✘", "yer": "yere düştü ✘"}[kind]
        return {"isabet": kind in ("hedef", "baska_araba"), "ilk_temas": kind, "araba": car,
                "merkeze_m": round(dist, 2), "sonuc_metni": f"{text} (merkeze {dist:.2f} m)"}

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

    if args.mod == "sar":
        import sar
        try:
            results, marker, total, abort_reason = sar.sar_mission(
                vehicle, lambda: get_rgb(client, cam), detector, payload, area(scene),
                alt=args.irtifa, packages=args.teslimat, speed=args.hiz,
                landing=tuple(cfg["landing"]), on_drop=on_drop_sar, drop_alt=drop_alt)
        finally:
            vehicle.release()
        p = client.simGetVehiclePose().position
        landing_err = math.hypot(p.x_val - cfg["landing"][0], p.y_val - cfg["landing"][1])
        ok = sum(r.get("isabet", False) for r in results)
        bad = sum(r.get("tehlikeli", False) for r in results)
        print(f"\n=== SAR GÖREVİ ({args.backend.upper()}) ===")
        print(f"Yardım paketi: {len(results)}  |  güvenli teslimat: {ok}/{len(results)}  |  TEHLİKELİ: {bad}")
        print(f"İniş: {'ArUco işaretçisiyle' if marker else 'işaretçisiz'}, helipad merkezine {landing_err:.2f} m")
        print(f"Toplam süre: {total:.0f} s" + (f"  |  İPTAL: {abort_reason}" if abort_reason else ""))
        out = ROOT / "results" / f"sar_{args.backend}_{datetime.now():%Y%m%d_%H%M%S}.json"
        out.write_text(json.dumps({"backend": args.backend, "model": args.model, "sure_s": round(total, 1),
                                   "birakma_irtifa_m": drop_alt,
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
                               "sure_s": round(total, 1), "inis_hatasi_m": round(landing_err, 2),
                               "inis_isaretci": marker, "iptal": abort_reason, "teslimatlar": results},
                              indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Kaydedildi: {out}")


if __name__ == "__main__":
    main()
