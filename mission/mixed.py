"""Karma görev (--hedef ikisi): tek aramada hem araba hem kişi aranır.

  araba bulunursa  -> havada durup paketi aracın TAVANINA bırakır (autonomy: ince hizalama, rüzgar telafisi)
  kişi bulunursa   -> yardım paketini kişiden en az SAFE_MIN_M uzağa, açık zemine bırakır (sar)
  ikisi birden     -> öncelik kişide (arama-kurtarmada önce insan)

Kişi kuralı her durumda geçerli: araca bırakırken de düşme noktasının PERSON_CLEAR_M içinde kimse olmamalı.
Yordamların hepsi autonomy.py ve sar.py'den; bu modül sadece aramayı ve sırayı yönetir.
"""
import math
import time

import autonomy
import fly_mission
import sar
from aruco_land import aruco_land_vehicle
from autonomy import (Abort, LANE_SPACING_M, MAX_APPROACHES, NEW_TARGET_M, PERSON_CLEAR_M, SEARCH_CONF,
                      align_over_target, estimate_wind, fine_align_target, ground_estimates, lawnmower,
                      people_near, safety_check)

CAR, PERSON = 0, 1


def pick_target(estimates, visited):
    """[(n, e, güven, sınıf)] içinden daha önce gidilmemiş en iyi hedef; kişiler önce."""
    fresh = [g for g in estimates if all(math.hypot(g[0] - vn, g[1] - ve) > NEW_TARGET_M for vn, ve in visited)]
    people = [g for g in fresh if g[3] == PERSON]
    pool = people or fresh
    return max(pool, key=lambda g: g[2]) if pool else None


def _car_drop(vehicle, grab, detector, payload, alt, log, approve=None):
    """Arabanın tavanına: takip ederek hizalan, ince hizala (rüzgar telafisiyle), kişi kontrolü, bırak."""
    if not align_over_target(vehicle, grab, detector):
        log("  yaklaşınca araba kayboldu, aramaya devam")
        return None
    target, wind, off = (0.0, 0.0), None, (0.0, 0.0)
    if autonomy.WIND_COMP is not None:
        import ballistics
        wind = estimate_wind(vehicle)
        off = ballistics.release_offset(max(0.5, vehicle.height_agl() - 0.7), wind)
        target = (-off[0], -off[1])
        log(f"  rüzgar {math.hypot(*wind):.1f} m/s -> bırakma noktası {math.hypot(*off):.1f} m rüzgara karşı kaydırıldı")
    fine = fine_align_target(vehicle, grab, detector, target=target)
    if fine is None:
        log("  ince hizalamada araba görülemedi, bırakılmadı")
        return None
    if people_near(vehicle, grab(), detector, (-off[0], -off[1])):
        log(f"  düşme noktasının {PERSON_CLEAR_M:.0f} m yakınında kişi var, BIRAKILMADI")
        return None
    if approve is not None and not approve(grab(), "araba tavanına teslimat"):
        log("  pilot onay vermedi, BIRAKILMADI")
        return None
    rec = {"hedef_turu": "araba", "kamera_hedef_ne_m": [round(fine[0], 3), round(fine[1], 3)]}
    if wind is not None:
        rec["ruzgar_tahmini_ne"] = [round(wind[0], 2), round(wind[1], 2)]
    rec["_handle"] = payload.release()
    return rec


def _person_drop(vehicle, grab, detector, payload, alt, log, approve=None):
    """Kişiye yardım: üstüne gel (konumu iyileştir), güvenli açık nokta seç, oraya git, alanı kontrol et, bırak."""
    seen = sar.align_over_person(vehicle, grab, detector)
    if seen is None:
        log("  yaklaşınca kişi görülemedi, aramaya devam")
        return None
    img, _ = seen
    pn, pe, _ = vehicle.position()
    drop = sar.choose_drop_point(vehicle, img, detector, (0.0, 0.0))
    if drop is None:
        log("  güvenli bırakma noktası bulunamadı, BIRAKILMADI")
        return None
    dn, de, score = drop
    log(f"  bırakma noktası: kişiden {sar.DROP_RADIUS_M} m, açıklık {score:.2f}")
    vehicle.goto(pn + dn, pe + de, -alt, 2.0, tol=0.4)
    vehicle.wait_still(max_speed=0.05)
    if not sar.zone_clear(vehicle, grab(), detector):
        log("  bırakma noktasına kişi yaklaştı, BIRAKILMADI")
        return None
    if approve is not None and not approve(grab(), "kişiye yardım paketi (güvenli mesafe)"):
        log("  pilot onay vermedi, BIRAKILMADI")
        return None
    return {"hedef_turu": "kisi", "kisi_tahmini": [round(pn, 2), round(pe, 2)], "aciklik": round(score, 2),
            "_handle": payload.release()}


def mixed_mission(vehicle, grab, detector, payload, area, alt=25.0, deliveries=3, speed=5.0,
                  landing=(0.0, 0.0), on_drop=None, log=print, approve=None):
    """on_drop(handle, tür) -> dict; tür 'araba' ya da 'kisi'. (sonuçlar, işaretçiyle_indi, süre, iptal) döner."""
    route = lawnmower(area, LANE_SPACING_M, alt)
    log(f"Karma arama (araba + kişi): {len(route) // 2} şerit, irtifa {alt:.0f} m")
    visited, results = [], []
    t_start, abort_reason, approaches, idx = time.time(), None, 0, 0
    vehicle.arm_takeoff(alt)
    try:
        while idx < len(route) and len(results) < deliveries and approaches < MAX_APPROACHES:
            vehicle.start_goto(*route[idx], speed)
            target = None
            while target is None:
                safety_check(vehicle, area, t_start)
                if vehicle.distance_to(*route[idx][:2]) < 3:
                    idx += 1
                    if idx >= len(route):
                        break
                    vehicle.start_goto(*route[idx], speed)
                img = grab()
                boxes = [b for b in detector.detect(img, classes=[CAR, PERSON])
                         if b[4] >= (sar.SEARCH_CONF if int(b[5]) == PERSON else SEARCH_CONF)]
                fly_mission.show(img, boxes)
                est = [(g[0], g[1], g[2], int(b[5])) for g, b in zip(ground_estimates(vehicle, img, boxes), boxes)]
                target = pick_target(est, visited)
            if target is None:
                break
            approaches += 1
            tn, te, conf, cls = target
            visited.append((tn, te))
            kind = "kisi" if cls == PERSON else "araba"
            log(f"\n{'Kişi' if cls == PERSON else 'Araba'} bulundu (güven {conf:.2f}) ~({tn:.1f}, {te:.1f}) -> yaklaşılıyor")
            vehicle.goto(tn, te, -alt, 4.0)
            safety_check(vehicle, area, t_start)
            rec = (_person_drop if cls == PERSON else _car_drop)(vehicle, grab, detector, payload, alt, log, approve)
            n, e, _ = vehicle.position()
            visited.append((n, e))
            if rec is None:
                continue
            handle = rec.pop("_handle")
            rec.update({"no": len(results) + 1, "n": round(n, 2), "e": round(e, 2), "guven": round(conf, 2),
                        "zaman_s": round(time.time() - t_start, 1)})
            if on_drop is not None:
                rec.update(on_drop(handle, kind) or {})
            results.append(rec)
            log(f"  paket {rec['no']} ({kind}) bırakıldı" + (f": {rec.get('sonuc_metni', '')}" if on_drop else ""))
            fly_mission.hide_preview()
    except Abort as ex:
        abort_reason = str(ex)
        log(f"\nGÜVENLİK: {abort_reason} -> görev iptal, eve dönülüyor")
    if abort_reason is None:
        log("\nKarma görev tamam, iniş noktasına dönülüyor")
    vehicle.goto(landing[0], landing[1], -alt, speed)
    marker = aruco_land_vehicle(vehicle, grab)
    return results, marker, time.time() - t_start, abort_reason
