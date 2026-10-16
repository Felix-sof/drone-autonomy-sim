"""Taşınabilir otonom görev çekirdeği: ara -> tespit et -> hizalan -> bırak -> devam et -> in.

Simülatörü hiç bilmez; sadece şu arayüzleri kullanır, böylece aynı kod AirSim'de, PX4 SITL'de
ve gerçek drone'da çalışır:
    vehicle   vehicle.py (AirSimVehicle / Px4Vehicle)
    grab()    BGR kamera karesi döndüren fonksiyon
    detector  perception.CarDetector
    payload   payload.py (SimPayload / GripperPayload)
    on_drop   isteğe bağlı: bırakmadan sonra çağrılır (simülasyonda isabet değerlendirmesi)

Koordinatlar yerel NED (kalkış noktası orijin), irtifa zeminden.
"""
import math
import time

import fly_mission
from aruco_land import aruco_land_vehicle
from perception import box_center, pixel_to_ground

LANE_SPACING_M = 35   # 25 m'de kamera ~50 m genişlik görür; şeritler örtüşsün
SEARCH_CONF = 0.5     # aramada "araba var" demek için gereken güven
TRACK_CONF = 0.3      # kilitlendikten sonra takipte yeterli güven
NEW_TARGET_M = 8.0    # daha önce gidilen/bırakılan bir yere bu kadar yakın tespitler yok sayılır
MAX_APPROACHES = 12   # güvenlik: bir görevde en fazla bu kadar hedefe yaklaşılır
ALIGN_TOL_M = 0.3     # hizalama toleransı (yerde, metre); PX4 tepki gecikmesiyle ±0.4 m salınım var, araba 5x2 m
MAX_MISSES = 10       # takipte hedef bu kadar ardışık karede görünmezse vazgeç
FENCE_MARGIN_M = 15   # görev alanının bu kadar dışına çıkılırsa görev iptal (geofence)
MIN_BATTERY = 0.25    # kalan pil bunun altına düşerse eve dön
MAX_MISSION_S = 900   # görev süresi sınırı (pil süresine göre ayarla)


class Abort(Exception):
    """Güvenlik ihlali: görev bırakılır, eve dönülüp inilir."""


def safety_check(vehicle, area, t_start):
    n, e, _ = vehicle.position()
    if not (area[0] - FENCE_MARGIN_M <= n <= area[1] + FENCE_MARGIN_M and
            area[2] - FENCE_MARGIN_M <= e <= area[3] + FENCE_MARGIN_M):
        raise Abort(f"geofence dışına çıkıldı ({n:.0f}, {e:.0f})")
    b = vehicle.battery()
    if b is not None and b < MIN_BATTERY:
        raise Abort(f"pil düşük (%{b * 100:.0f})")
    if time.time() - t_start > MAX_MISSION_S:
        raise Abort(f"görev süresi aşıldı ({MAX_MISSION_S} s)")


def lawnmower(area, spacing, alt):
    """Alanı x (kuzey) boyunca gidip gelen şeritlerle kaplayan NED noktaları."""
    pts, flip = [], False
    e = area[2] + spacing / 3
    while e <= area[3] + 1e-6:
        ns = (area[0], area[1]) if not flip else (area[1], area[0])
        pts += [(ns[0], e, -alt), (ns[1], e, -alt)]
        flip = not flip
        e += spacing
    return pts


def ground_estimates(vehicle, img, boxes):
    """Tespit kutularını (eğim telafili) dünya konumuna çevirir: [(n, e, güven), ...]"""
    n0, e0, _ = vehicle.position()
    roll, pitch, yaw = vehicle.attitude()
    height = vehicle.height_agl()
    out = []
    for b in boxes:
        g = pixel_to_ground(img.shape, *box_center(b), height, roll, pitch, yaw)
        if g is not None:
            out.append((n0 + g[0], e0 + g[1], b[4]))
    return out


def align_over_target(vehicle, grab, detector, timeout=40, show=True):
    """En yakın arabaya kilitlenip (takip) tam üstüne gelene kadar NED hız komutuyla kayar."""
    t0 = time.time()
    track, misses, good = None, 0, 0
    vn_f = ve_f = 0.0  # yumuşatılmış komut
    while time.time() - t0 < timeout:
        img = grab()
        roll, pitch, yaw = vehicle.attitude()
        h, w = img.shape[:2]
        boxes = detector.detect(img, conf=TRACK_CONF)
        ref = track if track is not None else (w / 2, h / 2)
        best = min(boxes, key=lambda b: math.dist(box_center(b), ref), default=None)
        if best is not None and track is not None and math.dist(box_center(best), track) > 0.12 * w:
            best = None  # başka arabaya sıçrama
        if best is None:
            misses += 1
            vehicle.hover()
            if misses > MAX_MISSES:
                return False
            time.sleep(0.1)
            continue
        misses = 0
        track = box_center(best)
        if show:
            fly_mission.show(img, [best])
        g = pixel_to_ground(img.shape, *track, vehicle.height_agl(), roll, pitch, yaw)
        if g is None:
            continue
        dn, de = g
        err = math.hypot(dn, de)
        good = good + 1 if err < ALIGN_TOL_M else 0
        if good >= 3:
            vehicle.hover()
            return True
        sp = min(1.5, 0.5 * err)
        vn, ve = (dn / err * sp, de / err * sp) if err > 1e-6 else (0.0, 0.0)
        vn_f, ve_f = 0.5 * vn_f + 0.5 * vn, 0.5 * ve_f + 0.5 * ve   # ani yön değişimini yumuşat
        vehicle.velocity_ned(vn_f, ve_f, 0.0)
        time.sleep(0.12)
    vehicle.hover()
    return False


def search_and_deliver(vehicle, grab, detector, payload, area, alt=25.0, deliveries=3, speed=5.0,
                       landing=(0.0, 0.0), on_drop=None, log=print):
    """Tam görev. (teslimatlar, işaretçiyle_indi_mi, süre_s, iptal_sebebi) döndürür."""
    route = lawnmower(area, LANE_SPACING_M, alt)
    log(f"Arama deseni: {len(route) // 2} şerit, irtifa {alt:.0f} m")
    visited, dropped, results = [], [], []
    approaches = 0
    t_start = time.time()
    abort_reason = None

    vehicle.arm_takeoff(alt)
    idx = 0
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
                boxes = [b for b in detector.detect(img) if b[4] >= SEARCH_CONF]
                fly_mission.show(img, boxes)
                fresh = [g for g in ground_estimates(vehicle, img, boxes)
                         if all(math.hypot(g[0] - vn, g[1] - ve) > NEW_TARGET_M for vn, ve in visited)]
                if fresh:
                    target = max(fresh, key=lambda g: g[2])
            if target is None:
                break

            approaches += 1
            tn, te, conf = target
            visited.append((tn, te))
            log(f"\nAraba bulundu (güven {conf:.2f}) ~({tn:.1f}, {te:.1f}) -> yaklaşılıyor")
            vehicle.goto(tn, te, -alt, 4.0)
            safety_check(vehicle, area, t_start)
            if not align_over_target(vehicle, grab, detector):
                log("  yaklaşınca kayboldu, aramaya devam")
                continue
            vehicle.wait_still(max_speed=0.05)
            n, e, _ = vehicle.position()
            visited.append((n, e))  # tahmin ile hizalanılan yer farklı olabilir: ikisi de işaretli
            if any(math.hypot(n - dn, e - de) < NEW_TARGET_M for dn, de in dropped):
                log("  bu araba zaten teslim aldı, bırakılmadı; aramaya devam")
                continue
            dropped.append((n, e))
            handle = payload.release()
            rec = {"no": len(results) + 1, "n": round(n, 2), "e": round(e, 2), "guven": round(conf, 2),
                   "zaman_s": round(time.time() - t_start, 1)}
            if on_drop is not None:
                rec.update(on_drop(handle) or {})
            results.append(rec)
            log(f"  paket {rec['no']} bırakıldı" + (f": {rec.get('sonuc_metni', '')}" if on_drop else ""))
            fly_mission.hide_preview()
    except Abort as ex:
        abort_reason = str(ex)
        fly_mission.hide_preview()
        log(f"\nGÜVENLİK: {abort_reason} -> görev iptal, eve dönülüyor")

    if abort_reason is None:
        log("\nGörev tamam, iniş noktasına dönülüyor")
    vehicle.goto(landing[0], landing[1], -alt, speed)
    marker = aruco_land_vehicle(vehicle, grab)
    return results, marker, time.time() - t_start, abort_reason
