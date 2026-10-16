"""Arama-kurtarma (SAR) görevi: kayıp/yaralı kişiyi bul -> güvenli mesafede açık bir noktaya
yardım paketi bırak -> aramaya devam -> in.

TEMEL GÜVENLİK KURALI: paket hiçbir kişiye SAFE_MIN_M'den yakın bırakılmaz.
  - bırakma noktası kişinin çevresindeki çemberden, diğer kişi/araçlardan uzak ve görüntüde
    "açık zemin" görünen (ağaç/gölge değil) aday seçilerek belirlenir
  - bırakmadan hemen önce yeni karede altındaki bölge yeniden kontrol edilir; bir kişi
    yaklaşmışsa paket BIRAKILMAZ
Gerçek sistemde ek olarak: pilotun onayı, düşük bırakma irtifası, yumuşak/paraşütlü paket.

Kamera: stabilizasyonlu gimbal, yaw kuzeye kilitli varsayılır (perception.CAMERA_STABILIZED).
"""
import math
import time

import cv2
import numpy as np

import fly_mission
import perception
from aruco_land import aruco_land_vehicle
from autonomy import Abort, LANE_SPACING_M, MAX_MISSES, lawnmower, safety_check
from perception import box_center, pixel_to_ground

CAR, PERSON = 0, 1
SEARCH_CONF = 0.45
SAFE_MIN_M = 5.0          # kişiye en az bu kadar uzak
DROP_RADIUS_M = 6.5       # aday noktaların kişiye uzaklığı
OTHERS_MIN_M = 4.0        # diğer kişi/araçlara en az bu kadar uzak
N_CANDIDATES = 16
NEW_PERSON_M = 8.0


def ground_to_pixel(shape, dn, de, height, hfov_deg=90.0):
    """Yerdeki (Δkuzey, Δdoğu) noktanın görüntüdeki pikseli (gimbal'lı, yaw kuzeye kilitli kamera)."""
    assert perception.CAMERA_STABILIZED and perception.CAMERA_YAW_LOCK is not None
    h, w = shape[:2]
    f = (w / 2) / math.tan(math.radians(hfov_deg / 2))
    yaw = perception.CAMERA_YAW_LOCK
    fwd = dn * math.cos(yaw) + de * math.sin(yaw)
    right = -dn * math.sin(yaw) + de * math.cos(yaw)
    return w / 2 + right / height * f, h / 2 - fwd / height * f


def openness(img, px, py, radius_px):
    """Noktanın etrafı ne kadar açık zemin? (0..1) Ağaç/çalı çok kenar içerir, açık toprak az."""
    h, w = img.shape[:2]
    x0, x1 = int(max(0, px - radius_px)), int(min(w, px + radius_px))
    y0, y1 = int(max(0, py - radius_px)), int(min(h, py + radius_px))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return 0.0  # görüntü dışında: bilinmiyor
    patch = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(patch, 60, 150)
    dark = (patch < 60).mean()           # koyu gölge/ağaç tacı
    return float(max(0.0, 1.0 - 4.0 * (edges > 0).mean() - dark))


def objects_on_ground(vehicle, img, boxes):
    roll, pitch, yaw = vehicle.attitude()
    h = vehicle.height_agl()
    out = []
    for b in boxes:
        g = pixel_to_ground(img.shape, *box_center(b), h, roll, pitch, yaw)
        if g is not None:
            out.append((g[0], g[1], int(b[5])))
    return out


def choose_drop_point(vehicle, img, detector, person_rel):
    """Kişinin çevresinde güvenli ve açık bir bırakma noktası (drone'a göre Δn, Δe) ya da None."""
    objs = objects_on_ground(vehicle, img, detector.detect(img, conf=0.3, classes=[CAR, PERSON]))
    h = vehicle.height_agl()
    f = (img.shape[1] / 2) / math.tan(math.radians(45))
    best = None
    for k in range(N_CANDIDATES):
        a = 2 * math.pi * k / N_CANDIDATES
        cn, ce = person_rel[0] + DROP_RADIUS_M * math.cos(a), person_rel[1] + DROP_RADIUS_M * math.sin(a)
        ok = True
        for on, oe, cls in objs:
            d = math.hypot(cn - on, ce - oe)
            if (cls == PERSON and d < SAFE_MIN_M) or (cls == CAR and d < OTHERS_MIN_M):
                ok = False
                break
        if not ok:
            continue
        px, py = ground_to_pixel(img.shape, cn, ce, h)
        score = openness(img, px, py, radius_px=1.5 / h * f)
        if best is None or score > best[2]:
            best = (cn, ce, score)
    return best


def zone_clear(vehicle, img, detector):
    """Drone'un tam altı (bırakma noktası) kişilerden SAFE_MIN_M uzakta mı?"""
    for on, oe, cls in objects_on_ground(vehicle, img, detector.detect(img, conf=0.25, classes=[PERSON])):
        if math.hypot(on, oe) < SAFE_MIN_M:
            return False
    return True


def align_over_person(vehicle, grab, detector, timeout=30):
    """Kişinin üstüne gelir (tespit konumunu iyileştirmek için; bırakma burada YAPILMAZ)."""
    t0, track, misses, good = time.time(), None, 0, 0
    while time.time() - t0 < timeout:
        img = grab()
        roll, pitch, yaw = vehicle.attitude()
        h, w = img.shape[:2]
        boxes = detector.detect(img, conf=0.3, classes=[PERSON])
        ref = track if track is not None else (w / 2, h / 2)
        b = min(boxes, key=lambda b: math.dist(box_center(b), ref), default=None)
        if b is not None and track is not None and math.dist(box_center(b), track) > 0.12 * w:
            b = None
        if b is None:
            misses += 1
            vehicle.hover()
            if misses > MAX_MISSES:
                return None
            time.sleep(0.1)
            continue
        misses, track = 0, box_center(b)
        fly_mission.show(img, [b])
        dn, de = pixel_to_ground(img.shape, *track, vehicle.height_agl(), roll, pitch, yaw)
        err = math.hypot(dn, de)
        good = good + 1 if err < 0.5 else 0
        if good >= 3:
            vehicle.hover()
            return img
        sp = min(1.5, 0.5 * err)
        vehicle.velocity_ned(dn / err * sp if err > 1e-6 else 0, de / err * sp if err > 1e-6 else 0, 0.0)
        time.sleep(0.12)
    return None


def sar_mission(vehicle, grab, detector, payload, area, alt=25.0, packages=3, speed=5.0,
                landing=(0.0, 0.0), on_drop=None, log=print):
    route = lawnmower(area, LANE_SPACING_M, alt)
    log(f"SAR arama deseni: {len(route) // 2} şerit, irtifa {alt:.0f} m")
    found, results = [], []
    t_start, abort_reason = time.time(), None
    vehicle.arm_takeoff(alt)
    idx = 0
    try:
        while idx < len(route) and len(results) < packages:
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
                boxes = detector.detect(img, conf=SEARCH_CONF, classes=[PERSON])
                fly_mission.show(img, boxes)
                n0, e0, _ = vehicle.position()
                for on, oe, _ in objects_on_ground(vehicle, img, boxes):
                    pn, pe = n0 + on, e0 + oe
                    if all(math.hypot(pn - fn, pe - fe) > NEW_PERSON_M for fn, fe in found):
                        target = (pn, pe)
                        break
            if target is None:
                break
            found.append(target)
            log(f"\nKişi bulundu ~({target[0]:.1f}, {target[1]:.1f}) -> üstüne gidiliyor")
            vehicle.goto(target[0], target[1], -alt, 4.0)
            img = align_over_person(vehicle, grab, detector)
            if img is None:
                log("  kişi yaklaşınca görülemedi, aramaya devam")
                continue
            pn, pe, _ = vehicle.position()
            found[-1] = (pn, pe)
            drop = choose_drop_point(vehicle, img, detector, (0.0, 0.0))
            if drop is None:
                log("  güvenli bırakma noktası bulunamadı, BIRAKILMADI")
                continue
            dn, de, score = drop
            log(f"  bırakma noktası: kişiden {DROP_RADIUS_M} m, açıklık {score:.2f}")
            vehicle.goto(pn + dn, pe + de, -alt, 2.0, tol=0.4)
            vehicle.wait_still(max_speed=0.05)
            if not zone_clear(vehicle, grab(), detector):
                log("  bırakma noktasına kişi yaklaştı, BIRAKILMADI")
                continue
            handle = payload.release()
            n, e, _ = vehicle.position()
            rec = {"no": len(results) + 1, "kisi_tahmini": [round(pn, 2), round(pe, 2)],
                   "birakma": [round(n, 2), round(e, 2)], "aciklik": round(score, 2),
                   "zaman_s": round(time.time() - t_start, 1)}
            if on_drop is not None:
                rec.update(on_drop(handle) or {})
            results.append(rec)
            log(f"  yardım paketi {rec['no']} bırakıldı" + (f": {rec.get('sonuc_metni', '')}" if on_drop else ""))
            fly_mission.hide_preview()
    except Abort as ex:
        abort_reason = str(ex)
        log(f"\nGÜVENLİK: {abort_reason} -> görev iptal, eve dönülüyor")
    if abort_reason is None:
        log("\nSAR görevi tamam, iniş noktasına dönülüyor")
    vehicle.goto(landing[0], landing[1], -alt, speed)
    marker = aruco_land_vehicle(vehicle, grab)
    return results, marker, time.time() - t_start, abort_reason
