"""Arama-kurtarma (SAR) görevi: kayıp/yaralı kişiyi bul -> yardım paketini ulaştır -> aramaya devam -> in.

İki teslim yöntemi:
  "tether" (uzanma, önerilen): drone kişinin YANINDA (gövde kenarından HANDOVER_GAP_M ötede) alçak
      irtifada durur, paket ip ucunda ~0.5 m/s ile kişinin kol mesafesine iner. Mahsur kalmış /
      hareket edemeyen / selde suyla çevrili kişiye ulaşır; serbest düşen cisim kişiye nişanlanmaz.
  "drop" (serbest bırakma): paket kişiden en az SAFE_MIN_M uzakta açık zemine bırakılır.

TEMEL GÜVENLİK KURALI: paket hiçbir kişiye SAFE_MIN_M'den yakın bırakılmaz.
  - bırakma noktası kişinin çevresindeki çemberden, diğer kişi/araçlardan uzak ve görüntüde
    "açık zemin" görünen (ağaç/gölge değil) aday seçilerek belirlenir
  - bırakmadan hemen önce yeni karede altındaki bölge yeniden kontrol edilir; bir kişi
    yaklaşmışsa paket BIRAKILMAZ
Bırakma irtifası (drop_alt, öneri 8 m; Skydio Dropper kılavuzu ~7.5 m): bırakma noktasına dikey
alçalınır, alçalırken her karede alt bölge yeniden kontrol edilir.
Gerçek sistemde ek olarak: pilotun onayı, yumuşak/paraşütlü paket, mesafe sensörü.

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
DESCENT_SPEED = 1.5
HANDOVER_GAP_M = 0.8      # ipli paket kişinin gövde kenarından bu kadar ötede (kol mesafesi)
HANDOVER_ALT_M = 8.0      # ipli indirmede bekleme irtifası (Wing ~7 m)
HANDOVER_TOL_M = 0.25     # drone'un hedef noktaya yatay hizalama toleransı
HANDOVER_CLEAR_M = 1.0    # aday nokta başka kişi/araç gövdesine en az bu kadar uzak


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


def box_half_extent_m(img_shape, box, height, hfov_deg=90.0):
    """Tespit kutusunun yerdeki yarı boyları (kuzey, doğu) metre (gimbal'lı, yaw kuzeye kilitli)."""
    f = (img_shape[1] / 2) / math.tan(math.radians(hfov_deg / 2))
    return (box[3] - box[1]) / 2 * height / f, (box[2] - box[0]) / 2 * height / f


def choose_handover_point(vehicle, img, detector, box):
    """Kişinin yanında, gövde kenarından HANDOVER_GAP_M ötede, açık ve başka kimseye/araca yakın
    olmayan bir nokta: (Δn, Δe) kişinin kutu merkezine göre, açıklık skoru. Yoksa None."""
    h = vehicle.height_agl()
    roll, pitch, yaw = vehicle.attitude()
    center = pixel_to_ground(img.shape, *box_center(box), h, roll, pitch, yaw)
    if center is None:
        return None
    half_n, half_e = box_half_extent_m(img.shape, box, h)
    others = []
    for b in detector.detect(img, conf=0.3, classes=[CAR, PERSON]):
        if b is box or math.dist(box_center(b), box_center(box)) < 3:
            continue
        g = pixel_to_ground(img.shape, *box_center(b), h, roll, pitch, yaw)
        if g is not None:
            on, oe = box_half_extent_m(img.shape, b, h)
            others.append((g[0] - center[0], g[1] - center[1], max(on, oe)))
    f = (img.shape[1] / 2) / math.tan(math.radians(45))
    best = None
    for k in range(N_CANDIDATES):
        a = 2 * math.pi * k / N_CANDIDATES
        r = abs(math.cos(a)) * half_n + abs(math.sin(a)) * half_e + HANDOVER_GAP_M
        cn, ce = r * math.cos(a), r * math.sin(a)
        if any(math.hypot(cn - on, ce - oe) < rad + HANDOVER_CLEAR_M for on, oe, rad in others):
            continue
        px, py = ground_to_pixel(img.shape, center[0] + cn, center[1] + ce, h)
        score = openness(img, px, py, radius_px=0.5 / h * f)
        if best is None or score > best[2]:
            best = (cn, ce, score)
    return best


def hover_beside_person(vehicle, grab, detector, offset, alt, timeout=60):
    """Kişiyi takip ederek, kişi + offset noktasının üstünde `alt` irtifaya alçalır. None = hazır."""
    t0, track, misses, good = time.time(), None, 0, 0
    vn_f = ve_f = 0.0
    while time.time() - t0 < timeout:
        img = grab()
        roll, pitch, yaw = vehicle.attitude()
        h = vehicle.height_agl()
        H, W = img.shape[:2]
        boxes = detector.detect(img, conf=0.25, classes=[PERSON])
        f = (W / 2) / math.tan(math.radians(45))
        # ilk karede kişi, merkezden offset kadar uzakta görünmeli
        ref = track if track is not None else (W / 2 - offset[1] / h * f, H / 2 + offset[0] / h * f)
        b = min(boxes, key=lambda b: math.dist(box_center(b), ref), default=None)
        if b is not None and math.dist(box_center(b), ref) > 0.15 * W:
            b = None
        if b is None:
            misses += 1
            vehicle.hover()
            if misses > MAX_MISSES:
                return "alçalırken kişi görüş dışında kaldı"
            time.sleep(0.1)
            continue
        misses, track = 0, box_center(b)
        fly_mission.show(img, [b])
        g = pixel_to_ground(img.shape, *track, h, roll, pitch, yaw)
        if g is None:
            continue
        dn, de = g[0] + offset[0], g[1] + offset[1]     # hedef noktaya göre drone'un hatası
        err = math.hypot(dn, de)
        left = h - alt
        good = good + 1 if (abs(left) < 0.2 and err < HANDOVER_TOL_M) else 0
        if good >= 3:
            vehicle.hover()
            return None
        sp = min(1.2, 0.5 * err)
        vn, ve = (dn / err * sp, de / err * sp) if err > 1e-6 else (0.0, 0.0)
        vn_f, ve_f = 0.5 * vn_f + 0.5 * vn, 0.5 * ve_f + 0.5 * ve
        centered = err < max(HANDOVER_TOL_M, 0.04 * h)
        vd = min(DESCENT_SPEED, 0.6 * left) if (centered and left > 0) else (-0.5 if left < -0.5 else 0.0)
        vehicle.velocity_ned(vn_f, ve_f, vd)
        time.sleep(0.12)
    vehicle.hover()
    return "alçalma zaman aşımı"


def zone_clear(vehicle, img, detector):
    """Drone'un tam altı (bırakma noktası) kişilerden SAFE_MIN_M uzakta mı?"""
    for on, oe, cls in objects_on_ground(vehicle, img, detector.detect(img, conf=0.25, classes=[PERSON])):
        if math.hypot(on, oe) < SAFE_MIN_M:
            return False
    return True


def descend_clear(vehicle, grab, detector, target_alt, timeout=40):
    """Bulunduğu yerde target_alt'a dikey alçalır; her karede altı kontrol eder. True = hazır."""
    n, e, _ = vehicle.position()
    t0 = time.time()
    while time.time() - t0 < timeout:
        img = grab()
        if not zone_clear(vehicle, img, detector):
            vehicle.hover()
            return False
        h = vehicle.height_agl()
        left = h - target_alt
        if abs(left) < 0.2:
            vehicle.hover()
            return True
        cn, ce, _ = vehicle.position()   # yatayda yerinde kal
        vehicle.velocity_ned(0.5 * (n - cn), 0.5 * (e - ce), max(-0.5, min(DESCENT_SPEED, 0.6 * left)))
        time.sleep(0.12)
    vehicle.hover()
    return False


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
            return img, b
        sp = min(1.5, 0.5 * err)
        vehicle.velocity_ned(dn / err * sp if err > 1e-6 else 0, de / err * sp if err > 1e-6 else 0, 0.0)
        time.sleep(0.12)
    return None


def _tether_delivery(vehicle, grab, detector, payload, img, pbox, alt, approve, log):
    """Kişinin yanına alçal, ipli paketi uzat. Kayıt sözlüğü (+ '_handle') ya da None."""
    pt = choose_handover_point(vehicle, img, detector, pbox)
    if pt is None:
        log("  kişinin yanında uygun nokta yok (başka kişi/araç çok yakın), uzatılmadı")
        return None
    dn, de, score = pt
    log(f"  uzatma noktası: kişinin {math.hypot(dn, de):.1f} m yanında, açıklık {score:.2f}; "
        f"{HANDOVER_ALT_M:.0f} m'ye alçalınıyor")
    why = hover_beside_person(vehicle, grab, detector, (dn, de), HANDOVER_ALT_M)
    if why is not None:
        log(f"  {why}, uzatılmadı; yükseliyor")
        vehicle.goto(*vehicle.position()[:2], -alt, 3.0)
        return None
    vehicle.wait_still(max_speed=0.05)
    if approve is not None and not approve(grab(), f"SAR ipli uzatma, {vehicle.height_agl():.1f} m"):
        log("  pilot onay vermedi, uzatılmadı")
        vehicle.goto(*vehicle.position()[:2], -alt, 3.0)
        return None
    h = vehicle.height_agl()
    handle = payload.release()       # ipli: yere indirip bırakır (bekler)
    return {"yontem": "ip", "aciklik": round(score, 2), "bekleme_irtifa_m": round(h, 2),
            "hedef_kaydirma_m": round(math.hypot(dn, de), 2), "_handle": handle}


def sar_mission(vehicle, grab, detector, payload, area, alt=25.0, packages=3, speed=5.0,
                landing=(0.0, 0.0), on_drop=None, log=print, drop_alt=None, approve=None, method="drop",
                points=None):
    """method: "tether" (kişinin yanına ipli uzatma) | "drop" (güvenli mesafeye serbest bırakma).
    drop_alt: "drop"ta bu irtifaya (zeminden, m) alçalıp bırakır; None = seyir irtifası."""
    import autonomy
    route = autonomy.mission_route(area, alt, points)
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
            seen = align_over_person(vehicle, grab, detector)
            if seen is None:
                log("  kişi yaklaşınca görülemedi, aramaya devam")
                continue
            img, pbox = seen
            pn, pe, _ = vehicle.position()
            found[-1] = (pn, pe)
            if method == "tether":
                rec = _tether_delivery(vehicle, grab, detector, payload, img, pbox, alt, approve, log)
                if rec is None:
                    continue
                n, e, _ = vehicle.position()
                rec.update({"no": len(results) + 1, "kisi_tahmini": [round(pn, 2), round(pe, 2)],
                            "birakma": [round(n, 2), round(e, 2)], "zaman_s": round(time.time() - t_start, 1)})
                if on_drop is not None:
                    rec.update(on_drop(rec.pop("_handle")) or {})
                rec.pop("_handle", None)
                vehicle.goto(n, e, -alt, 3.0)
                results.append(rec)
                log(f"  yardım paketi {rec['no']} uzatıldı" + (f": {rec.get('sonuc_metni', '')}" if on_drop else ""))
                fly_mission.hide_preview()
                continue
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
            if drop_alt is not None and drop_alt < alt - 0.5:
                log(f"  {drop_alt:.0f} m'ye alçalınıyor (her karede alt kontrol ediliyor)")
                if not descend_clear(vehicle, grab, detector, drop_alt) or not zone_clear(vehicle, grab(), detector):
                    log("  alçalırken altta kişi görüldü/alçalınamadı, BIRAKILMADI; yükseliyor")
                    vehicle.goto(*vehicle.position()[:2], -alt, 3.0)
                    continue
                vehicle.wait_still(max_speed=0.05)
            if approve is not None and not approve(grab(), f"SAR yardım paketi, {vehicle.height_agl():.1f} m"):
                log("  pilot onay vermedi, BIRAKILMADI")
                vehicle.goto(*vehicle.position()[:2], -alt, 3.0)
                continue
            handle = payload.release()
            n, e, _ = vehicle.position()
            rec = {"no": len(results) + 1, "kisi_tahmini": [round(pn, 2), round(pe, 2)],
                   "birakma": [round(n, 2), round(e, 2)], "aciklik": round(score, 2),
                   "zaman_s": round(time.time() - t_start, 1), "birakma_irtifa_m": round(vehicle.height_agl(), 2)}
            if on_drop is not None:
                rec.update(on_drop(handle) or {})
            if vehicle.height_agl() < alt - 0.5:
                vehicle.goto(n, e, -alt, 3.0)
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
