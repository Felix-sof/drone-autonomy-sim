"""Taşınabilir otonom görev çekirdeği: ara -> tespit et -> hizalan -> bırak -> devam et -> in.

Simülatörü hiç bilmez; sadece şu arayüzleri kullanır, böylece aynı kod AirSim'de, PX4 SITL'de
ve gerçek drone'da çalışır:
    vehicle   vehicle.py (AirSimVehicle / Px4Vehicle)
    grab()    BGR kamera karesi döndüren fonksiyon
    detector  perception.CarDetector
    payload   payload.py (SimPayload / GripperPayload)
    on_drop   isteğe bağlı: bırakmadan sonra çağrılır (simülasyonda isabet değerlendirmesi)

İki bırakma modu:
    "dropzone"  (varsayılan) araba bulunur, YANINDAKİ numaralı ArUco teslimat işaretçisine
                hizalanılır ve paket işaretçiye bırakılır; işaretçi yoksa BIRAKILMAZ.
                Teslimat drone'ları ve arama-kurtarma bu şekilde çalışır.
    "roof"      paket araba üstüne bırakılır (önceki deneyler).

Bırakma irtifası (drop_alt): gerçek teslimat drone'ları seyir irtifasından paket atmaz; hedefin
üstünde alçalıp bırakır (Amazon MK30 ~4 m, Wing ~7 m ipli vinç, Skydio Dropper ~7.5 m). drop_alt
verilirse drone işaretçiye ortalı kalarak dikey alçalır, alçalırken altta kişi görürse vazgeçer,
bırakır ve seyir irtifasına geri çıkar. 25 m'den serbest düşüşte paket ~22 m/s ile çarpar,
5 m'den ~10 m/s.

Koordinatlar yerel NED (kalkış noktası orijin), irtifa zeminden.
"""
import math
import time

import fly_mission
from aruco_land import aruco_land_vehicle, find_markers
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
FINE_TOL_M = 0.12     # bırakmadan önce ince hizalama: ortalama sapma bundan küçük olmalı
FINE_FRAMES = 8       # ince hizalamada ortalaması alınan kare sayısı
RELEASE_BIAS_NE = (0.0, 0.0)   # ölçülen sabit sapma telafisi (m): drone işaretçiye göre bu kadar
                               # kaydırılmış konumda bırakır (mission.json "release_bias_ne_m")
DESCENT_SPEED = 1.5   # m/s, bırakma için alçalma hızı
PERSON_CLEAR_M = 5.0  # alçalırken bırakma noktasına bu kadar yakın kişi görülürse bırakılmaz
DROPZONE_MAX_M = 5.5  # teslimat işaretçisi arabanın en fazla bu kadar yakınında (kendi noktası 3.8 m'de;
                      # 8 m'de bir araç komşu aracın işaretçisini seçebiliyordu)


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


def marker_offset_avg(vehicle, grab, marker_id, n=FINE_FRAMES):
    """Drone dururken n karede işaretçinin yerdeki ortalama konumu (drone'a göre Δn, Δe) ve
    karelerin yayılımı (m). İşaretçi görülemezse None."""
    pts = []
    for _ in range(n * 3):
        if len(pts) >= n:
            break
        img = grab()
        roll, pitch, yaw = vehicle.attitude()
        m = next((mk for mk in find_markers(img) if mk[0] == marker_id), None)
        if m is None:
            continue
        g = pixel_to_ground(img.shape, *m[1], vehicle.height_agl(), roll, pitch, yaw)
        if g is not None:
            pts.append(g)
    if len(pts) < max(3, n // 2):
        return None
    mn = sum(p[0] for p in pts) / len(pts)
    me = sum(p[1] for p in pts) / len(pts)
    spread = max(math.hypot(p[0] - mn, p[1] - me) for p in pts)
    return mn, me, spread


def fine_align_marker(vehicle, grab, marker_id, tries=3):
    """Bırakmadan hemen önce: dur, FINE_FRAMES karenin ortalamasıyla ölç; RELEASE_BIAS_NE dahil
    hedefe FINE_TOL_M'den uzaksa sıkı toleransla tekrar hizalan. Son ölçüm (Δn, Δe, yayılım) ya da None."""
    last = None
    for _ in range(tries):
        vehicle.wait_still(max_speed=0.05)
        last = marker_offset_avg(vehicle, grab, marker_id)
        if last is None:
            return None
        if math.hypot(last[0] - RELEASE_BIAS_NE[0], last[1] - RELEASE_BIAS_NE[1]) < FINE_TOL_M:
            return last
        align_over_marker(vehicle, grab, marker_id, tol=FINE_TOL_M * 0.8, gain=0.4)
    return last


def align_over_marker(vehicle, grab, marker_id, timeout=40, tol=ALIGN_TOL_M, gain=0.5):
    """Belirli bir ArUco işaretçisinin (RELEASE_BIAS_NE kadar kaydırılmış) üstüne gelene kadar
    NED hız komutuyla kayar."""
    t0 = time.time()
    misses, good = 0, 0
    vn_f = ve_f = 0.0
    while time.time() - t0 < timeout:
        img = grab()
        roll, pitch, yaw = vehicle.attitude()
        m = next((mk for mk in find_markers(img) if mk[0] == marker_id), None)
        if m is None:
            misses += 1
            vehicle.hover()
            if misses > MAX_MISSES:
                return False
            time.sleep(0.1)
            continue
        misses = 0
        g = pixel_to_ground(img.shape, *m[1], vehicle.height_agl(), roll, pitch, yaw)
        if g is None:
            continue
        dn, de = g[0] - RELEASE_BIAS_NE[0], g[1] - RELEASE_BIAS_NE[1]
        err = math.hypot(dn, de)
        good = good + 1 if err < tol else 0
        if good >= 3:
            vehicle.hover()
            return True
        sp = min(1.5, gain * err)
        vn, ve = (dn / err * sp, de / err * sp) if err > 1e-6 else (0.0, 0.0)
        vn_f, ve_f = 0.5 * vn_f + 0.5 * vn, 0.5 * ve_f + 0.5 * ve
        vehicle.velocity_ned(vn_f, ve_f, 0.0)
        time.sleep(0.12)
    vehicle.hover()
    return False


def descend_over_marker(vehicle, grab, marker_id, target_alt, detector=None, timeout=60):
    """İşaretçiye ortalı kalarak target_alt'a (zeminden) dikey alçalır.

    None döndürürse hazır; aksi halde vazgeçme sebebi (str). Ortalama kaçarsa alçalma durur,
    önce yeniden ortalanır; altta kişi görülürse (iki sınıflı modelde) bırakmadan vazgeçilir.
    """
    t0 = time.time()
    misses, good = 0, 0
    vn_f = ve_f = 0.0
    while time.time() - t0 < timeout:
        img = grab()
        roll, pitch, yaw = vehicle.attitude()
        h = vehicle.height_agl()
        m = next((mk for mk in find_markers(img) if mk[0] == marker_id), None)
        if m is None:
            misses += 1
            vehicle.hover()
            if misses > MAX_MISSES:
                return "alçalırken işaretçi kayboldu"
            time.sleep(0.1)
            continue
        misses = 0
        g = pixel_to_ground(img.shape, *m[1], h, roll, pitch, yaw)
        if g is None:
            continue
        dn, de = g
        if detector is not None:
            for pn, pe, _ in ground_estimates(vehicle, img, detector.detect(img, conf=0.3, classes=[1])):
                n0, e0, _ = vehicle.position()
                if math.hypot(pn - (n0 + dn), pe - (e0 + de)) < PERSON_CLEAR_M:
                    vehicle.hover()
                    return "bırakma noktasının yakınında kişi var"
        err = math.hypot(dn, de)
        left = h - target_alt
        centered = err < max(ALIGN_TOL_M, 0.04 * h)   # yüksekte biraz gevşek, aşağıda sıkı
        good = good + 1 if (left < 0.2 and err < ALIGN_TOL_M) else 0
        if good >= 3:
            vehicle.hover()
            return None
        sp = min(1.5, 0.5 * err)
        vn, ve = (dn / err * sp, de / err * sp) if err > 1e-6 else (0.0, 0.0)
        vn_f, ve_f = 0.5 * vn_f + 0.5 * vn, 0.5 * ve_f + 0.5 * ve
        vd = min(DESCENT_SPEED, 0.6 * left) if (centered and left > 0) else 0.0
        if left < -0.5:
            vd = -0.5   # fazla alçaldı: hafifçe yüksel
        vehicle.velocity_ned(vn_f, ve_f, vd)
        time.sleep(0.12)
    vehicle.hover()
    return "alçalma zaman aşımı"


def nearest_dropzone(vehicle, img):
    """Görüntü merkezindeki (hizalanılan) arabaya en yakın teslimat işaretçisi: (id, mesafe_m) ya da None."""
    roll, pitch, yaw = vehicle.attitude()
    h = vehicle.height_agl()
    best = None
    for mid, center, _ in find_markers(img):
        g = pixel_to_ground(img.shape, *center, h, roll, pitch, yaw)
        if g is None:
            continue
        d = math.hypot(*g)
        if d <= DROPZONE_MAX_M and (best is None or d < best[1]):
            best = (mid, d)
    return best


def search_and_deliver(vehicle, grab, detector, payload, area, alt=25.0, deliveries=3, speed=5.0,
                       landing=(0.0, 0.0), on_drop=None, log=print, mode="dropzone", drop_alt=None, approve=None):
    """Tam görev. (teslimatlar, işaretçiyle_indi_mi, süre_s, iptal_sebebi) döndürür.

    drop_alt: dropzone modunda bu irtifaya (zeminden, m) alçalıp bırakır; None = seyir irtifasından.
    approve:  isteğe bağlı approve(img, açıklama) -> bool; bırakmadan hemen önce çağrılır
              (gerçek uçuşta pilot onayı). False dönerse paket bırakılmaz.
    """
    route = lawnmower(area, LANE_SPACING_M, alt)
    log(f"Arama deseni: {len(route) // 2} şerit, irtifa {alt:.0f} m")
    visited, dropped, results = [], [], []
    served_ids = set()  # teslimat yapılan işaretçi numaraları
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
            if mode == "roof" and any(math.hypot(n - dn, e - de) < NEW_TARGET_M for dn, de in dropped):
                log("  bu araba zaten teslim aldı, bırakılmadı; aramaya devam")
                continue
            marker_id = None
            if mode == "dropzone":
                dz = None
                for _ in range(15):          # birkaç kare dene (işaretçi bulanıklık/açı ile kaçabilir)
                    dz = nearest_dropzone(vehicle, grab())
                    if dz:
                        break
                    time.sleep(0.1)
                if dz is None:
                    log("  bu arabanın yanında teslimat işaretçisi yok, BIRAKILMADI; aramaya devam")
                    continue
                marker_id = dz[0]
                if marker_id in served_ids:
                    log(f"  #{marker_id} zaten teslim aldı, bırakılmadı; aramaya devam")
                    continue
                log(f"  teslimat noktası #{marker_id} ({dz[1]:.1f} m yanda) -> hizalanılıyor")
                if not align_over_marker(vehicle, grab, marker_id):
                    log("  işaretçi kayboldu, bırakılmadı; aramaya devam")
                    continue
                if drop_alt is not None and drop_alt < alt - 0.5:
                    log(f"  {drop_alt:.0f} m'ye alçalınıyor")
                    why = descend_over_marker(vehicle, grab, marker_id, drop_alt, detector)
                    if why is not None:
                        log(f"  {why}, BIRAKILMADI; seyir irtifasına dönülüyor")
                        n, e, _ = vehicle.position()
                        vehicle.goto(n, e, -alt, 3.0)
                        continue
                fine = fine_align_marker(vehicle, grab, marker_id)
                if fine is None:
                    log("  ince hizalamada işaretçi görülemedi, BIRAKILMADI; aramaya devam")
                    continue
                n, e, _ = vehicle.position()
                served_ids.add(marker_id)
            if approve is not None and not approve(grab(), f"teslimat #{marker_id}, {vehicle.height_agl():.1f} m"):
                log("  pilot onay vermedi, BIRAKILMADI")
                if vehicle.height_agl() < alt - 0.5:
                    vehicle.goto(n, e, -alt, 3.0)
                continue
            dropped.append((n, e))
            handle = payload.release()
            rec = {"no": len(results) + 1, "n": round(n, 2), "e": round(e, 2), "guven": round(conf, 2),
                   "isaretci": marker_id, "zaman_s": round(time.time() - t_start, 1)}
            if mode == "dropzone":   # bırakma anında kameranın ölçtüğü işaretçi sapması (ortalama)
                rec["kamera_isaretci_ne_m"] = [round(fine[0], 3), round(fine[1], 3)]
                rec["kamera_yayilim_m"] = round(fine[2], 3)
            rec["birakma_irtifa_m"] = round(vehicle.height_agl(), 2)
            if on_drop is not None:
                rec.update(on_drop(handle, marker_id) or {})
            results.append(rec)
            log(f"  paket {rec['no']} bırakıldı" + (f": {rec.get('sonuc_metni', '')}" if on_drop else ""))
            fly_mission.hide_preview()
            if vehicle.height_agl() < alt - 0.5:
                vehicle.goto(n, e, -alt, 3.0)   # seyir irtifasına dikey çık
    except Abort as ex:
        abort_reason = str(ex)
        fly_mission.hide_preview()
        log(f"\nGÜVENLİK: {abort_reason} -> görev iptal, eve dönülüyor")

    if abort_reason is None:
        log("\nGörev tamam, iniş noktasına dönülüyor")
    vehicle.goto(landing[0], landing[1], -alt, speed)
    marker = aruco_land_vehicle(vehicle, grab)
    return results, marker, time.time() - t_start, abort_reason
