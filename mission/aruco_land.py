"""Görüntüyle hassas iniş: helipad'deki ArUco işaretçisini kamerayla bulup üstüne iner.

Simülatörün hiçbir hazır bilgisini kullanmaz; sadece araç arayüzü (vehicle.py) ve kamera
karesi ister, bu yüzden aynı kod PX4/gerçek drone'da da çalışır:
  - işaretçinin görüntüdeki kenarı (piksel) + gerçek boyutu (4 m) -> kameraya uzaklık
  - piksel konumu -> kamera ışını; drone'un o anki roll/pitch'iyle yatay düzleme döndürülür
    (eğim telafisi: 2 m/s'de kayan drone ~10° eğilir, 25 m'de bu 4-5 m sahte sapma demek;
     telafisiz ilk denemede drone işaretçinin etrafında salınıp uzaklaştı)
  - yatay sapma kuzey/doğu'ya çevrilir, hız komutu dünya ekseninde verilir (tırmanma olmaz)
Çok alçakta işaretçi kadraja sığmaz; FINAL_HEIGHT_M'den sonra dikey son iniş.

Kamera: gövdeye sabit, dik aşağı. Görüntünün üstü gövde x (ileri), sağı gövde y (sağ).
"""
import math
import time

import cv2
import numpy as np

import fly_mission

MARKER_DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
MARKER_SIZE_M = 4.0     # siyah karenin kenarı (beyaz kenar boşluğu hariç)
HFOV_DEG = 90.0         # settings.json'daki alt kamera
FINAL_HEIGHT_M = 3.0    # bu yükseklikten sonra işaretçi kadraja sığmaz: dikey son iniş
GAIN = 0.5
MAX_SPEED = 1.5

_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(MARKER_DICT),
                                    cv2.aruco.DetectorParameters())


def find_marker(img):
    """(merkez_px, kenar_px, köşeler) ya da None."""
    corners, ids, _ = _detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return None
    for c, i in zip(corners, ids.ravel()):
        if i == MARKER_ID:
            pts = c[0]
            side = np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)])
            return pts.mean(axis=0), side, pts
    return None


def find_markers(img, exclude=(MARKER_ID,)):
    """Görüntüdeki bütün işaretçiler: [(id, merkez_px, kenar_px), ...] (varsayılan: helipad hariç)."""
    corners, ids, _ = _detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        if int(i) in exclude:
            continue
        pts = c[0]
        side = np.mean([np.linalg.norm(pts[k] - pts[(k + 1) % 4]) for k in range(4)])
        out.append((int(i), pts.mean(axis=0), side))
    return out


def marker_offset(img, marker, roll=0.0, pitch=0.0):
    """İşaretçinin drone'a göre yatay konumu (ileri_m, sağ_m; yaw yönünde) ve yükseklik (m).

    roll/pitch verilirse kamera ışını yatay düzleme döndürülür (eğim telafisi).
    """
    import perception
    roll, pitch, _ = perception.camera_angles(roll, pitch, 0.0)
    h, w = img.shape[:2]
    (cx, cy), side, _ = marker
    f = (w / 2) / math.tan(math.radians(HFOV_DEG / 2))
    distance = MARKER_SIZE_M * f / side
    ray_body = np.array([-(cy - h / 2) / f, (cx - w / 2) / f, 1.0])   # gövde: x ileri, y sağ, z aşağı
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    ray = ry @ rx @ ray_body                                          # yaw'sız yatay çerçeve
    height = distance * ray[2] / np.linalg.norm(ray)
    return height * ray[0] / ray[2], height * ray[1] / ray[2], height


def draw(img, marker, text):
    vis = img.copy()
    if marker is not None:
        cv2.polylines(vis, [marker[2].astype(int)], True, (0, 255, 0), 3)
    h, w = vis.shape[:2]
    cv2.drawMarker(vis, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
    cv2.putText(vis, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return vis


def aruco_land_vehicle(vehicle, grab, timeout=150, verbose=True):
    """İşaretçiye görüntüyle iner. grab(): BGR kamera karesi döndüren fonksiyon.

    True: işaretçiyle hizalanıp indi; False: işaretçi bulunamadı, bulunduğu yere indi.
    """
    t0 = time.time()
    lost, last_h, seen = 0, None, False
    vn_f = ve_f = 0.0  # yumuşatılmış komut
    while time.time() - t0 < timeout:
        img = grab()
        roll, pitch, yaw = vehicle.attitude()   # kareyle aynı ana yakın
        m = find_marker(img)
        if m is None:
            lost += 1
            vehicle.hover()
            if seen and last_h is not None and last_h < FINAL_HEIGHT_M + 2:
                break                      # alçakta kadrajdan taştı: son iniş
            if lost > 60:
                break                      # işaretçi yok: bulunduğu yere in (GPS dönüş noktası)
            time.sleep(0.1)
            continue
        lost, seen = 0, True
        fwd, right, height = marker_offset(img, m, roll, pitch)
        import perception
        yaw = perception.camera_angles(roll, pitch, yaw)[2]   # görüntü ekseninin dünyadaki yönü
        last_h = height
        err = math.hypot(fwd, right)
        fly_mission.show(draw(img, m, f"yukseklik {height:.1f} m  sapma {err:.2f} m"), [])
        if height < FINAL_HEIGHT_M:
            break
        # yaw yönündeki sapmayı kuzey/doğuya çevir
        n = fwd * math.cos(yaw) - right * math.sin(yaw)
        e = fwd * math.sin(yaw) + right * math.cos(yaw)
        sp = min(MAX_SPEED, GAIN * err)
        vn, ve = (n / err * sp, e / err * sp) if err > 1e-6 else (0.0, 0.0)
        vn_f, ve_f = 0.5 * vn_f + 0.5 * vn, 0.5 * ve_f + 0.5 * ve   # ani yön değişimini yumuşat
        vd = min(1.5, 0.2 * height) if err < max(0.3, 0.04 * height) else 0.0   # ortalandıkça alçal
        vehicle.velocity_ned(vn_f, ve_f, vd)
        time.sleep(0.15)
    fly_mission.hide_preview()
    if verbose:
        print("  ArUco: işaretçiyle hizalandı, son iniş" if seen else "  ArUco: işaretçi bulunamadı, yerinde iniliyor")
    vehicle.wait_still(max_speed=0.15, timeout=5)
    vehicle.land()
    return seen


def aruco_land(client, cam, timeout=150, verbose=True):
    """AirSim istemcisiyle kısa yol (mevcut görev scriptleri için)."""
    from sim_client import get_rgb
    from vehicle import AirSimVehicle
    return aruco_land_vehicle(AirSimVehicle(client), lambda: get_rgb(client, cam), timeout, verbose)
