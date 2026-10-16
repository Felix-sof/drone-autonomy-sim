"""Algı: araba tespiti ve görüntü pikselinden yerdeki konuma geometri (simülatörden bağımsız).

Kamera gövdeye sabit, dik aşağı bakıyor (gerçek drone'da gimbal yoksa da aynı geometri).
Görüntünün üstü gövde x (ileri), sağı gövde y (sağ). Düz zemin varsayımı.
"""
import math

import numpy as np

# Kamera stabilizasyonlu gimbal'da mı? True: kamera eğilmez, roll/pitch telafisi yapılmaz.
# False: kamera gövdeye sabit, ışın drone'un roll/pitch'iyle yataya döndürülür.
# Telemetri (PX4/MAVLink) ile kamera karesi farklı anlarda geldiğinde gövdeye sabit kamerada
# telafi hatası 25 m'de ±5 m'ye çıktı (PX4 SITL); gerçek sistemde gimbal önerilir.
CAMERA_STABILIZED = False
# Gimbal yaw'ı dünyaya kilitliyse (radyan; 0 = görüntünün üstü kuzey). None: drone'un yaw'ını izler.
# Simülasyonda yaw'ı serbest bırakınca dike yakın açıda Unreal gimbal'ı yaw'ı kararlı tutamadı
# (görüntü kendi kendine döndü); kuzeye kilitlemek bunu çözdü. Gerçek gimbal'larda "yaw lock" modu.
CAMERA_YAW_LOCK = None


def camera_angles(roll, pitch, yaw):
    """Kameranın gerçek açıları: gimbal varsa roll/pitch sıfır, yaw kilitliyse kilit açısı."""
    if CAMERA_STABILIZED:
        roll = pitch = 0.0
        if CAMERA_YAW_LOCK is not None:
            yaw = CAMERA_YAW_LOCK
    return roll, pitch, yaw


def pixel_ray_level(shape, px, py, roll, pitch, hfov_deg=90.0):
    """Pikselden çıkan kamera ışını, yaw'sız yatay çerçevede (ileri, sağ, aşağı)."""
    h, w = shape[:2]
    f = (w / 2) / math.tan(math.radians(hfov_deg / 2))
    ray = np.array([-(py - h / 2) / f, (px - w / 2) / f, 1.0])
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx @ ray


def pixel_to_ground(shape, px, py, height, roll, pitch, yaw, hfov_deg=90.0):
    """Pikseldeki noktanın drone'a göre yerdeki konumu (Δkuzey, Δdoğu), metre.

    height: kameranın zeminden yüksekliği. Eğim (roll/pitch) telafi edilir: 25 m'de 10°'lik
    eğim telafi edilmezse ~4.5 m hata demek.
    """
    roll, pitch, yaw = camera_angles(roll, pitch, yaw)
    ray = pixel_ray_level(shape, px, py, roll, pitch, hfov_deg)
    if ray[2] <= 1e-3:
        return None  # ufka bakan ışın
    fwd, right = height * ray[0] / ray[2], height * ray[1] / ray[2]
    return (fwd * math.cos(yaw) - right * math.sin(yaw),
            fwd * math.sin(yaw) + right * math.cos(yaw))


class CarDetector:
    """YOLO tespiti. detect(img, conf, classes) -> [(x1, y1, x2, y2, güven, sınıf), ...]

    Sınıflar (iki sınıflı model): 0 araba, 1 kişi. classes=None: varsayılan sınıflar
    (default_classes; araba görevleri için [0]).
    """

    def __init__(self, model_path, conf=0.3, imgsz=960, default_classes=(0,)):
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.conf = conf
        self.imgsz = imgsz
        self.default_classes = list(default_classes)

    def detect(self, img, conf=None, classes=None):
        r = self.model.predict(img, conf=self.conf if conf is None else conf, imgsz=self.imgsz, verbose=False,
                               classes=self.default_classes if classes is None else list(classes))[0]
        return [(*b, c, int(k)) for b, c, k in zip(r.boxes.xyxy.cpu().tolist(), r.boxes.conf.cpu().tolist(),
                                                    r.boxes.cls.cpu().tolist())]


def box_center(b):
    return (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
