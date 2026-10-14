"""AirSim / Cosys-AirSim istemcisi ve ortak yardımcılar."""
import json
import time
from pathlib import Path

import numpy as np

try:
    import cosysairsim as airsim  # Cosys-AirSim (UE5)
except ImportError:
    import airsim  # Orijinal AirSim / Colosseum

ROOT = Path(__file__).resolve().parent.parent


def load_mission(path=None):
    path = Path(path) if path else ROOT / "mission" / "mission.json"
    return json.loads(path.read_text(encoding="utf-8"))


def connect():
    """Bağlanır. Grafik ayarları burada DEĞİŞTİRİLMEZ.

    Unreal 5.8'de çalışırken scalability/render cvar'ı değiştirmek IsInGameThread
    "ensure" uyarısı veriyor ve AirSim kamerasını o editör oturumu boyunca karartıyor.
    Ayarlar projenin Config/DefaultEngine.ini dosyasında:
      - Lumen kapalı (FPS için)
      - Virtual Shadow Maps AÇIK: kapalıyken AirSim kamerası simsiyah çekiyor.
    Not: drone yerdeyken alt kamera zemin hizasında kalır, siyah görünmesi normaldir.
    Kameranın uçuşta sağlam olduğunu camera_check.py test eder.
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    return client


def wait_until_still(client, max_speed=0.2, timeout=10):
    """Drone'un yatay hareketi bitene kadar havada bekler."""
    client.hoverAsync().join()
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = client.getMultirotorState().kinematics_estimated.linear_velocity
        if (v.x_val ** 2 + v.y_val ** 2) ** 0.5 < max_speed:
            return
        time.sleep(0.2)


def safe_land(client):
    """Yumuşak iniş: dur, 2 m'ye yavaşça in, yere değmesini bekle, sonra motorları kapat.

    Durmadan inişe geçmek yana kayarak yere değmeye ve devrilmeye yol açar.
    """
    wait_until_still(client)
    client.moveToZAsync(-1.5, 1).join()
    wait_until_still(client)
    client.landAsync().join()
    t0 = time.time()
    while client.getMultirotorState().landed_state != airsim.LandedState.Landed and time.time() - t0 < 10:
        time.sleep(0.2)
    time.sleep(2)
    client.armDisarm(False)


def get_rgb(client, camera):
    """Kameradan tek bir kare alır, OpenCV/YOLO'nun beklediği (H, W, 3) BGR dizisi döndürür.

    Cosys-AirSim sıkıştırılmamış görüntüyü RGB sırasıyla veriyor (orijinal AirSim BGR'di).
    Çevrilmezse kırmızı-mavi yer değiştirir: orman toprağı mavi, kırmızı araba mavi görünür.
    """
    req = airsim.ImageRequest(camera, airsim.ImageType.Scene, False, False)
    resp = client.simGetImages([req])[0]
    img = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
    return img.reshape(resp.height, resp.width, -1)[:, :, 2::-1].copy()  # RGB(A) -> BGR


def setup_detection(client, camera, patterns, radius_m):
    """Unreal'in kendi bildiği nesnelerden otomatik bounding box üretmeyi açar."""
    image_type = airsim.ImageType.Scene
    client.simClearDetectionMeshNames(camera, image_type)
    client.simSetDetectionFilterRadius(camera, image_type, radius_m * 100)
    for p in patterns:
        client.simAddDetectionFilterMeshName(camera, image_type, p)


def get_boxes(client, camera):
    """Gerçek (ground-truth) kutular: [(x1, y1, x2, y2, isim), ...] piksel cinsinden."""
    dets = client.simGetDetections(camera, airsim.ImageType.Scene) or []
    return [(d.box2D.min.x_val, d.box2D.min.y_val,
             d.box2D.max.x_val, d.box2D.max.y_val, d.name) for d in dets]


def randomize_weather(client, rng):
    """Domain randomization: saat, sis, yağmur. Modelin gerçek dünyada da çalışması için önemli."""
    hour = rng.integers(7, 19)
    client.simSetTimeOfDay(True, start_datetime=f"2026-06-15 {hour:02d}:00:00",
                           celestial_clock_speed=1, update_interval_secs=60, move_sun=True)
    client.simEnableWeather(True)
    client.simSetWeatherParameter(airsim.WeatherParameter.Fog, float(rng.uniform(0, 0.25)))
    client.simSetWeatherParameter(airsim.WeatherParameter.Rain, float(rng.choice([0, 0, rng.uniform(0, 0.5)])))
