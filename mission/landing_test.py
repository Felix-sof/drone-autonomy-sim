"""ArUco hassas iniş testi: kaydırılmış noktalardan sadece kamerayla iner, hatayı ölçer.

Unreal'in bilgisi sadece değerlendirmede kullanılır (helipad'in gerçek konumu).
Kullanım: python mission/landing_test.py [deneme] [irtifa_m]
"""
import json
import math
import sys

import numpy as np

import fly_mission
from aruco_land import aruco_land
from sim_client import ROOT, connect, load_mission

n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
alt = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
fly_mission.SHOW_PREVIEW = "--goster" in sys.argv
cfg = load_mission()
import perception  # noqa: E402
perception.CAMERA_STABILIZED = cfg.get("camera_gimbal", False)
perception.CAMERA_YAW_LOCK = (None if cfg.get("camera_yaw_lock_deg") is None
                              else __import__("math").radians(cfg["camera_yaw_lock_deg"]))
scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
G = scene["ground_z_ned"]
rng = np.random.default_rng()
client = connect()
hx, hy = cfg["landing"]  # setup_forest helipad'i tam bu NED noktasına koyar (sadece değerlendirme)
errs = []
for i in range(1, n + 1):
    client.reset()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(G - alt, 3).join()
    a, r = rng.uniform(0, 2 * math.pi), rng.uniform(2, 6)   # helipad'den 2-6 m kaydır (açıklık içinde)
    client.moveToPositionAsync(hx + r * math.cos(a), hy + r * math.sin(a), G - alt, 3).join()
    ok = aruco_land(client, cfg["camera"], verbose=False)
    d = client.getMultirotorState().kinematics_estimated.position
    e = math.hypot(d.x_val - hx, d.y_val - hy)
    errs.append(e)
    print(f"Deneme {i}: başlangıç sapması {r:.1f} m -> iniş hatası {e:.2f} m ({'işaretçiyle' if ok else 'İŞARETÇİSİZ'})")
client.enableApiControl(False)
print(f"\nArUco iniş ({alt:.0f} m'den): ortalama hata {np.mean(errs):.2f} m, en kötü {max(errs):.2f} m")
