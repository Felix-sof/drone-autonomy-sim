"""Kuşbakışı fotoğraf: drone yükselip alt kameradan birkaç kare alır, results/ altına kaydeder.

Kullanım: python mission/snapshot.py [irtifa_m]
"""
import json
import sys
from datetime import datetime

import cv2
import numpy as np

from sim_client import ROOT, connect, get_boxes, get_rgb, load_mission, safe_land, setup_detection, wait_until_still

alt = float(sys.argv[1]) if len(sys.argv) > 1 else 45
cfg = load_mission()
scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
G = scene["ground_z_ned"]
c = connect()
setup_detection(c, cfg["camera"], cfg["target_mesh_patterns"], 120)
c.takeoffAsync().join()
c.moveToZAsync(G - alt, 4).join()

shots = []
# başlangıç noktası ve araba olan iki açıklığın üstü
for x, y in [(0, 0)] + [tuple(cl[:2]) for cl in scene["clearings"][1:3]]:
    c.moveToPositionAsync(x, y, G - alt, 6).join()
    wait_until_still(c, 0.1)
    img = get_rgb(c, cfg["camera"])
    for x1, y1, x2, y2, _ in get_boxes(c, cfg["camera"]):
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
    shots.append(cv2.resize(img, (640, 360)))

c.moveToPositionAsync(0, 0, G - alt, 6).join()
safe_land(c)
c.enableApiControl(False)

out = ROOT / "results" / f"snapshot_{datetime.now():%Y%m%d_%H%M%S}.jpg"
(out).write_bytes(cv2.imencode(".jpg", np.vstack(shots))[1].tobytes())
print(f"Kaydedildi: {out}")
