"""Kamera sağlık testi: kalk, iki farklı noktadan kare al; siyah veya donmuşsa hata ver."""
import sys

import numpy as np

from sim_client import connect, get_rgb, safe_land, wait_until_still

client = connect()
client.takeoffAsync().join()
frames = []
for x, y in [(5, 10), (-5, 15)]:
    client.moveToPositionAsync(x, y, -20, 4).join()
    wait_until_still(client)
    frames.append(get_rgb(client, "bottom_cam"))
safe_land(client)
client.enableApiControl(False)

a, b = frames
diff = np.abs(a.astype(int) - b.astype(int)).mean()
print(f"parlaklık: {a.mean():.0f} / {b.mean():.0f}, iki kare farkı: {diff:.1f}")
if a.mean() < 2 or diff < 1:
    print("KAMERA SORUNLU (siyah veya donmuş)")
    sys.exit(1)
print("KAMERA SAĞLAM")
