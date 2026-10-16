"""PX4 yerel çerçevesi ile AirSim dünya çerçevesi uyuşuyor mu? (sim.sh px4 açıkken)

PX4'e bir kare çizdirir; her köşede PX4'ün kendi konum/yön tahminini AirSim'in gerçek
değerleriyle karşılaştırır. Çıktı: konum farkı, yön (yaw) farkı ve iki çerçeve arasındaki
dönme açısı. PX4 görevindeki "yaklaşınca kayboldu" sorununun teşhisi için.
"""
import math
import warnings

import numpy as np

warnings.filterwarnings("ignore")
from sim_client import airsim  # noqa: E402
from vehicle import Px4Vehicle, quat_to_euler  # noqa: E402

c = airsim.MultirotorClient()
c.confirmConnection()
v = Px4Vehicle()


def airsim_state():
    p = c.simGetVehiclePose()
    _, _, yaw = quat_to_euler(p.orientation.w_val, p.orientation.x_val, p.orientation.y_val, p.orientation.z_val)
    return np.array([p.position.x_val, p.position.y_val, p.position.z_val]), yaw


p0_sim, _ = airsim_state()
p0_px4 = np.array(v.position())
v.arm_takeoff(20)
px4_pts, sim_pts = [], []
for n, e in [(0, 0), (25, 0), (25, 25), (0, 25)]:
    v.goto(n, e, -20, 4, tol=0.3)
    v.wait_still(0.1)
    ps, ys = airsim_state()
    pp = np.array(v.position())
    yp = v.attitude()[2]
    px4_pts.append(pp - p0_px4)
    sim_pts.append(ps - p0_sim)
    print(f"hedef ({n:>2}, {e:>2}) | PX4 {np.round(pp - p0_px4, 2)} yaw {math.degrees(yp):6.1f}° | "
          f"AirSim {np.round(ps - p0_sim, 2)} yaw {math.degrees(ys):6.1f}°")

# iki çerçeve arasındaki dönme (yatay düzlemde en küçük kareler)
A, B = np.array(px4_pts)[:, :2], np.array(sim_pts)[:, :2]
ang = math.atan2(np.sum(A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]), np.sum(A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]))
scale = np.linalg.norm(B) / max(np.linalg.norm(A), 1e-9)
print(f"\nPX4 -> AirSim dönme: {math.degrees(ang):.1f}°, ölçek: {scale:.3f}")
v.goto(0, 0, -20, 4)
v.land()
