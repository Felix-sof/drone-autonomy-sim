"""Teşhis: kameradaki kutu konumundan hesaplanan sapma, gerçek sapmayla tutuyor mu?"""
import json

from sim_client import ROOT, airsim, connect, get_boxes, get_rgb, load_mission, setup_detection, wait_until_still

cfg = load_mission()
scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
G = scene["ground_z_ned"]
cam = cfg["camera"]
c = connect()
setup_detection(c, cam, cfg["target_mesh_patterns"], 80)
c.takeoffAsync().join()
car = scene["cars"][0]
cp = c.simGetObjectPose(car).position
print("araba", car, round(cp.x_val, 2), round(cp.y_val, 2))

for ox, oy in [(0, 0), (2, 0), (0, 2), (-2, -2)]:
    c.moveToPositionAsync(cp.x_val + ox, cp.y_val + oy, G - 26, 3).join()  # ağaçların üstünden
    c.moveToZAsync(G - 10, 2).join()
    wait_until_still(c, 0.05)
    d = c.getMultirotorState().kinematics_estimated.position
    cam_info = c.simGetCameraInfo(cam)
    img = get_rgb(c, cam)
    h, w = img.shape[:2]
    boxes = [b for b in get_boxes(c, cam) if f"{car}_" in b[4]]
    if not boxes:
        print(f"kayma ({ox:+},{oy:+}): araba görülmedi")
        continue
    b = boxes[0]
    ex = ((b[0] + b[2]) / 2 - w / 2) / (w / 2)
    ey = ((b[1] + b[3]) / 2 - h / 2) / (h / 2)
    height = G - d.z_val
    dx, dy = -ey * height * h / w, ex * height
    true_dx, true_dy = cp.x_val - d.x_val, cp.y_val - d.y_val
    print(f"kayma ({ox:+},{oy:+}) | drone ({d.x_val:.2f},{d.y_val:.2f}) | kutu {[round(v) for v in b[:4]]}"
          f" | hesap dx={dx:+.2f} dy={dy:+.2f} | gerçek dx={true_dx:+.2f} dy={true_dy:+.2f}"
          f" | kamera z={cam_info.pose.position.z_val:.2f}")

c.moveToZAsync(G - 26, 3).join()  # önce ağaçların üstüne çık
c.moveToPositionAsync(0, 0, G - 26, 4).join()
c.landAsync().join()
c.enableApiControl(False)
