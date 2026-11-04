"""Demo videosu: tek bir teslimat görevini iki kameradan kaydeder (yazısız).

  bash mission/sim.sh airsim
  python mission/record_video.py --model C:/UE/drone_runs/carperson3/weights/best.pt              # tavan
  python mission/record_video.py --model ... --mod dropzone                                       # teslimat noktası

Solda takip kamerası (settings.json "chase_cam": drone'un 3 m arkası, 1.3 m üstü), sağda drone'un
alt kamerası ve modelin kutuları: kilitlenilen araba kalın kırmızı (beyaz kenarlı), diğer arabalar
ince kırmızı, kişiler sarı. Kayıt görevden bağımsız bir iş parçacığında, kendi AirSim bağlantısıyla
sabit hızda yapılır (görev bir yere uçarken de kare kaçmaz). Sonunda ffmpeg ile 2 kat hızlı H.264.
Çıktı: docs/videos/arac_ustu.mp4 ya da teslimat_noktasi.mp4
"""
import argparse
import asyncio
import json
import math
import shutil
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np

import drop_test
import fly_mission
import perception
from autonomy import search_and_deliver
from collect_data import area, scatter_cars, scatter_people
from payload import SimPayload
from perception import CarDetector
from sim_client import ROOT, airsim, get_rgb, load_mission
from vehicle import AirSimVehicle

SIZE = (960, 540)
RED, WHITE, YELLOW = (0, 0, 255), (255, 255, 255), (0, 220, 255)


def draw_boxes(img, dets, src_shape, locked):
    """Kutuları çiz: kilitli hedef kalın kırmızı + beyaz kenar; diğerleri ince. Koordinatlar tam çözünürlükten."""
    sy, sx = img.shape[0] / src_shape[0], img.shape[1] / src_shape[1]
    for d in dets:
        x1, y1, x2, y2 = int(d[0] * sx), int(d[1] * sy), int(d[2] * sx), int(d[3] * sy)
        person = len(d) > 5 and int(d[5]) == 1
        t = 4 if locked else 2
        cv2.rectangle(img, (x1, y1), (x2, y2), WHITE, t + 3)            # her zeminde seçilsin
        cv2.rectangle(img, (x1, y1), (x2, y2), YELLOW if person else RED, t)


class VideoRecorder(threading.Thread):
    """Alt kamera + takip kamerasını tek istekte, ~fps kare/s çekip doğrudan dosyaya yazar."""

    def __init__(self, cam, chase, path, fps=8):
        super().__init__(daemon=True)
        self.cam, self.chase, self.path, self.fps = cam, chase, path, fps
        self.client = None                          # run() içinde açılır (msgpackrpc iş parçacığına bağlı)
        self.dets, self.src_shape, self.det_t, self.locked = [], (1080, 1920), 0.0, False
        self.stop_flag = False
        self.frames = 0

    def update(self, img, dets, locked):   # görevden: son tespitler
        self.dets, self.src_shape, self.det_t, self.locked = list(dets), img.shape[:2], time.time(), locked

    def _grab(self):
        reqs = [airsim.ImageRequest(c, airsim.ImageType.Scene, False, False) for c in (self.cam, self.chase) if c]
        out = []
        for r in self.client.simGetImages(reqs):
            a = np.frombuffer(r.image_data_uint8, dtype=np.uint8).reshape(r.height, r.width, -1)[:, :, 2::-1]
            out.append(cv2.resize(np.ascontiguousarray(a), SIZE))
        return out

    def run(self):
        asyncio.set_event_loop(asyncio.new_event_loop())   # AirSim istemcisi bu iş parçacığında olay döngüsü ister
        self.client = airsim.MultirotorClient()            # görevle aynı bağlantıyı paylaşmasın
        self.client.confirmConnection()
        w = SIZE[0] * (2 if self.chase else 1)
        vw = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, SIZE[1]))
        period = 1.0 / self.fps
        while not self.stop_flag:
            t0 = time.time()
            imgs = self._grab()
            drone = imgs[0]
            if time.time() - self.det_t < 0.5:          # sadece güncel tespitleri çiz
                draw_boxes(drone, self.dets, self.src_shape, self.locked)
            vw.write(np.hstack([imgs[1], drone]) if self.chase else drone)
            self.frames += 1
            time.sleep(max(0.0, period - (time.time() - t0)))
        vw.release()


def to_h264(src, dst, speed=2.0):
    ff = shutil.which("ffmpeg")
    if ff is None:
        shutil.move(src, dst)
        return
    subprocess.run([ff, "-y", "-loglevel", "error", "-i", str(src), "-filter:v", f"setpts={1 / speed}*PTS",
                    "-c:v", "libx264", "-crf", "31", "-preset", "slow", "-pix_fmt", "yuv420p", "-an", str(dst)],
                   check=True)
    Path(src).unlink()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--mod", choices=["roof", "dropzone"], default="roof",
                    help="roof: aracın tavanına (varsayılan); dropzone: aracın yanındaki teslimat noktasına")
    ap.add_argument("--hiz", type=float, default=2.0, help="video oynatma hızı (kat)")
    ap.add_argument("--cikti", default=None)
    args = ap.parse_args()

    cfg = load_mission()
    perception.CAMERA_STABILIZED = cfg.get("camera_gimbal", False)
    perception.CAMERA_YAW_LOCK = None if cfg.get("camera_yaw_lock_deg") is None else math.radians(cfg["camera_yaw_lock_deg"])
    scene = json.loads((ROOT / "mission" / "scene_objects.json").read_text(encoding="utf-8"))
    G = scene["ground_z_ned"]
    drop_test.GROUND_Z = G
    cam = cfg["camera"]
    client = airsim.MultirotorClient()
    client.confirmConnection()
    rng = np.random.default_rng()
    placed = scatter_cars(client, scene, cfg["landing"], rng)
    scatter_people(client, scene, rng, list(placed or []))

    out = Path(args.cikti) if args.cikti else ROOT / "docs" / "videos" / (
        "arac_ustu.mp4" if args.mod == "roof" else "teslimat_noktasi.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    # OpenCV ve ffmpeg Türkçe karakterli yola (Masaüstü) yazamıyor: önce ASCII bir klasörde üret, sonra taşı
    tmp = Path("C:/UE/videos")
    tmp.mkdir(parents=True, exist_ok=True)
    raw, tmp_out = tmp / "raw.mp4", tmp / out.name
    rec = VideoRecorder(cam, cfg.get("chase_camera"), raw)

    phase = {"locked": False}
    fly_mission.SHOW_PREVIEW = True
    fly_mission.show = lambda img, dets: rec.update(img, dets, phase["locked"] and len(dets) == 1)
    fly_mission.hide_preview = lambda: None
    drop_test.show = lambda img, dets: None
    drop_test.hide_preview = lambda: None

    def log(msg):
        print(msg)
        if "bulundu" in msg:
            phase["locked"] = True          # bu andan itibaren tek kutu = kilitlenilen hedef
        elif "bırakıldı" in msg or "Görev tamam" in msg:
            phase["locked"] = False

    def on_drop(pkg, marker_id=None):     # ölçüm: Unreal'in gerçek konumları (videoya yazılmaz)
        _, impact = drop_test.wait_until_rest(client, pkg, cam)
        if args.mod == "dropzone" and marker_id is not None:
            pad = client.simGetObjectPose(scene["dropzones"][marker_id - 1]).position
            print(f"ölçüm: paket işaretçi merkezine {100 * math.hypot(impact[0] - pad.x_val, impact[1] - pad.y_val):.0f} cm")
        else:
            car = min(scene["cars"], key=lambda c: math.hypot(client.simGetObjectPose(c).position.x_val - impact[0],
                                                              client.simGetObjectPose(c).position.y_val - impact[1]))
            p = client.simGetObjectPose(car).position
            kind = drop_test.classify_impact(client, scene["cars"], car, impact)
            print(f"ölçüm: ilk temas = {kind} ({'TAVANA İSABET' if kind == 'hedef' else 'ISKA'}), "
                  f"araç merkezine {100 * math.hypot(impact[0] - p.x_val, impact[1] - p.y_val):.0f} cm")
        return {}

    vehicle = AirSimVehicle(client, ground_z=G)
    det = CarDetector(args.model, imgsz=960)
    rec.start()
    try:
        search_and_deliver(vehicle, lambda: get_rgb(client, cam), det, SimPayload(client), area(scene),
                           alt=25.0, deliveries=1, speed=5.0, landing=tuple(cfg["landing"]),
                           on_drop=on_drop, log=log, mode=args.mod)
        # iniş biter bitmez dur: kontrol bırakılınca alt kamera gökyüzüne dönüyor
    finally:
        rec.stop_flag = True
        rec.join()
        vehicle.release()
    to_h264(raw, tmp_out, args.hiz)
    shutil.move(str(tmp_out), str(out))
    print(f"{rec.frames} kare -> {out} ({out.stat().st_size / 1e6:.1f} MB, ~{rec.frames / rec.fps / args.hiz:.0f} s)")


if __name__ == "__main__":
    main()
