"""README için kısa demo GIF'i: arama -> hizalanma -> 25 m'den bırakma -> işaretçiyle iniş.

  bash mission/sim.sh airsim
  python mission/record_gif.py --model C:/UE/drone_runs/carperson3/weights/best.pt

Tek bir teslimat görevi uçulur (görev çekirdeği autonomy.py, AirSim uçuş kontrolcüsü). Her önizleme
karesi yakalanır; istenirse yanına Unreal penceresindeki dış görünüm eklenir (--dis-gorunum: Unreal
penceresi ekranda açık ve önde olmalı). Sonunda her sahneden eşit sürede kare seçilip
docs/images/demo.gif (ve .mp4) yazılır.
"""
import argparse
import ctypes
import ctypes.wintypes as wt
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageGrab

import drop_test
import fly_mission
import perception
from autonomy import search_and_deliver
from collect_data import area, scatter_cars, scatter_people
from payload import SimPayload
from perception import CarDetector
from sim_client import ROOT, airsim, get_rgb, load_mission
from vehicle import AirSimVehicle

SCENES = [("arama", "1 · Arama: model arabaları buluyor"),
          ("hizalanma", "2 · Teslimat noktasına hizalanma"),
          ("birakma", "3 · 25 m'den bırakma"),
          ("inis", "4 · İşaretçiyle iniş")]
TILE = (480, 270)
FONT = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 22)
SMALL = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 16)


class Recorder:
    def __init__(self, external):
        self.frames = []          # (zaman, sahne, drone_karesi_bgr, dış_görünüm_rgb ya da None)
        self.scene = "arama"
        self.last = 0.0
        self.external = external
        self.rect = None
        if external:
            ctypes.windll.user32.SetProcessDPIAware()
            self.rect = self._unreal_rect()

    @staticmethod
    def _unreal_rect():
        user32 = ctypes.windll.user32
        found = []

        def cb(hwnd, _):
            n = user32.GetWindowTextLengthW(hwnd)
            buf = ctypes.create_unicode_buffer(n + 1)
            user32.GetWindowTextW(hwnd, buf, n + 1)
            if "Unreal Editor" in buf.value and user32.IsWindowVisible(hwnd):
                found.append(hwnd)
            return True
        user32.EnumWindows(ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)(cb), 0)
        if not found:
            print("uyarı: Unreal penceresi bulunamadı, sadece drone kamerası kaydedilecek")
            return None
        r = wt.RECT()
        user32.GetWindowRect(found[0], ctypes.byref(r))
        user32.SetForegroundWindow(found[0])
        return r.left, r.top, r.right, r.bottom

    def show(self, img, dets):
        now = time.time()
        if now - self.last < 0.1:          # en fazla ~10 kare/s
            return
        self.last = now
        vis = img.copy()
        for d in dets:
            x1, y1, x2, y2 = (int(v) for v in d[:4])
            cls = int(d[5]) if len(d) > 5 else 0
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 220, 255) if cls == 1 else (0, 0, 255), 4)
        ext = None
        if self.rect is not None:
            try:
                ext = ImageGrab.grab(bbox=self.rect, all_screens=True)
            except OSError:
                ext = None
        self.frames.append((now, self.scene, cv2.resize(vis, TILE), ext))


def compose(scene_title, drone_bgr, ext):
    tiles = []
    if ext is not None:
        tiles.append(ext.convert("RGB").resize(TILE))
    tiles.append(Image.fromarray(cv2.cvtColor(drone_bgr, cv2.COLOR_BGR2RGB)))
    w = TILE[0] * len(tiles)
    canvas = Image.new("RGB", (w, TILE[1] + 40), (18, 18, 18))
    for i, t in enumerate(tiles):
        canvas.paste(t, (i * TILE[0], 40))
    d = ImageDraw.Draw(canvas)
    d.text((12, 8), scene_title, font=FONT, fill=(255, 255, 255))
    d.text((w - 260, 12), "Unreal Engine + PX4 · YOLO", font=SMALL, fill=(170, 170, 170))
    if ext is not None:
        d.text((8, 46), "dış görünüm", font=SMALL, fill=(255, 255, 255))
    d.text((len(tiles) * TILE[0] - TILE[0] + 8, 46), "drone kamerası", font=SMALL, fill=(255, 255, 255))
    return canvas


def build(rec, out_dir, per_scene_s=4.0, fps=10):
    out_dir.mkdir(parents=True, exist_ok=True)
    picked = []
    for key, title in SCENES:
        fr = [f for f in rec.frames if f[1] == key]
        if not fr:
            continue
        n = int(per_scene_s * fps)
        idx = np.linspace(0, len(fr) - 1, min(n, len(fr))).astype(int)
        # birakma: son kareleri (paketin yere varışı) mutlaka içersin
        picked += [compose(title, fr[i][2], fr[i][3]) for i in idx]
    if not picked:
        raise SystemExit("hiç kare yakalanmadı")
    gif = out_dir / "demo.gif"
    pal = [p.quantize(colors=128, method=Image.Quantize.MEDIANCUT) for p in picked]
    pal[0].save(gif, save_all=True, append_images=pal[1:], duration=int(1000 / fps), loop=0, optimize=True)
    mp4 = out_dir / "demo.mp4"
    w, h = picked[0].size
    vw = cv2.VideoWriter(str(mp4), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for p in picked:
        vw.write(cv2.cvtColor(np.array(p), cv2.COLOR_RGB2BGR))
    vw.release()
    print(f"{len(picked)} kare -> {gif} ({gif.stat().st_size / 1e6:.1f} MB), {mp4}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--dis-gorunum", action="store_true", help="Unreal penceresini de kaydet (önde olmalı)")
    ap.add_argument("--sahne-s", type=float, default=4.0, help="her sahnenin GIF'teki süresi")
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

    rec = Recorder(args.dis_gorunum)
    fly_mission.SHOW_PREVIEW = True
    fly_mission.show = rec.show
    drop_test.show = rec.show
    fly_mission.hide_preview = lambda: None
    drop_test.hide_preview = lambda: None

    def log(msg):
        print(msg)
        if "bulundu" in msg or "teslimat noktası" in msg:
            rec.scene = "hizalanma"
        elif "Görev tamam" in msg or "GÜVENLİK" in msg:
            rec.scene = "inis"

    payload = SimPayload(client)
    release = payload.release

    def release_and_mark():
        rec.scene = "birakma"
        return release()
    payload.release = release_and_mark

    def on_drop(pkg, marker_id=None):
        drop_test.wait_until_rest(client, pkg, cam)   # düşüş boyunca kare yakalanır
        return {}

    vehicle = AirSimVehicle(client, ground_z=G)
    try:
        search_and_deliver(vehicle, lambda: get_rgb(client, cam), CarDetector(args.model, imgsz=960), payload,
                           area(scene), alt=25.0, deliveries=1, speed=5.0, landing=tuple(cfg["landing"]),
                           on_drop=on_drop, log=log)
    finally:
        vehicle.release()
    build(rec, ROOT / "docs" / "images", per_scene_s=args.sahne_s)


if __name__ == "__main__":
    main()
