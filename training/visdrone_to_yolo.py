"""VisDrone2019-DET (gerçek drone görüntüleri) -> YOLO veri seti.

VisDrone sınıfları: 1 yaya, 2 insan, 3 bisiklet, 4 car, 5 van, 6 truck, 7 tricycle,
8 awning-tricycle, 9 bus, 10 motor, 11 diğer; skor 0 = yok sayılan bölge.

  --siniflar car         -> 0: car (car + van)                         C:/UE/visdrone/yolo_car
  --siniflar carperson   -> 0: car (car + van), 1: person (yaya + insan) C:/UE/visdrone/yolo_carperson

Lisans: VisDrone akademik/ticari olmayan kullanım içindir.

  python training/visdrone_to_yolo.py C:/UE/visdrone/VisDrone2019-DET-val val --siniflar carperson
"""
import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("src")
ap.add_argument("split", nargs="?", default="val")
ap.add_argument("--siniflar", choices=["car", "carperson"], default="car")
a = ap.parse_args()

src = Path(a.src)
MAP = {4: 0, 5: 0} if a.siniflar == "car" else {4: 0, 5: 0, 1: 1, 2: 1}
NAMES = ["car"] if a.siniflar == "car" else ["car", "person"]
OUT = Path(rf"C:\UE\visdrone\yolo_{a.siniflar}")
MIN_PX = {0: 4, 1: 2}   # kişiler küçük: daha küçük kutulara izin ver

(OUT / "images" / a.split).mkdir(parents=True, exist_ok=True)
(OUT / "labels" / a.split).mkdir(parents=True, exist_ok=True)
n_img, counts = 0, {i: 0 for i in range(len(NAMES))}
for ann in sorted((src / "annotations").glob("*.txt")):
    img_path = src / "images" / f"{ann.stem}.jpg"
    img = cv2.imdecode(np.fromfile(str(img_path), np.uint8), 0)
    if img is None:
        continue
    h, w = img.shape
    lines = []
    for row in ann.read_text().strip().splitlines():
        v = [int(x) for x in row.strip().strip(",").split(",")[:8]]
        x, y, bw, bh, score, cat = v[:6]
        if score == 0 or cat not in MAP:
            continue
        c = MAP[cat]
        if bw < MIN_PX[c] or bh < MIN_PX[c]:
            continue
        lines.append(f"{c} {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} {bw / w:.6f} {bh / h:.6f}")
        counts[c] += 1
    shutil.copy(img_path, OUT / "images" / a.split / img_path.name)
    (OUT / "labels" / a.split / f"{ann.stem}.txt").write_text("\n".join(lines))
    n_img += 1

names = "".join(f"  {i}: {n}\n" for i, n in enumerate(NAMES))
(OUT / "data.yaml").write_text(f"path: {OUT.as_posix()}\ntrain: images/train\nval: images/val\nnames:\n{names}",
                               encoding="utf-8")
print(f"{a.split}: {n_img} görüntü, " + ", ".join(f"{NAMES[i]} {counts[i]}" for i in counts) + f" -> {OUT}")
