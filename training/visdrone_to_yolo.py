"""VisDrone2019-DET (gerçek drone görüntüleri) -> tek sınıflı YOLO "car" veri seti.

VisDrone sınıfları: 1 yaya, 2 insan, 3 bisiklet, 4 car, 5 van, 6 truck, 7 tricycle,
8 awning-tricycle, 9 bus, 10 motor, 11 diğer; skor 0 = yok sayılan bölge.
Bizim "car" sınıfımız = car + van. (Kamyon/otobüs dahil değil.)

Lisans: VisDrone akademik/ticari olmayan kullanım içindir.

  python training/visdrone_to_yolo.py C:/UE/visdrone/VisDrone2019-DET-val val
"""
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

src = Path(sys.argv[1])
split = sys.argv[2] if len(sys.argv) > 2 else "val"
OUT = Path(r"C:\UE\visdrone\yolo_car")
CAR_CLASSES = {4, 5}

(OUT / "images" / split).mkdir(parents=True, exist_ok=True)
(OUT / "labels" / split).mkdir(parents=True, exist_ok=True)
n_img = n_box = 0
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
        if score == 0 or cat not in CAR_CLASSES or bw < 4 or bh < 4:
            continue
        lines.append(f"0 {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} {bw / w:.6f} {bh / h:.6f}")
    shutil.copy(img_path, OUT / "images" / split / img_path.name)
    (OUT / "labels" / split / f"{ann.stem}.txt").write_text("\n".join(lines))
    n_img += 1
    n_box += len(lines)

(OUT / "data.yaml").write_text(
    f"path: {OUT.as_posix()}\ntrain: images/train\nval: images/val\nnames:\n  0: car\n", encoding="utf-8")
print(f"{split}: {n_img} görüntü, {n_box} araç kutusu -> {OUT}")
