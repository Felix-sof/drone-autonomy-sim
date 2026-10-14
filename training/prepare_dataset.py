"""dataset/raw altındaki tüm uçuşları birleştirip train/val olarak ayırır (YOLO formatı)."""
import random
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RAW = ROOT / "dataset" / "raw"
OUT = Path(r"C:\UE\drone_data\yolo")  # OneDrive ve Türkçe karakterli yolun dışında
VAL_RATIO = 0.15

images = sorted(RAW.glob("*/images/*.jpg"))
if not images:
    raise SystemExit("dataset/raw içinde görüntü yok, önce bir uçuş yap.")
random.seed(0)
random.shuffle(images)

shutil.rmtree(OUT, ignore_errors=True)
n_val = int(len(images) * VAL_RATIO)
for i, img in enumerate(images):
    split = "val" if i < n_val else "train"
    label = img.parent.parent / "labels" / f"{img.stem}.txt"
    name = f"{img.parent.parent.name}_{img.stem}"
    for sub, src, ext in (("images", img, ".jpg"), ("labels", label, ".txt")):
        dst = OUT / sub / split
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copy(src, dst / f"{name}{ext}")

(OUT / "data.yaml").write_text(
    f"path: {OUT.as_posix()}\ntrain: images/train\nval: images/val\nnames:\n  0: car\n",
    encoding="utf-8")
print(f"{len(images) - n_val} train, {n_val} val -> {OUT}")
