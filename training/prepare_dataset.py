"""dataset/raw altındaki tüm uçuşları birleştirip train/val olarak ayırır (YOLO formatı)."""
import random
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RAW = ROOT / "dataset" / "raw"
OUT = Path(r"C:\UE\drone_data\yolo")  # OneDrive ve Türkçe karakterli yolun dışında
VAL_RATIO = 0.15

images = sorted(RAW.glob("*/images/*.jpg"))
if not images:
    raise SystemExit("dataset/raw içinde görüntü yok, önce bir uçuş yap.")

# Bölme KARE değil TUR (oturum klasörü) bazında: art arda kareler neredeyse aynı olduğu için
# kare bazında bölünce doğrulama eğitimin kopyası oluyor ve başarı olduğundan yüksek çıkıyor
# (orman modelinde val mAP50 0.81 gösterip canlı sahnede arabaları kaçırdı).
sessions = sorted({img.parent.parent.name for img in images})
random.seed(0)
random.shuffle(sessions)
val_sessions = set(sessions[:max(1, round(len(sessions) * VAL_RATIO))])

shutil.rmtree(OUT, ignore_errors=True)
n_val = 0
for img in images:
    split = "val" if img.parent.parent.name in val_sessions else "train"
    n_val += split == "val"
    label = img.parent.parent / "labels" / f"{img.stem}.txt"
    name = f"{img.parent.parent.name}_{img.stem}"
    for sub, src, ext in (("images", img, ".jpg"), ("labels", label, ".txt")):
        dst = OUT / sub / split
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copy(src, dst / f"{name}{ext}")

# sınıf isimleri: python training/prepare_dataset.py car,person   (varsayılan: car)
NAMES = (sys.argv[1] if len(sys.argv) > 1 else "car").split(",")
names = "".join(f"  {i}: {n}\n" for i, n in enumerate(NAMES))
(OUT / "data.yaml").write_text(
    f"path: {OUT.as_posix()}\ntrain: images/train\nval: images/val\nnames:\n{names}", encoding="utf-8")
print(f"{len(images) - n_val} train, {n_val} val -> {OUT}")
