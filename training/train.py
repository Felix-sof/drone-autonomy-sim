"""YOLO modelini simülasyon verisiyle eğitir. RTX 4060 Laptop (8 GB) için ayarlı.

Önce: python training/prepare_dataset.py
Unreal açıkken eğitme; ikisi aynı ekran kartı belleğini paylaşır.
"""
from ultralytics import YOLO

DATA = r"C:\UE\drone_data\yolo\data.yaml"
RUNS = r"C:\UE\drone_runs"

if __name__ == "__main__":  # Windows'ta veri yükleyici işçileri için gerekli
    model = YOLO("yolo11n.pt")  # küçük ve hızlı; daha iyi sonuç için yolo11s.pt
    model.train(
        data=DATA,
        epochs=60,
        imgsz=960,    # yukarıdan bakınca arabalar küçük kalıyor, çözünürlüğü yüksek tut
        batch=8,
        workers=4,
        project=RUNS,
        name="car",
        exist_ok=True,
    )
    print(rf"En iyi model: {RUNS}\car\weights\best.pt")
