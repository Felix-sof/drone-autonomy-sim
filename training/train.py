"""YOLO modelini simülasyon verisiyle eğitir. RTX 4060 Laptop (8 GB) için ayarlı.

Önce: python training/prepare_dataset.py
Unreal açıkken eğitme; ikisi aynı ekran kartı belleğini paylaşır.

  python training/train.py                                  # yolo11n, 60 epoch, ad "car"
  python training/train.py --model yolo11s.pt --epochs 80 --name car_forest
"""
import argparse
import sys
import threading

DATA = r"C:\UE\drone_data\yolo\data.yaml"
RUNS = r"C:\UE\drone_runs"


def run(args):
    from ultralytics import YOLO  # büyük yığınlı iş parçacığında içe aktarılsın (aşağıya bak)

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=960,    # yukarıdan bakınca arabalar küçük kalıyor, çözünürlüğü yüksek tut
        batch=args.batch,
        workers=4,
        project=RUNS,
        name=args.name,
        exist_ok=True,
        plots=not args.no_plots,
    )
    print(rf"En iyi model: {RUNS}\{args.name}\weights\best.pt")


if __name__ == "__main__":  # Windows'ta veri yükleyici işçileri için gerekli
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="yolo11n.pt", help="yolo11n.pt hızlı, yolo11s.pt daha isabetli")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--name", default="car")
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--data", default=DATA, help="ör. C:/UE/mixed/data.yaml (simülasyon + VisDrone)")
    ap.add_argument("--no-plots", action="store_true", help="Eğitim grafiklerini çizme")
    args = ap.parse_args()

    # Windows'ta iş parçacığı yığını 1 MB; matplotlib'in pyparsing grameri bu sınıra
    # dayanıp ara sıra "stack overflow" ile Python'u çökertiyor (2026-10-16 gecesi üç kez).
    # Eğitimi 64 MB yığınlı bir iş parçacığında çalıştırmak bunu çözer.
    threading.stack_size(64 * 1024 * 1024)
    sys.setrecursionlimit(100_000)
    t = threading.Thread(target=run, args=(args,))
    t.start()
    t.join()
