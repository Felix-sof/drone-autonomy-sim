"""Bir YOLO modelini bir veri setinde ölçer (ör. gerçek drone görüntüleri: VisDrone).

  python training/evaluate.py C:/UE/drone_runs/car_forest2/weights/best.pt C:/UE/visdrone/yolo_car/data.yaml --device cpu
"""
import argparse
import sys
import threading


def run(a):
    from ultralytics import YOLO
    m = YOLO(a.model)
    r = m.val(data=a.data, imgsz=a.imgsz, batch=8, device=a.device, plots=False, verbose=False,
              project=r"C:\UE\drone_runs", name="eval", exist_ok=True)
    p, rc, m50, m5095 = r.box.mp, r.box.mr, r.box.map50, r.box.map
    print(f"\nSONUÇ: precision {p:.3f}  recall {rc:.3f}  mAP50 {m50:.3f}  mAP50-95 {m5095:.3f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("data")
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--device", default="0")
    a = ap.parse_args()
    threading.stack_size(64 * 1024 * 1024)  # matplotlib/pyparsing stack overflow (bkz. train.py)
    sys.setrecursionlimit(100_000)
    t = threading.Thread(target=run, args=(a,))
    t.start()
    t.join()
