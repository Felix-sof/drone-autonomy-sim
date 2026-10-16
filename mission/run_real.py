"""Gerçek drone başlatıcısı (Pixhawk/PX4 + yardımcı bilgisayar + kamera + tutucu).

Test merdiveni, sırayla (bir öncekini geçmeden sonrakine geçme):
  1) bench    kamera + tespit, drone YOK:
       python mission/run_real.py bench --camera 0 --model best.onnx --goster
  2) connect  PX4'e bağlan, telemetri oku, MOTOR ÇALIŞMAZ:
       python mission/run_real.py connect --px4 serial:///dev/ttyACM0:57600
  3) mission  tam otonom görev (pervaneli uçuş!), açık onay ister:
       python mission/run_real.py mission --px4 ... --camera ... --model ... \
           --alan=-40,40,-30,30 --irtifa 25 --teslimat 1 --onay UCUS-ALANI-BOS

Görev alanı (--alan) kalkış noktasına göre metre: kuzey_min,kuzey_max,doğu_min,doğu_max.
PX4 tarafında ayrıca kendi geofence'ini (GF_*) ve RTL ayarlarını tanımla; bu kod onlara ek
bir güvenlik katmanıdır, yerine geçmez. Yasal izin, boş ve izinli test alanı, gözlemci ve
manuel kumanda ile anında devralma imkânı olmadan uçurma.
"""
import argparse
import math
import time

import fly_mission
from camera import OpenCVCamera
from perception import CarDetector

CONFIRM = "UCUS-ALANI-BOS"


def bench(a):
    cam = OpenCVCamera(a.camera)
    det = CarDetector(a.model)
    fly_mission.SHOW_PREVIEW = a.goster
    t0, n = time.time(), 0
    while time.time() - t0 < a.sure:
        img = cam.grab()
        t = time.time()
        boxes = det.detect(img, conf=0.5)
        n += 1
        fly_mission.show(img, boxes)
        if n % 10 == 0:
            print(f"{n} kare | tespit {len(boxes)} | çıkarım {1000 * (time.time() - t):.0f} ms"
                  f" | kare yaşı {1000 * cam.age():.0f} ms")
    print(f"Ortalama {n / (time.time() - t0):.1f} kare/s")
    cam.close()


def connect(a):
    from vehicle import Px4Vehicle
    print(f"PX4'e bağlanılıyor: {a.px4}")
    v = Px4Vehicle(a.px4)
    for _ in range(10):
        r, p, y = (math.degrees(x) for x in v.attitude())
        b = v.battery()
        print(f"konum NED {tuple(round(x, 2) for x in v.position())} | yükseklik {v.height_agl():.2f} m | "
              f"roll {r:.1f} pitch {p:.1f} yaw {y:.1f} | pil {'?' if b is None else f'%{b * 100:.0f}'}")
        time.sleep(1)


def mission(a):
    if a.onay != CONFIRM:
        raise SystemExit(f"Gerçek uçuş için --onay {CONFIRM} ver (alan boş, izinli, gözlemci var, "
                         f"manuel kumanda elinde).")
    from autonomy import search_and_deliver
    from payload import GripperPayload
    from vehicle import Px4Vehicle
    area = tuple(float(x) for x in a.alan.split(","))
    cam = OpenCVCamera(a.camera)
    det = CarDetector(a.model)
    v = Px4Vehicle(a.px4)
    fly_mission.SHOW_PREVIEW = a.goster
    results, marker, total, abort = search_and_deliver(
        v, cam.grab, det, GripperPayload(v), area, alt=a.irtifa, deliveries=a.teslimat, speed=a.hiz)
    print(f"Teslimat: {len(results)} | iniş: {'işaretçiyle' if marker else 'yerinde'} | süre {total:.0f} s"
          + (f" | İPTAL: {abort}" if abort else ""))
    cam.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("mod", choices=["bench", "connect", "mission"])
    ap.add_argument("--camera", default="0")
    ap.add_argument("--model", default="C:/UE/drone_runs/car_mixed/weights/best.pt")
    ap.add_argument("--px4", default="serial:///dev/ttyACM0:57600")
    ap.add_argument("--alan", default="-40,40,-30,30")
    ap.add_argument("--irtifa", type=float, default=25.0)
    ap.add_argument("--teslimat", type=int, default=1)
    ap.add_argument("--hiz", type=float, default=4.0)
    ap.add_argument("--sure", type=float, default=30.0, help="bench süresi (s)")
    ap.add_argument("--goster", action="store_true")
    ap.add_argument("--onay", default="")
    a = ap.parse_args()
    {"bench": bench, "connect": connect, "mission": mission}[a.mod](a)
