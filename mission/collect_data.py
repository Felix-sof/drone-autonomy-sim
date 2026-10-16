"""Otomatik veri toplama: her turda arabaları rastgele dağıt, rastgele bir rota uç, helipad'e in, tekrarla.

Her tur farklı bir sahne demek: arabaların yeri, yönü, boyu, uçuş rotası, irtifa,
saat ve hava durumu değişir. Model tek bir sahneyi ezberlemek yerine genelleşmeyi öğrenir.

Kullanım:
  python mission/collect_data.py              # 10 tur
  python mission/collect_data.py --turlar 3
Durdurmak için Ctrl+C (o ana kadar kaydedilenler kalır).
"""
import argparse
import json
import math
import time

import numpy as np
from cosysairsim.utils import euler_to_quaternion

import fly_mission
from fly_mission import Recorder, fly_once
from sim_client import ROOT, airsim, connect, load_mission

FLAT_AREA = (-25, 25, -5, 25)  # düz harita (setup_level.py) için görev alanı


def area(scene):
    return tuple(scene.get("area", FLAT_AREA))


def scatter_cars(client, scene, landing, rng):
    """Arabaları rastgele dağıtır: birbirine ve helipad'e çok yakın değil.

    Orman haritasında (setup_forest.py) sadece açıklıklara, gerçek araba modeli biraz
    büyütülüp küçültülerek; düz haritada (setup_level.py) kutular alanın her yerine.
    """
    lx, ly = landing
    clearings = scene.get("clearings", [])[1:]  # ilki iniş açıklığı
    a = area(scene)
    placed = []
    for name in scene["cars"]:
        for _ in range(500):
            if clearings:
                cx, cy, cr = clearings[rng.integers(len(clearings))]
                ang, d = rng.uniform(0, 2 * math.pi), rng.uniform(0, max(cr - 5.0, 0.5))
                x, y = cx + d * math.cos(ang), cy + d * math.sin(ang)
            else:
                x, y = rng.uniform(a[0], a[1]), rng.uniform(a[2], a[3])
            if math.hypot(x - lx, y - ly) > 8 and all(math.hypot(x - px, y - py) > 7 for px, py in placed):
                break
        placed.append((x, y))
        if "car_pivot_height_m" in scene:  # gerçek model: oranı koruyarak ölçekle
            s = rng.uniform(0.9, 1.1)
            client.simSetObjectScale(name, airsim.Vector3r(s, s, s))
            z = scene["ground_z_ned"] - scene["car_pivot_height_m"] * s
        else:  # kutu: boyutlar ayrı ayrı
            length, width, height = rng.uniform(3.8, 5.2), rng.uniform(1.7, 2.0), rng.uniform(1.4, 1.9)
            client.simSetObjectScale(name, airsim.Vector3r(length, width, height))
            z = scene["ground_z_ned"] - height / 2
        pose = airsim.Pose(airsim.Vector3r(x, y, z), euler_to_quaternion(0, 0, rng.uniform(-math.pi, math.pi)))
        if not client.simSetObjectPose(name, pose, True):
            print(f"  uyarı: {name} taşınamadı (harita scriptini tekrar çalıştır)")
        paints = scene.get("paint_materials")
        if paints:  # gövde (malzeme yuvası 0) rengini değiştir: model renge değil şekle odaklansın
            client.simSetObjectMaterial(name, paints[rng.integers(len(paints))], 0)


def hover_shots(client, cfg, scene, rec, rng, z, n_cars=3, per_car=2):
    """Birkaç arabanın üstünde (biraz kaydırarak) dümdüz durup çekim yapar.

    Uçarken çekilen karelerde drone eğik ve arabalar çoğu zaman kenarda; bırakma
    sırasında ise drone durgun ve araba merkezde. Model bu açıyı da görmeli.
    """
    from fly_mission import get_boxes, get_rgb  # noqa: döngüsel import olmasın diye burada
    cam = cfg["camera"]
    for i in rng.choice(len(scene["cars"]), size=min(n_cars, len(scene["cars"])), replace=False):
        p = client.simGetObjectPose(scene["cars"][i]).position
        for _ in range(per_car):
            ox, oy = rng.uniform(-6, 6, size=2)
            client.moveToPositionAsync(p.x_val + ox, p.y_val + oy, z, 5).join()
            client.hoverAsync().join()
            time.sleep(0.8)
            rec.save(get_rgb(client, cam), get_boxes(client, cam))


def random_route(rng, scene, n=5):
    """Alanın içinde rastgele n nokta; merkez etrafında açıya göre sıralı, böylece rota kendini kesmez."""
    a = area(scene)
    pts = [(rng.uniform(a[0], a[1]), rng.uniform(a[2], a[3])) for _ in range(n)]
    cx, cy = np.mean(pts, axis=0)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--turlar", type=int, default=10)
    ap.add_argument("--goster", action="store_true", help="Küçük kamera önizleme penceresi")
    args = ap.parse_args()
    fly_mission.SHOW_PREVIEW = args.goster

    cfg = load_mission()
    scene_file = ROOT / "mission" / "scene_objects.json"
    if not scene_file.exists():
        raise SystemExit("mission/scene_objects.json yok: önce Unreal'de setup_level.py'yi çalıştır.")
    scene = json.loads(scene_file.read_text(encoding="utf-8"))

    client = connect()
    rng = np.random.default_rng()
    total_imgs = 0
    try:
        for i in range(1, args.turlar + 1):
            print(f"\n=== Tur {i}/{args.turlar} ===")
            scatter_cars(client, scene, cfg["landing"], rng)
            rec = Recorder()
            # ormanda ağaçların üstünde kal (en uzun ladin ~17 m)
            lo = scene.get("cruise_alt_m", 15)
            z = scene["ground_z_ned"] - rng.uniform(lo, lo + 12)
            found = fly_once(client, cfg, rec, None, random_route(rng, scene), z, rng,
                             before_landing=lambda c: hover_shots(c, cfg, scene, rec, rng, z))
            total_imgs += rec.count
            print(f"Tur {i}: {rec.count} görüntü, {len(found)} karede araba. Toplam: {total_imgs}")
            if rec.count < 5:
                print("Bu turda neredeyse hiç kare kaydedilmedi: kamera donmuş/siyah olabilir. Duruyorum.")
                break
    except KeyboardInterrupt:
        print("\nDurduruldu.")
    finally:
        client.enableApiControl(False)
    print(f"\nToplam {total_imgs} görüntü kaydedildi -> dataset/raw/")


if __name__ == "__main__":
    main()
