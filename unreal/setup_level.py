"""Unreal editörü içinde çalışır: zemini büyütür, rastgele 'arabalar' ve bir helipad yerleştirir.

Çalıştırma (Play kapalıyken), Output Log alt satırındaki komut kutusuna:
    py "C:/Users/sanbe/OneDrive/Masaüstü/D/unreal/setup_level.py"

Her çalıştırmada arabalar yeni rastgele konum, yön ve renklerle yeniden dizilir;
böylece farklı uçuşlarda farklı sahnelerden veri toplanır.
"""
import json
import random
from pathlib import Path

import unreal

N_CARS = 10
FOLDER = "DroneSim"           # Outliner'da bu klasörde toplanır, script tekrar çalışınca silinir
GAME = "/Game/DroneSim"
CAR_SIZE_M = (4.5, 1.9, 1.5)  # uzunluk, genişlik, yükseklik
CAR_COLORS = {
    "Red": (0.8, 0.05, 0.05), "Blue": (0.05, 0.15, 0.7), "White": (0.9, 0.9, 0.9),
    "Black": (0.03, 0.03, 0.03), "Silver": (0.5, 0.5, 0.55), "Yellow": (0.9, 0.7, 0.05),
}

mission = json.loads((Path(__file__).resolve().parent.parent / "mission" / "mission.json")
                     .read_text(encoding="utf-8"))
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
lib = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary


def mesh_copy(src, name):
    """AirSim tespiti mesh adına bakar; küpü 'SM_Car' adıyla kopyalayınca '*Car*' filtresine takılır."""
    path = f"{GAME}/{name}"
    if not lib.does_asset_exist(path):
        lib.duplicate_asset(src, path)
    return lib.load_asset(path)


def color_mat(name, rgb):
    path = f"{GAME}/{name}"
    if lib.does_asset_exist(path):
        return lib.load_asset(path)
    mi = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, GAME, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    mel.set_material_instance_parent(mi, lib.load_asset("/Engine/BasicShapes/BasicShapeMaterial"))
    mel.set_material_instance_vector_parameter_value(mi, "Color", unreal.LinearColor(*rgb, 1.0))
    lib.save_loaded_asset(mi)
    return mi


def spawn(mesh, loc, yaw, scale, mat, label):
    rot = unreal.Rotator()
    rot.yaw = yaw
    a = eas.spawn_actor_from_object(mesh, loc, rot)
    a.set_actor_scale3d(unreal.Vector(*scale))
    a.static_mesh_component.set_material(0, mat)
    a.set_actor_label(label)
    a.set_folder_path(FOLDER)
    return a


actors = eas.get_all_level_actors()

# 1) Önceki çalıştırmadan kalanları temizle
for a in actors:
    if str(a.get_folder_path()) == FOLDER:
        eas.destroy_actor(a)
actors = eas.get_all_level_actors()

# 2) AirSim koordinatlarının sıfırı PlayerStart'tır
starts = [a for a in actors if isinstance(a, unreal.PlayerStart)]
if not starts:
    raise RuntimeError("Haritada PlayerStart yok.")
origin = starts[0].get_actor_location()

# 3) Zemini bul, 300 m x 300 m'ye büyüt
floors = [a for a in actors if "floor" in a.get_actor_label().lower()]
if floors:
    floor = floors[0]
    _, ext = floor.get_actor_bounds(False)
    s = floor.get_actor_scale3d()
    f = 15000.0 / max(ext.x, ext.y)
    if f > 1.0:
        floor.set_actor_scale3d(unreal.Vector(s.x * f, s.y * f, s.z))
    c, ext = floor.get_actor_bounds(False)
    ground_z = c.z + ext.z
else:
    ground_z = origin.z - 50.0
    unreal.log_warning("Zemin bulunamadı, PlayerStart'ın 50 cm altı zemin kabul edildi.")


def ned_to_ue(x, y, height_cm):
    """NED metre (x ileri, y sağ) -> Unreal santimetre."""
    return unreal.Vector(origin.x + x * 100, origin.y + y * 100, ground_z + height_cm)


# 3b) Kargo paketi modeli: drop_test.py simülasyon sırasında bundan paket oluşturur.
# Malzeme mesh'in kendisine verilir ki oluşturulan her kopya kahverengi olsun.
pkg_mesh = mesh_copy("/Engine/BasicShapes/Cube", "SM_Package")
try:
    pkg_mesh.set_material(0, color_mat("MI_Package", (0.45, 0.28, 0.12)))
    lib.save_loaded_asset(pkg_mesh)
except Exception as e:
    unreal.log_warning(f"Paket rengi ayarlanamadı (paket beyaz görünecek): {e}")

# 4) Arabalar: rota boyunca rastgele, birbirine ve helipad'e çok yakın olmayacak şekilde
car_mesh = mesh_copy("/Engine/BasicShapes/Cube", "SM_Car")
car_mats = [color_mat(f"MI_Car_{n}", rgb) for n, rgb in CAR_COLORS.items()]
xs = [p[0] for p in mission["waypoints"]]
ys = [p[1] for p in mission["waypoints"]]
lx, ly = mission["landing"]
placed = []
car_names = []
tries = 0
while len(placed) < N_CARS and tries < 1000:
    tries += 1
    x = random.uniform(min(xs) - 5, max(xs) + 5)
    y = random.uniform(min(ys) - 5, max(ys) + 5)
    if (x - lx) ** 2 + (y - ly) ** 2 < 8 ** 2:
        continue
    if any((x - px) ** 2 + (y - py) ** 2 < 6 ** 2 for px, py in placed):
        continue
    placed.append((x, y))
    L, W, H = CAR_SIZE_M
    car = spawn(car_mesh, ned_to_ue(x, y, H * 50), random.uniform(0, 360), (L, W, H),
                random.choice(car_mats), f"Car_{len(placed):02d}")
    # Uçuş sırasında Python'dan yer değiştirebilmek için hareketli olmalı
    car.static_mesh_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    car_names.append(car.get_name())

# 5) Helipad: 6 m koyu gri zemin + beyaz "H"
pad_mesh = mesh_copy("/Engine/BasicShapes/Plane", "SM_Helipad")
cube = lib.load_asset("/Engine/BasicShapes/Cube")
spawn(pad_mesh, ned_to_ue(lx, ly, 1), 0, (6, 6, 1), color_mat("MI_Pad", (0.08, 0.08, 0.08)), "Helipad")
white = color_mat("MI_White", (0.95, 0.95, 0.95))
for dy in (-1.0, 1.0):  # H'nin iki dikey çubuğu
    spawn(cube, ned_to_ue(lx, ly + dy, 2), 0, (3.0, 0.4, 0.02), white, "Helipad_H")
spawn(cube, ned_to_ue(lx, ly, 2), 0, (0.4, 2.0, 0.02), white, "Helipad_H")  # orta çizgi

# Python tarafı (collect_data.py) arabaları bu isimlerle bulup her turda yeniden dağıtır
scene_file = Path(__file__).resolve().parent.parent / "mission" / "scene_objects.json"
scene_file.write_text(json.dumps({"cars": car_names, "car_height_m": CAR_SIZE_M[2],
                                  "ground_z_ned": (origin.z - ground_z) / 100.0}, indent=2),
                      encoding="utf-8")

unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).save_current_level()
unreal.log(f"DroneSim: {len(placed)} araba ve helipad yerleştirildi, harita kaydedildi.")
