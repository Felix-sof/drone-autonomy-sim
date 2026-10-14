"""Editörde çalışır: NewMap'ten ForestMap oluşturur ve içine orman kurar.

- Orman zemini (MI_GroundSummer, doku tekrarı ayarlı)
- Ladin ormanı: drone'un iniş/bırakma yapabileceği açıklıklar bırakılarak
- Gerçekçi arabalar (Fab paketi), gövde rengi her arabada farklı, açıklıklara yerleşir
- Başlangıç açıklığında helipad, paket modeli
- Python tarafı için mission/scene_objects.json (araba isimleri, açıklıklar, güvenli irtifa)

Çalıştırma (Play kapalıyken), Output Log komut kutusuna:
    py "C:/UE/setup_forest.py"
Tekrar çalıştırınca orman aynı kalır (sabit tohum), arabalar yeniden dağılır.
"""
import json
import math
import random
from pathlib import Path

import unreal

FOREST_SEED = 7            # orman düzeni sabit; değiştirirsen başka bir orman çıkar
TREE_SPACING_M = 7.5       # ağaçlar arası ortalama mesafe; FPS düşükse büyüt (6.0'da ~490 ağaç GPU'yu %99'a çıkardı)
AREA = (-45, 45, -35, 55)  # görev alanı (NED x_min, x_max, y_min, y_max, metre)
FOREST_MARGIN_M = 30       # görev alanının dışına da orman uzansın
N_CLEARINGS = 9            # iniş/araba açıklıkları (başlangıç açıklığı hariç)
N_CARS = 10
CRUISE_ALT_M = 25          # en uzun ağaç ~17 m; seyir irtifası bunun üstünde

FOLDER = "DroneSim"
GAME = "/Game/DroneSim"
CAR_SRC = "/Game/Fab/Cars_for_Arcade_Demolition_Racing_Games/source/StaticMeshes/source"
SPRUCE = "/Game/PN_interactiveSpruceForest/Meshes"
TREES = ([f"{SPRUCE}/full/low/spruce_full_0{i}_low" for i in (1, 2, 3)] * 3
         + [f"{SPRUCE}/half/low/spruce_half_0{i}_low" for i in (1, 2, 3, 4)] * 2
         + [f"{SPRUCE}/small/spruce_small_0{i}" for i in (5, 6, 7)])
UNDERGROWTH = [f"{SPRUCE}/small/spruce_small_0{i}" for i in (2, 3, 4)]
CAR_COLORS = {
    "Red": (0.6, 0.03, 0.03), "Blue": (0.03, 0.1, 0.5), "White": (0.85, 0.85, 0.85),
    "Black": (0.02, 0.02, 0.02), "Silver": (0.45, 0.45, 0.5), "Green": (0.05, 0.25, 0.08),
    "Yellow": (0.8, 0.6, 0.05), "Orange": (0.8, 0.3, 0.03),
}

ROOT = Path(__file__).resolve().parent.parent
mission = json.loads((ROOT / "mission" / "mission.json").read_text(encoding="utf-8"))
eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
lib = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary


def mesh_copy(src, name):
    """AirSim tespiti mesh adına bakar; kopya adı '*Car*' gibi filtrelere uysun diye."""
    path = f"{GAME}/{name}"
    if not lib.does_asset_exist(path):
        lib.duplicate_asset(src, path)
    return lib.load_asset(path)


def material_instance(name, parent_path, vectors=None, scalars=None):
    path = f"{GAME}/{name}"
    if lib.does_asset_exist(path):
        mi = lib.load_asset(path)
    else:
        mi = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            name, GAME, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        mel.set_material_instance_parent(mi, lib.load_asset(parent_path))
    for k, v in (vectors or {}).items():
        mel.set_material_instance_vector_parameter_value(mi, k, unreal.LinearColor(*v, 1.0))
    for k, v in (scalars or {}).items():
        mel.set_material_instance_scalar_parameter_value(mi, k, v)
    lib.save_loaded_asset(mi)
    return mi


def freeze_foliage(actor):
    """Ağacın rüzgarda sallanmasını (World Position Offset) kapatır.

    Bu paket ağaçları her karede hareket ettiriyor; Virtual Shadow Maps (AirSim kamerası
    için açık kalmak zorunda) da bu yüzden her karede bütün ormanın gölgesini yeniden
    çiziyor. Kapatınca GPU yükü ciddi düşer; drone fotoğrafı için sallanma gereksiz.
    """
    smc = actor.static_mesh_component
    for prop, value in (("evaluate_world_position_offset", False),
                        ("world_position_offset_disable_distance", 1),
                        ("shadow_cache_invalidation_behavior", unreal.ShadowCacheInvalidationBehavior.STATIC)):
        try:
            smc.set_editor_property(prop, value)
        except Exception:
            pass  # bu sürümde olmayan özellik


def spawn(mesh, loc, yaw, scale, label, folder=FOLDER, material=None):
    rot = unreal.Rotator()
    rot.yaw = yaw
    a = eas.spawn_actor_from_object(mesh, loc, rot)
    if folder.endswith("Forest"):
        freeze_foliage(a)
    a.set_actor_scale3d(unreal.Vector(*scale) if isinstance(scale, tuple) else unreal.Vector(scale, scale, scale))
    if material is not None:
        a.static_mesh_component.set_material(0, material)
    a.set_actor_label(label)
    a.set_folder_path(folder)
    return a


# --- 1) ForestMap'i aç (yoksa NewMap şablonundan oluştur) ---
# Dikkat: eski haritanın World nesnesine Python'da referans tutulursa harita değişirken
# Unreal "World Memory Leaks" diye çöker. O yüzden yol kontrolü tek ifadede, değişken yok.
# (duplicate_asset + load_map da bu çökmeye yol açtı; new_level_from_template doğru yol.)
if not unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world().get_path_name().startswith("/Game/ForestMap"):
    if lib.does_asset_exist("/Game/ForestMap"):
        les.load_level("/Game/ForestMap")
    else:
        les.new_level_from_template("/Game/ForestMap", "/Game/NewMap")

# --- 2) Önceki çalıştırmadan kalanları temizle ---
for a in eas.get_all_level_actors():
    if str(a.get_folder_path()).startswith(FOLDER):
        eas.destroy_actor(a)
actors = eas.get_all_level_actors()

starts = [a for a in actors if isinstance(a, unreal.PlayerStart)]
if not starts:
    raise RuntimeError("Haritada PlayerStart yok.")
origin = starts[0].get_actor_location()

# --- 3) Zemin: 300 m, orman zemini malzemesi ---
floors = [a for a in actors if "floor" in a.get_actor_label().lower()]
if not floors:
    raise RuntimeError("Zemin (Floor) bulunamadı.")
floor = floors[0]
_, ext = floor.get_actor_bounds(False)
s = floor.get_actor_scale3d()
f = 15000.0 / max(ext.x, ext.y)
if abs(f - 1.0) > 0.01:
    floor.set_actor_scale3d(unreal.Vector(s.x * f, s.y * f, s.z))
c, ext = floor.get_actor_bounds(False)
ground_z = c.z + ext.z
ground_mi = material_instance(
    "MI_ForestGround", "/Game/PN_interactiveSpruceForest/Materials/MaterialInstances/ground/MI_GroundSummer",
    scalars={"Tiling": 100.0})  # 300 m zeminde ~3 m'de bir tekrar
floor.static_mesh_component.set_material(0, ground_mi)


def ned_to_ue(x, y, height_cm=0.0):
    return unreal.Vector(origin.x + x * 100, origin.y + y * 100, ground_z + height_cm)


# --- 4) Açıklıklar: başlangıç/iniş açıklığı + rastgele açıklıklar (sabit tohum) ---
rng = random.Random(FOREST_SEED)
lx, ly = mission["landing"]
clearings = [(lx, ly, 12.0)]
tries = 0
while len(clearings) < N_CLEARINGS + 1 and tries < 5000:
    tries += 1
    x, y, r = rng.uniform(AREA[0], AREA[1]), rng.uniform(AREA[2], AREA[3]), rng.uniform(9, 13)
    if all(math.hypot(x - cx, y - cy) > r + cr + 8 for cx, cy, cr in clearings):
        clearings.append((x, y, r))


def in_clearing(x, y, pad=0.0):
    return any(math.hypot(x - cx, y - cy) < cr + pad for cx, cy, cr in clearings)


# --- 5) Orman: hafif rastgele ızgara, açıklıkların dışına ---
tree_meshes = [lib.load_asset(p) for p in TREES]
bush_meshes = [lib.load_asset(p) for p in UNDERGROWTH]
x0, x1 = AREA[0] - FOREST_MARGIN_M, AREA[1] + FOREST_MARGIN_M
y0, y1 = AREA[2] - FOREST_MARGIN_M, AREA[3] + FOREST_MARGIN_M
n_trees = n_bush = 0
with unreal.ScopedSlowTask(1, "Orman dikiliyor...") as task:
    task.make_dialog(True)
    x = x0
    while x < x1:
        y = y0
        while y < y1:
            tx = x + rng.uniform(-0.4, 0.4) * TREE_SPACING_M
            ty = y + rng.uniform(-0.4, 0.4) * TREE_SPACING_M
            # açıklık kenarı düzgün daire olmasın diye pay rastgele
            if not in_clearing(tx, ty, pad=rng.uniform(1.0, 4.0)):
                spawn(rng.choice(tree_meshes), ned_to_ue(tx, ty), rng.uniform(0, 360),
                      rng.uniform(0.8, 1.2), f"Tree_{n_trees:04d}", f"{FOLDER}/Forest")
                n_trees += 1
            elif rng.random() < 0.08 and in_clearing(tx, ty) and not in_clearing(tx, ty, pad=-4.0):
                # açıklık kenarında küçük fidanlar
                spawn(rng.choice(bush_meshes), ned_to_ue(tx, ty), rng.uniform(0, 360),
                      rng.uniform(0.8, 1.3), f"Bush_{n_bush:03d}", f"{FOLDER}/Forest")
                n_bush += 1
            y += TREE_SPACING_M
        x += TREE_SPACING_M

# --- 6) Arabalar: gerçek model, gövde rengi farklı, açıklıklara ---
car_mesh = mesh_copy(CAR_SRC, "SM_Car_Sedan")
bb = car_mesh.get_bounding_box()
car_pivot_h = -bb.min.z / 100.0           # pivot'un tekerlek altından yüksekliği
car_height = (bb.max.z - bb.min.z) / 100.0
car_mats = [material_instance(f"MI_CarBody_{n}", "/Engine/BasicShapes/BasicShapeMaterial",
                              vectors={"Color": rgb}) for n, rgb in CAR_COLORS.items()]


def car_spot(placed):
    """Rastgele bir açıklıkta, kenardan ve diğer arabalardan uzak bir nokta (iniş açıklığı hariç)."""
    for _ in range(500):
        cx, cy, cr = rng.choice(clearings[1:])
        a, d = rng.uniform(0, 2 * math.pi), rng.uniform(0, max(cr - 5.0, 0.5))
        x, y = cx + d * math.cos(a), cy + d * math.sin(a)
        if all(math.hypot(x - px, y - py) > 7 for px, py in placed):
            return x, y
    return None


car_names, placed = [], []
for i in range(N_CARS):
    spot = car_spot(placed)
    if spot is None:
        break
    placed.append(spot)
    car = spawn(car_mesh, ned_to_ue(*spot, car_pivot_h * 100), rng.uniform(0, 360), 1.0,
                f"Car_{i + 1:02d}", material=rng.choice(car_mats))
    car.static_mesh_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    car_names.append(car.get_name())

# --- 7) Helipad ve paket modeli ---
pad_mesh = mesh_copy("/Engine/BasicShapes/Plane", "SM_Helipad")
cube = lib.load_asset("/Engine/BasicShapes/Cube")
basic = "/Engine/BasicShapes/BasicShapeMaterial"
spawn(pad_mesh, ned_to_ue(lx, ly, 1), 0, (6, 6, 1), "Helipad",
      material=material_instance("MI_Pad", basic, vectors={"Color": (0.08, 0.08, 0.08)}))
white = material_instance("MI_White", basic, vectors={"Color": (0.95, 0.95, 0.95)})
for dy in (-1.0, 1.0):
    spawn(cube, ned_to_ue(lx, ly + dy, 2), 0, (3.0, 0.4, 0.02), "Helipad_H", material=white)
spawn(cube, ned_to_ue(lx, ly, 2), 0, (0.4, 2.0, 0.02), "Helipad_H", material=white)

pkg_mesh = mesh_copy("/Engine/BasicShapes/Cube", "SM_Package")
try:
    pkg_mesh.set_material(0, material_instance("MI_Package", basic, vectors={"Color": (0.45, 0.28, 0.12)}))
    lib.save_loaded_asset(pkg_mesh)
except Exception as e:
    unreal.log_warning(f"Paket rengi ayarlanamadı: {e}")

# --- 8) Python tarafı için sahne bilgisi ---
scene = {
    "map": "ForestMap",
    "cars": car_names,
    "car_height_m": round(car_height, 3),
    "car_pivot_height_m": round(car_pivot_h, 3),
    "ground_z_ned": (origin.z - ground_z) / 100.0,
    "area": list(AREA),
    "clearings": [[round(x, 2), round(y, 2), round(r, 2)] for x, y, r in clearings],
    "cruise_alt_m": CRUISE_ALT_M,
}
(ROOT / "mission" / "scene_objects.json").write_text(json.dumps(scene, indent=2), encoding="utf-8")

les.save_current_level()
unreal.log(f"DroneSim orman: {n_trees} ağaç, {n_bush} fidan, {len(clearings)} açıklık, "
           f"{len(car_names)} araba (yükseklik {car_height:.2f} m). ForestMap kaydedildi.")
