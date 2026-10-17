"""Editörde çalışır: NewMap'ten ForestMap oluşturur ve içine orman kurar.

- Orman zemini (MI_GroundSummer, doku tekrarı ayarlı)
- Ladin ormanı: drone'un iniş/bırakma yapabileceği açıklıklar bırakılarak
- Gerçekçi arabalar (Fab paketi), gövde rengi her arabada farklı, açıklıklara yerleşir
- Başlangıç açıklığında helipad, paket modeli
- Python tarafı için mission/scene_objects.json (araba isimleri, açıklıklar, güvenli irtifa)
- Biyom havuzları: yaz ladini, karlı ladin, çöl (palmiye/çöl ağacı/çalı), kaya. Hepsi hareketli;
  mission/world.py her turda bir biyom seçip havuzu yeniden dağıtır, kullanılmayanları park eder,
  zemin malzemesini değiştirir. Kurulumda varsayılan düzen = yaz ormanı (eski harita).

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
DROPZONE_OFFSET_M = 3.8    # teslimat işaretçisinin araç merkezine uzaklığı (yan taraf)
DROPZONE_SIZE_M = 2.67     # levha (beyaz kenar dahil)
DROPZONE_MARKER_M = 2.0    # siyah ArUco karesi
N_PEOPLE = 32              # arama-kurtarma: açıklıklarda kişiler (ayakta / yürüyen / koşan / yatan)
MANNEQUIN = "/Game/PN_interactiveSpruceForest/UE4_Mannequin/Mannequin/Character/Mesh/SK_Mannequin"
ANIMS = "/Game/PN_interactiveSpruceForest/UE4_Mannequin/Mannequin/Animations"
# duruş -> (animasyon, adet). Yürüme/koşma rastgele bir anda dondurulur: farklı bacak/kol pozları
PEOPLE_POSES = [("stand", "ThirdPersonIdle", 10), ("walk", "ThirdPersonWalk", 7),
                ("run", "ThirdPersonRun", 4), ("lie", None, 11)]
PARK = (3000.0, 3000.0)    # kullanılmayan havuz nesnelerinin park yeri (NED, m; kameranın çok dışında)
MOUT = "/Game/MOUT_Fabrication/Meshes"
WINTER = "/Game/PN_interactiveSpruceForest/ExampleContent/Winter/Meshes"
# havuz adı -> (mesh listesi, adet). Kurulumda yaz ormanı yerleşir, bunlar park edilir.
POOLS = {
    "winter": ([f"{WINTER}/full/low/winter_spruce_full_0{i}_low" for i in (1, 2, 3)] * 3
               + [f"{WINTER}/half/low/winter_spruce_half_0{i}_low" for i in (1, 2, 3, 4)] * 2
               + [f"{WINTER}/small/winter_spruce_small_0{i}" for i in (2, 4, 5, 6)], 330),
    "palm": ([f"{MOUT}/Foliage/PalmTrees/SM_PalmTree_{i:02d}" for i in range(1, 11)], 50),
    "desert_tree": ([f"{MOUT}/Foliage/DesertTree/SM_Desert_Tree_0{i}" for i in range(1, 7)], 70),
    "shrub": ([f"{MOUT}/Foliage/DesertShrub/SM_Desert_Shrub_0{i}" for i in range(1, 6)]
              + [f"{MOUT}/Foliage/DesertBush/SM_Shrub_{n}" for n in ("A", "Bare_A", "Bare_B", "B_01", "B_02",
                                                                     "B_03", "B_04", "B_05")], 160),
    "rock": ([f"{MOUT}/RockSet/SM_Rock_0{i}" for i in (1, 2, 3, 4)]
             + [f"{MOUT}/RockSet/SM_Pebles_0{i}" for i in (1, 2, 3, 4)], 120),
    "undergrowth": ([f"/Game/PN_interactiveSpruceForest/Meshes/small/spruce_small_0{i}" for i in (2, 3, 4, 5, 6, 7)], 90),
}
_PN = "/Game/PN_interactiveSpruceForest/Textures/MasterTextures"
_T = "/Game/MOUT_Fabrication/Textures/Terrain"
# zemin adı -> (doku, döşeme sayısı, renk çarpanı). Gerçek Material (simSetObjectMaterial instance
# yükleyemiyor). Çöl paketinin arazi dokuları birbirine çok benziyor: renk çarpanıyla çeşitlendirilir.
GROUND_TEXTURES = {
    "forest": (f"{_PN}/GroundTextures/ground_A", 100, None),
    "forest_dark": (f"{_PN}/GroundTextures/ground_A", 70, (0.55, 0.45, 0.35)),
    "moss": (f"{_PN}/CoverTextures/Moss_A", 100, None),
    "grass_dark": (f"{_PN}/CoverTextures/Moss_A", 80, (0.45, 0.62, 0.35)),
    "grass_dry": (f"{_PN}/CoverTextures/Moss_A", 80, (0.85, 0.75, 0.45)),
    "snow": (f"{_PN}/CoverTextures/Snow_A", 60, None),
    "snow_dirty": (f"{_PN}/CoverTextures/Snow_A", 45, (0.8, 0.8, 0.82)),
    "terrain_01": (f"{_T}/01/T_Terrain_01_Albedo", 40, None),
    "sand_red": (f"{_T}/04/T_Terrain_04_Albedo", 25, (1.0, 0.72, 0.55)),
    "sand_yellow": (f"{_T}/06/T_Terrain_06_Albedo", 25, (1.0, 0.9, 0.6)),
    "rock_gray": (f"{_T}/07/T_Terrain_07_Albedo", 30, (0.6, 0.62, 0.66)),
    "dirt_brown": (f"{_T}/09/T_Terrain_09_Albedo", 30, (0.6, 0.45, 0.32)),
}
CLOTH_COLORS = {   # kişilerin "kıyafet" rengi (manken tek malzemeli: tüm gövde)
    "Navy": (0.02, 0.04, 0.15), "Olive": (0.12, 0.14, 0.04), "Khaki": (0.35, 0.3, 0.18),
    "Red": (0.5, 0.03, 0.03), "OrangeVest": (0.9, 0.3, 0.02), "Gray": (0.2, 0.2, 0.2),
    "Black": (0.015, 0.015, 0.015), "Brown": (0.15, 0.07, 0.03), "SkyBlue": (0.25, 0.45, 0.7),
    "Yellow": (0.75, 0.6, 0.05), "Green": (0.04, 0.2, 0.06), "Purple": (0.18, 0.04, 0.25),
}

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
    "Yellow": (0.8, 0.6, 0.05), "Orange": (0.8, 0.3, 0.03), "DarkGray": (0.1, 0.1, 0.11),
    "Beige": (0.55, 0.45, 0.3), "LightBlue": (0.3, 0.5, 0.75), "Maroon": (0.25, 0.02, 0.04),
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
        # Kaydetmezsek sadece bellekte kalır: editör otomatik kapanıp açılınca model kaybolur ve
        # o modeli kullanan aktörler GÖRÜNMEZ olur (teslimat işaretçilerinde bu oldu).
        lib.save_asset(path, only_if_is_dirty=False)
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


def paint_material(name, rgb):
    """Tek renkli araba boyası, gerçek bir Material (instance değil).

    AirSim'in simSetObjectMaterial'ı sadece UMaterial yükleyebiliyor; collect_data.py
    her turda arabaların boyasını bunlardan rastgele değiştiriyor.
    """
    path = f"{GAME}/{name}"
    if lib.does_asset_exist(path):
        return lib.load_asset(path)
    m = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, GAME, unreal.Material, unreal.MaterialFactoryNew())
    color = mel.create_material_expression(m, unreal.MaterialExpressionConstant3Vector, -400, 0)
    color.set_editor_property("constant", unreal.LinearColor(*rgb, 1.0))
    mel.connect_material_property(color, "", unreal.MaterialProperty.MP_BASE_COLOR)
    for prop, value, y in ((unreal.MaterialProperty.MP_ROUGHNESS, 0.3, 150),
                           (unreal.MaterialProperty.MP_METALLIC, 0.4, 250)):
        k = mel.create_material_expression(m, unreal.MaterialExpressionConstant, -400, y)
        k.set_editor_property("r", value)
        mel.connect_material_property(k, "", prop)
    mel.recompile_material(m)
    lib.save_loaded_asset(m)
    return m


def cloth_material(name, rgb):
    """Mat kıyafet rengi (metalik değil); kişilere her turda world.py atar."""
    path = f"{GAME}/{name}"
    if lib.does_asset_exist(path):
        return lib.load_asset(path)
    m = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, GAME, unreal.Material, unreal.MaterialFactoryNew())
    color = mel.create_material_expression(m, unreal.MaterialExpressionConstant3Vector, -400, 0)
    color.set_editor_property("constant", unreal.LinearColor(*rgb, 1.0))
    mel.connect_material_property(color, "", unreal.MaterialProperty.MP_BASE_COLOR)
    k = mel.create_material_expression(m, unreal.MaterialExpressionConstant, -400, 150)
    k.set_editor_property("r", 0.85)
    mel.connect_material_property(k, "", unreal.MaterialProperty.MP_ROUGHNESS)
    mel.recompile_material(m)
    lib.save_loaded_asset(m)
    return m


def tiled_material(name, tex_path, tiling, tint=None):
    """Dokuyu zemine `tiling` kez döşeyen gerçek Material (runtime'da zemine atanabilsin)."""
    path = f"{GAME}/{name}"
    if lib.does_asset_exist(path):
        return lib.load_asset(path)
    tex = lib.load_asset(tex_path)
    if tex is None:
        unreal.log_warning(f"doku yok: {tex_path}")
        return None
    m = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, GAME, unreal.Material, unreal.MaterialFactoryNew())
    uv = mel.create_material_expression(m, unreal.MaterialExpressionTextureCoordinate, -800, 0)
    uv.set_editor_property("u_tiling", float(tiling))
    uv.set_editor_property("v_tiling", float(tiling))
    sample = mel.create_material_expression(m, unreal.MaterialExpressionTextureSample, -400, 0)
    sample.set_editor_property("texture", tex)
    # örnekleyici tipi dokuya uymazsa malzeme derlenmez (siyah/varsayılan görünür)
    sample.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_COLOR
                               if tex.get_editor_property("srgb") else unreal.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR)
    mel.connect_material_expressions(uv, "", sample, "UVs")
    if tint is None:
        mel.connect_material_property(sample, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
    else:   # renk çarpanı: aynı dokudan farklı zemin
        col = mel.create_material_expression(m, unreal.MaterialExpressionConstant3Vector, -400, -200)
        col.set_editor_property("constant", unreal.LinearColor(*tint, 1.0))
        mul = mel.create_material_expression(m, unreal.MaterialExpressionMultiply, -200, 0)
        mel.connect_material_expressions(sample, "RGB", mul, "A")
        mel.connect_material_expressions(col, "", mul, "B")
        mel.connect_material_property(mul, "", unreal.MaterialProperty.MP_BASE_COLOR)
    rough = mel.create_material_expression(m, unreal.MaterialExpressionConstant, -400, 250)
    rough.set_editor_property("r", 0.95)
    mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    mel.recompile_material(m)
    lib.save_loaded_asset(m)
    return m


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


def spawn(mesh, loc, yaw, scale, label, folder=FOLDER, material=None, movable=False):
    rot = unreal.Rotator()
    rot.yaw = yaw
    a = eas.spawn_actor_from_object(mesh, loc, rot)
    if movable:   # world.py her turda yerini değiştirir (Static mobility'de simSetObjectPose başarısız)
        a.static_mesh_component.set_mobility(unreal.ComponentMobility.MOVABLE)
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
floor.static_mesh_component.set_mobility(unreal.ComponentMobility.MOVABLE)   # runtime malzeme değişimi
ground_mats = {}
for gname, (tex, tiling, tint) in GROUND_TEXTURES.items():
    gm = tiled_material(f"M_Ground_{gname}", tex, tiling, tint)
    if gm is not None:
        ground_mats[gname] = gm.get_path_name()


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
pools = {}   # havuz adı -> aktör adları (world.py için)
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
                t = spawn(rng.choice(tree_meshes), ned_to_ue(tx, ty), rng.uniform(0, 360),
                          rng.uniform(0.8, 1.2), f"Tree_{n_trees:04d}", f"{FOLDER}/Forest", movable=True)
                pools.setdefault("spruce", []).append(t.get_name())
                n_trees += 1
            elif rng.random() < 0.08 and in_clearing(tx, ty) and not in_clearing(tx, ty, pad=-4.0):
                # açıklık kenarında küçük fidanlar
                b = spawn(rng.choice(bush_meshes), ned_to_ue(tx, ty), rng.uniform(0, 360),
                          rng.uniform(0.8, 1.3), f"Bush_{n_bush:03d}", f"{FOLDER}/Forest", movable=True)
                pools.setdefault("sapling", []).append(b.get_name())
                n_bush += 1
            y += TREE_SPACING_M
        x += TREE_SPACING_M

# --- 5b) Diğer biyom havuzları: park yerinde, görünmez boyutta (world.py yerleştirir) ---
for pool, (paths, count) in POOLS.items():
    meshes = [m for m in (lib.load_asset(pth) for pth in paths) if m is not None]
    if not meshes:
        unreal.log_warning(f"havuz {pool}: mesh bulunamadı")
        continue
    names = []
    for k in range(count):
        a = spawn(rng.choice(meshes), ned_to_ue(PARK[0] + k * 0.5, PARK[1], -5000), 0, 0.01,
                  f"{pool}_{k:03d}", f"{FOLDER}/Forest", movable=True)
        names.append(a.get_name())
    pools[pool] = names

# --- 6) Arabalar: gerçek model, gövde rengi farklı, açıklıklara ---
car_mesh = mesh_copy(CAR_SRC, "SM_Car_Sedan")
bb = car_mesh.get_bounding_box()
car_pivot_h = -bb.min.z / 100.0           # pivot'un tekerlek altından yüksekliği
car_height = (bb.max.z - bb.min.z) / 100.0
car_mats = [paint_material(f"M_CarPaint_{n}", rgb) for n, rgb in CAR_COLORS.items()]


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
def texture_material(name):
    """unreal/assets/T_<name>.png dokusunu içe aktarıp M_<name> malzemesi yapar (keskin kenarlı).

    ArUco işaretçileri için: drone iniş/teslimat noktasını Unreal'in bilgisinden değil, işaretçiyi
    kamerayla görerek bulur (gerçek hassas iniş ve teslimat sistemleri de böyle çalışır).
    """
    tex_path = f"{GAME}/T_{name}"
    if not lib.does_asset_exist(tex_path):
        task = unreal.AssetImportTask()
        task.filename = str(ROOT / "unreal" / "assets" / f"T_{name}.png")
        task.destination_path = GAME
        task.automated = True
        task.replace_existing = True
        task.save = True
        unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    tex = lib.load_asset(tex_path)
    tex.set_editor_property("filter", unreal.TextureFilter.TF_NEAREST)        # keskin kenar
    tex.set_editor_property("mip_gen_settings", unreal.TextureMipGenSettings.TMGS_NO_MIPMAPS)
    lib.save_loaded_asset(tex)
    path = f"{GAME}/M_{name}"
    if lib.does_asset_exist(path):
        return lib.load_asset(path)
    m = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        f"M_{name}", GAME, unreal.Material, unreal.MaterialFactoryNew())
    sample = mel.create_material_expression(m, unreal.MaterialExpressionTextureSample, -400, 0)
    sample.set_editor_property("texture", tex)
    mel.connect_material_property(sample, "RGB", unreal.MaterialProperty.MP_BASE_COLOR)
    rough = mel.create_material_expression(m, unreal.MaterialExpressionConstant, -400, 250)
    rough.set_editor_property("r", 0.9)
    mel.connect_material_property(rough, "", unreal.MaterialProperty.MP_ROUGHNESS)
    mel.recompile_material(m)
    lib.save_loaded_asset(m)
    return m


pad_mesh = mesh_copy("/Engine/BasicShapes/Plane", "SM_Helipad")
basic = "/Engine/BasicShapes/BasicShapeMaterial"
# Helipad: 6 m = 4 m ArUco (DICT_4X4_50, id 0) + 1 m beyaz kenar.
# Plane'in UV'si 0-1: doku tam oturur; yaw 0 -> işaretçinin "üstü" kuzey (NED x+)
spawn(pad_mesh, ned_to_ue(lx, ly, 1), 0, (6, 6, 1), "Helipad", material=texture_material("HelipadAruco"))

# Teslimat noktaları: her aracın yanında numaralı ArUco (id 1-10, 2 m işaretçi, 2.67 m levha).
# Paket aracın tavanına değil bu işaretçiye bırakılır (teslimat drone'larının çalışma biçimi).
dz_mesh = mesh_copy("/Engine/BasicShapes/Plane", "SM_DropZone")
dropzones = []
for i, name in enumerate(car_names, start=1):
    car = next(a for a in eas.get_all_level_actors() if a.get_name() == name)
    loc, yaw = car.get_actor_location(), math.radians(car.get_actor_rotation().yaw)
    # aracın yan tarafına (mesh x = genişlik ekseni) DROPZONE_OFFSET_M
    off = DROPZONE_OFFSET_M * 100
    pad_loc = unreal.Vector(loc.x + off * math.cos(yaw), loc.y + off * math.sin(yaw), ground_z + 1)
    pad = spawn(dz_mesh, pad_loc, 0, (DROPZONE_SIZE_M, DROPZONE_SIZE_M, 1), f"DropZone_{i:02d}",
                material=texture_material(f"DropZone_{i:02d}"))
    pad.static_mesh_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    dropzones.append(pad.get_name())

# Kişiler (arama-kurtarma): açıklıklarda, araçlardan ve iniş noktasından uzak.
# Ayakta olanlar nefes alma animasyonunda; yatanlar yaralı (yere paralel). Tespit filtresi mesh adına
# bakar ("*Mannequin*"). Paket KİŞİNİN ÜSTÜNE DEĞİL güvenli mesafede yanına bırakılır (autonomy sar modu).
people, people_pose = [], []
mannequin = lib.load_asset(MANNEQUIN)
cloth_mats = [cloth_material(f"M_Cloth_{n}", rgb) for n, rgb in CLOTH_COLORS.items()]
poses = [(pose, anim) for pose, anim, n in PEOPLE_POSES for _ in range(n)]
for k, (pose, anim_name) in enumerate(poses[:N_PEOPLE]):
    for _ in range(500):
        cx, cy, cr = rng.choice(clearings[1:])
        a, d = rng.uniform(0, 2 * math.pi), rng.uniform(0, max(cr - 3.0, 0.5))
        x, y = cx + d * math.cos(a), cy + d * math.sin(a)
        if (all(math.hypot(x - px, y - py) > 4 for px, py in placed) and
                math.hypot(x - lx, y - ly) > 12):
            break
    placed.append((x, y))
    lying = pose == "lie"
    rot = unreal.Rotator()
    rot.yaw = rng.uniform(0, 360)
    if lying:
        rot.roll = 90.0
    person = eas.spawn_actor_from_object(mannequin, ned_to_ue(x, y, 15 if lying else 0), rot)
    person.set_actor_label(f"Person_{k + 1:02d}")
    person.set_folder_path(FOLDER)
    comp = person.skeletal_mesh_component
    comp.set_mobility(unreal.ComponentMobility.MOVABLE)
    # kıyafet rengi burada kalıcı verilir: AirSim'in simSetObjectMaterial'ı iskeletli mesh'te
    # True döndürüp hiçbir şey yapmıyor. %20'si varsayılan beyaz manken kalır.
    if rng.random() > 0.2:
        comp.set_material(0, rng.choice(cloth_mats))
    anim = lib.load_asset(f"{ANIMS}/{anim_name}") if anim_name else None
    if anim is not None:
        comp.set_animation_mode(unreal.AnimationMode.ANIMATION_SINGLE_NODE)
        frozen = pose in ("walk", "run")   # adımın rastgele bir anında donmuş poz
        comp.override_animation_data(anim, not frozen, not frozen, rng.uniform(0, 0.9), 0.0 if frozen else 1.0)
    people.append(person.get_name())
    people_pose.append(pose)

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
    "paint_materials": [m.get_path_name() for m in car_mats],  # simSetObjectMaterial için
    "dropzones": dropzones,              # cars[i] aracının teslimat noktası; ArUco id = i + 1
    "dropzone_offset_m": DROPZONE_OFFSET_M,
    "dropzone_marker_m": DROPZONE_MARKER_M,
    "people": people,                    # arama-kurtarma kişileri
    "people_pose": people_pose,          # "stand" / "walk" / "run" / "lie"
    "floor": floor.get_name(),
    "ground_materials": ground_mats,     # biyom zemini adı -> Material yolu
    "cloth_materials": [m.get_path_name() for m in cloth_mats],
    "pools": pools,                      # spruce/sapling (kurulumda yerleşik), winter/palm/... (park)
    "park": list(PARK),
    "forest_seed": FOREST_SEED,
    "tree_spacing_m": TREE_SPACING_M,
    "forest_margin_m": FOREST_MARGIN_M,
}
(ROOT / "mission" / "scene_objects.json").write_text(json.dumps(scene, indent=2), encoding="utf-8")

les.save_current_level()
unreal.log(f"DroneSim orman: {n_trees} ağaç, {n_bush} fidan, {len(clearings)} açıklık, "
           f"{len(car_names)} araba (yükseklik {car_height:.2f} m), {len(people)} kişi, "
           f"havuzlar {dict((k, len(v)) for k, v in pools.items())}, {len(ground_mats)} zemin. ForestMap kaydedildi.")
