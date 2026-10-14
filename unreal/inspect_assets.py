"""Editörde çalışır: indirilen paketlerdeki modellerin boyutlarını ve malzemelerini raporlar.

Sonuç hem Output Log'a hem de unreal/inspect_report.txt dosyasına yazılır.
"""
from pathlib import Path

import unreal

lib = unreal.EditorAssetLibrary
lines = []


def log(s):
    lines.append(s)
    unreal.log(s)


def describe_mesh(path):
    m = lib.load_asset(path)
    if not isinstance(m, unreal.StaticMesh):
        log(f"{path}: StaticMesh değil ({type(m).__name__})")
        return
    b = m.get_bounding_box()
    size = (b.max - b.min) / 100.0
    mats = [str(s.material_slot_name) for s in m.static_materials]
    log(f"{path}\n   boyut (m): {size.x:.2f} x {size.y:.2f} x {size.z:.2f}"
        f" | LOD: {m.get_num_lods()} | malzeme yuvası: {len(mats)} {mats[:12]}"
        f" | üçgen (LOD0): {m.get_num_triangles(0)}")


log("=== ARABA ===")
for p in lib.list_assets("/Game/Fab", recursive=True):
    if lib.find_asset_data(p).asset_class_path.asset_name == "StaticMesh":
        describe_mesh(p)

log("\n=== AĞAÇLAR (low + small) ===")
for folder in ["/Game/PN_interactiveSpruceForest/Meshes/full/low",
               "/Game/PN_interactiveSpruceForest/Meshes/half/low",
               "/Game/PN_interactiveSpruceForest/Meshes/small"]:
    for p in lib.list_assets(folder, recursive=False):
        describe_mesh(p)

log("\n=== ZEMİN MALZEMELERİ ===")
for folder in ["/Game/PN_interactiveSpruceForest/Materials/MaterialInstances/ground",
               "/Game/PN_interactiveSpruceForest/ExampleContent",
               "/Game/MOUT_Fabrication/Materials/Terrain/Material_Instances",
               "/Game/MOUT_Fabrication/Materials/Foliage/Material_Instances"]:
    for p in lib.list_assets(folder, recursive=True):
        log(f"   {p}")

log("\n=== MOUT: araç/doğa modelleri (ad eşleşmesi) ===")
keys = ("vehicle", "car", "truck", "rock", "grass", "bush", "tree", "foliage", "road")
for p in lib.list_assets("/Game/MOUT_Fabrication", recursive=True):
    name = p.split("/")[-1].lower()
    if name.startswith("sm_") and any(k in name for k in keys):
        log(f"   {p}")

out = Path(__file__).resolve().parent / "inspect_report.txt"
out.write_text("\n".join(lines), encoding="utf-8")
unreal.log(f"Rapor yazıldı: {out}")
