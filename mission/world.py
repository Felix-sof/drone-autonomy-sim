"""Her turda dünyayı değiştirir: biyom (zemin + bitki örtüsü), açıklıkların yeri, ağaç sıklığı.

Model "beyaz manken + turuncu orman toprağı"nı ezberlemesin diye veri toplamada ve görev
testlerinde kullanılır. unreal/setup_forest.py'nin kurduğu havuzları kullanır (scene_objects.json):
    pools       spruce, sapling (yaz ladini), winter (karlı ladin), palm, desert_tree, shrub, rock
    ground_materials   zemin adı -> Material yolu
Kullanılmayan havuz nesneleri park yerine (kameranın çok dışına) çekilip küçültülür.

    clearings = randomize_world(client, scene, rng)          # rastgele biyom
    clearings = randomize_world(client, scene, rng, "col")   # belirli biyom
"""
import math

from cosysairsim.utils import euler_to_quaternion

from sim_client import airsim

# biyom -> zemin adayları ve havuz yerleşimi {havuz: (yerleşim, oran)}
#   forest  : açıklıklar dışında, ladin aralığında hafif rastgele ızgara (sık orman)
#   scatter : açıklıklar dışında rastgele, havuzun `oran` kadarı
#   edge    : açıklık kenarlarında (fidan)
BIOMES = {
    "yaz_ormani": {"ground": ["forest", "forest_dark", "grass_dark", "dirt_brown"],
                   "pools": {"spruce": ("forest", 1.0), "sapling": ("edge", 1.0), "undergrowth": ("edge", 1.0)}},
    "kis_ormani": {"ground": ["snow", "snow_dirty"],
                   "pools": {"winter": ("forest", 1.0)}},
    "seyrek_orman": {"ground": ["moss", "grass_dark", "grass_dry", "forest"],
                     "pools": {"spruce": ("forest", 0.35), "undergrowth": ("scatter", 1.0), "rock": ("scatter", 0.4),
                               "shrub": ("scatter", 0.3)}},
    "col": {"ground": ["terrain_01", "sand_red", "sand_yellow"],
            "pools": {"palm": ("scatter", 1.0), "desert_tree": ("scatter", 1.0), "shrub": ("scatter", 1.0),
                      "rock": ("scatter", 0.5)}},
    "kayalik": {"ground": ["rock_gray", "dirt_brown", "grass_dry"],
                "pools": {"rock": ("scatter", 1.0), "shrub": ("scatter", 0.6), "spruce": ("forest", 0.2),
                          "undergrowth": ("scatter", 0.5)}},
}
N_CLEARINGS = 9
SCALE = {"rock": (0.8, 3.0), "shrub": (0.7, 1.4), "palm": (0.8, 1.2), "desert_tree": (0.8, 1.3),
         "undergrowth": (0.7, 1.4)}

_parked = set()   # bu süreçte zaten park edilmiş nesneler (tekrar RPC atma)


def random_clearings(scene, rng, landing):
    """İniş açıklığı + çakışmayan N_CLEARINGS rastgele açıklık [(x, y, r), ...]."""
    a = scene["area"]
    out = [(landing[0], landing[1], 12.0)]
    for _ in range(5000):
        if len(out) >= N_CLEARINGS + 1:
            break
        x, y, r = rng.uniform(a[0], a[1]), rng.uniform(a[2], a[3]), rng.uniform(9, 13)
        if all(math.hypot(x - cx, y - cy) > r + cr + 8 for cx, cy, cr in out):
            out.append((x, y, r))
    return out


def _in_clearing(x, y, clearings, pad=0.0):
    return any(math.hypot(x - cx, y - cy) < cr + pad for cx, cy, cr in clearings)


def _positions(mode, n, scene, clearings, rng):
    """Yerleşim moduna göre en fazla n nokta."""
    a, m = scene["area"], scene.get("forest_margin_m", 30)
    x0, x1, y0, y1 = a[0] - m, a[1] + m, a[2] - m, a[3] + m
    pts = []
    if mode == "forest":
        sp = scene.get("tree_spacing_m", 7.5) * rng.uniform(0.9, 1.15)   # her turda biraz farklı sıklık
        x = x0
        while x < x1:
            y = y0
            while y < y1:
                tx, ty = x + rng.uniform(-0.4, 0.4) * sp, y + rng.uniform(-0.4, 0.4) * sp
                if not _in_clearing(tx, ty, clearings, pad=rng.uniform(1.0, 4.0)):
                    pts.append((tx, ty))
                y += sp
            x += sp
        rng.shuffle(pts)
    elif mode == "scatter":
        for _ in range(n * 20):
            if len(pts) >= n:
                break
            tx, ty = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if not _in_clearing(tx, ty, clearings, pad=2.0):
                pts.append((tx, ty))
    elif mode == "edge":
        for _ in range(n * 20):
            if len(pts) >= n:
                break
            cx, cy, cr = clearings[1 + rng.integers(len(clearings) - 1)]
            ang, d = rng.uniform(0, 2 * math.pi), cr + rng.uniform(-2.5, 0.5)
            pts.append((cx + d * math.cos(ang), cy + d * math.sin(ang)))
    return pts[:n]


def _place(client, name, x, y, z, yaw, s):
    client.simSetObjectPose(name, airsim.Pose(airsim.Vector3r(x, y, z), euler_to_quaternion(0, 0, yaw)), True)
    client.simSetObjectScale(name, airsim.Vector3r(s, s, s))
    _parked.discard(name)


def _park(client, scene, name, k):
    if name in _parked:
        return
    px, py = scene.get("park", [3000.0, 3000.0])
    client.simSetObjectPose(name, airsim.Pose(airsim.Vector3r(px + k * 0.5, py, 50.0), airsim.Quaternionr()), True)
    client.simSetObjectScale(name, airsim.Vector3r(0.01, 0.01, 0.01))
    _parked.add(name)


def randomize_world(client, scene, rng, biome=None, landing=(0.0, 0.0), log=print):
    """Biyomu uygular, yeni açıklıkları döndürür (scene["clearings"] da güncellenir)."""
    pools = scene.get("pools")
    if not pools:   # eski harita: havuz yok, sadece açıklıklar sabit
        return scene["clearings"]
    biome = biome or list(BIOMES)[rng.integers(len(BIOMES))]
    spec = BIOMES[biome]
    clearings = random_clearings(scene, rng, landing)
    G = scene["ground_z_ned"]

    grounds = [g for g in spec["ground"] if g in scene.get("ground_materials", {})]
    ground = grounds[rng.integers(len(grounds))] if grounds else None
    if ground and scene.get("floor"):
        client.simSetObjectMaterial(scene["floor"], scene["ground_materials"][ground])

    used = 0
    for pool, names in pools.items():
        mode, frac = spec["pools"].get(pool, (None, 0.0))
        pts = _positions(mode, int(len(names) * frac), scene, clearings, rng) if mode else []
        lo, hi = SCALE.get(pool, (0.8, 1.2))
        for k, name in enumerate(names):
            if k < len(pts):
                _place(client, name, pts[k][0], pts[k][1], G, rng.uniform(-math.pi, math.pi), rng.uniform(lo, hi))
                used += 1
            else:
                _park(client, scene, name, k)

    # kişilerin kıyafet rengi kurulumda verilir (simSetObjectMaterial iskeletli mesh'te çalışmıyor)
    scene["clearings"] = [list(c) for c in clearings]
    scene["biome"] = biome
    log(f"  biyom: {biome}, zemin: {ground}, {used} bitki/kaya, {len(clearings)} açıklık")
    return clearings
