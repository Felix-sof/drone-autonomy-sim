"""Görev mantığı testleri — simülatörsüz, sahte drone + sahte kamera/tespit ile kapalı döngü.

python -m pytest tests -q
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mission"))
import autonomy  # noqa: E402
import fly_mission  # noqa: E402
import perception  # noqa: E402
import sar  # noqa: E402

SHAPE = (720, 1280, 3)
F = 640.0  # 90° yatay görüş, 1280 px


@pytest.fixture(autouse=True)
def gimbal(monkeypatch):
    monkeypatch.setattr(perception, "CAMERA_STABILIZED", True)
    monkeypatch.setattr(perception, "CAMERA_YAW_LOCK", 0.0)
    monkeypatch.setattr(fly_mission, "SHOW_PREVIEW", False)
    monkeypatch.setattr(time, "sleep", lambda s: None)  # testler hızlı koşsun


class FakeVehicle:
    """Hız komutunu birinci dereceden gecikmeyle izleyen nokta-kütle drone; kamera her karede dt ilerletir."""

    def __init__(self, n=0.0, e=0.0, alt=25.0, battery=0.9):
        self.p = np.array([n, e, -alt], dtype=float)
        self.v = np.zeros(3)
        self.cmd = np.zeros(3)
        self.battery_level = battery

    def step(self, dt=0.12, tau=0.3):
        self.v += (self.cmd - self.v) * min(1.0, dt / tau)
        self.p += self.v * dt

    # arayüz
    def velocity_ned(self, vn, ve, vd):
        self.cmd = np.array([vn, ve, vd])

    def hover(self):
        self.cmd = np.zeros(3)

    def position(self):
        return tuple(self.p)

    def velocity(self):
        return tuple(self.v)

    def attitude(self):
        return (0.0, 0.0, 0.0)

    def height_agl(self):
        return -self.p[2]

    def battery(self):
        return self.battery_level

    def wait_still(self, max_speed=0.2, timeout=10):
        self.hover()
        for _ in range(100):
            self.step()
        return True

    def distance_to(self, n, e, d=None):
        return math.hypot(self.p[0] - n, self.p[1] - e)


class FakeWorld:
    """Yerde sabit nesneler; grab() drone'u bir adım ilerletir ve kareyi döndürür."""

    def __init__(self, vehicle, objects):
        self.v, self.objects = vehicle, objects  # [(n, e, sınıf), ...]
        self.frames = 0

    def grab(self):
        self.v.step()
        self.frames += 1
        return np.zeros(SHAPE, np.uint8)

    def detect(self, img, conf=None, classes=None):
        h = self.v.height_agl()
        boxes = []
        for n, e, cls in self.objects:
            if classes is not None and cls not in classes:
                continue
            dn, de = n - self.v.p[0], e - self.v.p[1]
            px, py = 640 + de / h * F, 360 - dn / h * F
            if 0 <= px < 1280 and 0 <= py < 720:
                boxes.append((px - 20, py - 10, px + 20, py + 10, 0.9, cls))
        return boxes


def test_lawnmower_covers_area():
    pts = autonomy.lawnmower((-45, 45, -35, 55), 35, 25)
    assert len(pts) % 2 == 0 and all(p[2] == -25 for p in pts)
    lanes = sorted({p[1] for p in pts})
    assert lanes[0] - (-35) <= 35 and 55 - lanes[-1] <= 35          # kenarlar kapsanıyor
    assert all(b - a <= 35 + 1e-9 for a, b in zip(lanes, lanes[1:]))


@pytest.mark.parametrize("pos,battery,elapsed,reason", [
    ((0, 0), 0.9, 0, None),
    ((80, 0), 0.9, 0, "geofence"),
    ((0, 0), 0.10, 0, "pil"),
    ((0, 0), 0.9, 10_000, "süre"),
])
def test_safety_check(pos, battery, elapsed, reason):
    v = FakeVehicle(*pos, battery=battery)
    area = (-45, 45, -35, 55)
    if reason is None:
        autonomy.safety_check(v, area, time.time() - elapsed)
    else:
        with pytest.raises(autonomy.Abort, match=reason):
            autonomy.safety_check(v, area, time.time() - elapsed)


def test_align_converges_over_car():
    v = FakeVehicle(0, 0)
    world = FakeWorld(v, [(4.0, -3.0, 0)])
    assert autonomy.align_over_target(v, world.grab, world, timeout=30)
    assert math.hypot(v.p[0] - 4.0, v.p[1] + 3.0) < autonomy.ALIGN_TOL_M + 0.1


def test_align_does_not_jump_to_other_car():
    # yanda (12 m) ikinci bir araba var; kilitlenilen arabada kalmalı
    v = FakeVehicle(0, 0)
    world = FakeWorld(v, [(2.0, 1.0, 0), (2.0, 13.0, 0)])
    assert autonomy.align_over_target(v, world.grab, world, timeout=30)
    assert math.hypot(v.p[0] - 2.0, v.p[1] - 1.0) < 0.5


def test_align_gives_up_when_nothing_seen():
    v = FakeVehicle(0, 0)
    world = FakeWorld(v, [])
    assert autonomy.align_over_target(v, world.grab, world, timeout=30) is False


def test_sar_drop_point_keeps_distance_from_people():
    v = FakeVehicle(0, 0)
    # hizalanılan kişi drone'un altında, bir başkası 7 m kuzeyde, bir araba 6 m doğuda
    world = FakeWorld(v, [(0.0, 0.0, 1), (7.0, 0.0, 1), (0.0, 6.0, 0)])
    img = np.full(SHAPE, (60, 110, 160), np.uint8)   # düz toprak: her yer açık
    dn, de, score = sar.choose_drop_point(v, img, world, (0.0, 0.0))
    for n, e, cls in world.objects:
        d = math.hypot(dn - n, de - e)
        assert d >= (sar.SAFE_MIN_M if cls == 1 else sar.OTHERS_MIN_M)


def test_sar_zone_clear_detects_person_below():
    v = FakeVehicle(0, 0)
    assert sar.zone_clear(v, np.zeros(SHAPE, np.uint8), FakeWorld(v, [(10.0, 0.0, 1)]))
    assert not sar.zone_clear(v, np.zeros(SHAPE, np.uint8), FakeWorld(v, [(2.0, 1.0, 1)]))


def fake_marker(monkeypatch, v, n, e, marker_id=3):
    """find_markers'ı yerdeki (n, e) işaretçisini drone'dan görüyormuş gibi taklit eder."""
    def find(img, exclude=(0,)):
        h = v.height_agl()
        dn, de = n - v.p[0], e - v.p[1]
        return [(marker_id, (640 + de / h * F, 360 - dn / h * F), None)]
    monkeypatch.setattr(autonomy, "find_markers", find)


def test_descend_over_marker_reaches_drop_alt_centered(monkeypatch):
    v = FakeVehicle(0, 0, alt=25)
    fake_marker(monkeypatch, v, 1.0, -0.8)
    assert autonomy.descend_over_marker(v, FakeWorld(v, []).grab, 3, 5.0) is None
    assert abs(v.height_agl() - 5.0) < 0.3
    assert math.hypot(v.p[0] - 1.0, v.p[1] + 0.8) < autonomy.ALIGN_TOL_M + 0.1


def test_descend_over_marker_aborts_if_person_near(monkeypatch):
    v = FakeVehicle(0, 0, alt=25)
    fake_marker(monkeypatch, v, 0.0, 0.0)
    world = FakeWorld(v, [(2.0, 1.0, 1)])          # işaretçiye 2.2 m'de bir kişi
    why = autonomy.descend_over_marker(v, world.grab, 3, 5.0, detector=world)
    assert why is not None and "kişi" in why
    assert v.height_agl() > 20                     # hiç alçalmadan vazgeçti


def test_sar_descend_clear(monkeypatch):
    v = FakeVehicle(0, 0, alt=25)
    assert sar.descend_clear(v, FakeWorld(v, [(7.0, 0.0, 1)]).grab, FakeWorld(v, [(7.0, 0.0, 1)]), 8.0)
    assert abs(v.height_agl() - 8.0) < 0.3 and math.hypot(v.p[0], v.p[1]) < 0.3
    v2 = FakeVehicle(0, 0, alt=25)
    w2 = FakeWorld(v2, [(1.0, 1.0, 1)])            # tam altta kişi
    assert not sar.descend_clear(v2, w2.grab, w2, 8.0)
