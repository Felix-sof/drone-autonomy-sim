"""SAR yardımcıları testleri (simülatör gerekmez): python -m pytest tests -q"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mission"))
import perception  # noqa: E402
from perception import pixel_to_ground  # noqa: E402
from sar import ground_to_pixel, openness  # noqa: E402

SHAPE = (720, 1280, 3)


@pytest.fixture(autouse=True)
def gimbal():
    old = perception.CAMERA_STABILIZED, perception.CAMERA_YAW_LOCK
    perception.CAMERA_STABILIZED, perception.CAMERA_YAW_LOCK = True, 0.0
    yield
    perception.CAMERA_STABILIZED, perception.CAMERA_YAW_LOCK = old


@pytest.mark.parametrize("dn,de", [(0, 0), (5, 0), (0, -7), (3.5, 4.2), (-6, -2)])
def test_ground_pixel_roundtrip(dn, de):
    px, py = ground_to_pixel(SHAPE, dn, de, 25.0)
    # drone eğik olsa da gimbal'lı kamerada roll/pitch yok sayılmalı; yaw kilitli (kuzey)
    n, e = pixel_to_ground(SHAPE, px, py, 25.0, math.radians(8), math.radians(-5), math.radians(30))
    assert n == pytest.approx(dn, abs=1e-6) and e == pytest.approx(de, abs=1e-6)


def test_north_is_up():
    px, py = ground_to_pixel(SHAPE, 5, 0, 25.0)
    assert py < 360 and px == pytest.approx(640)


def test_openness_ground_vs_tree():
    rng = np.random.default_rng(0)
    ground = np.full((200, 200, 3), (60, 110, 160), np.uint8)       # düz kahverengi toprak (BGR)
    tree = rng.integers(0, 120, (200, 200, 3), dtype=np.uint8)       # karanlık, dokulu
    assert openness(ground, 100, 100, 40) > 0.8
    assert openness(tree, 100, 100, 40) < 0.3


def test_openness_outside_image_is_zero():
    img = np.full((100, 100, 3), 128, np.uint8)
    assert openness(img, -500, -500, 10) == 0.0
