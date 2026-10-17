"""Algı geometrisi testleri (simülatör gerekmez): python -m pytest tests -q"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mission"))
from perception import pixel_to_ground  # noqa: E402

SHAPE = (720, 1280, 3)   # h, w
H = 25.0                 # yükseklik (m)
F = 640.0                # 90° yatay görüş, 1280 genişlik -> odak 640 px


def test_center_is_below():
    dn, de = pixel_to_ground(SHAPE, 640, 360, H, 0, 0, 0)
    assert abs(dn) < 1e-9 and abs(de) < 1e-9


def test_axes_level_yaw0():
    # görüntünün üstü = ileri = kuzey; sağı = doğu
    dn, de = pixel_to_ground(SHAPE, 640, 360 - 64, H, 0, 0, 0)   # 64 px yukarı
    assert dn == pytest.approx(H * 64 / F) and abs(de) < 1e-9
    dn, de = pixel_to_ground(SHAPE, 640 + 64, 360, H, 0, 0, 0)   # 64 px sağa
    assert de == pytest.approx(H * 64 / F) and abs(dn) < 1e-9


def test_yaw_rotates_offset():
    # burun doğuya (yaw 90°): görüntünün üstü artık doğu
    dn, de = pixel_to_ground(SHAPE, 640, 360 - 64, H, 0, 0, math.pi / 2)
    assert de == pytest.approx(H * 64 / F) and abs(dn) < 1e-6


def test_pitch_compensation():
    # burun 10° aşağı (pitch -10°): kamera geriye bakar; görüntü merkezi drone'un arkasındaki
    # yeri gösterir -> merkez piksel yerde H*tan(10°) geride olmalı
    dn, de = pixel_to_ground(SHAPE, 640, 360, H, 0, math.radians(-10), 0)
    assert dn == pytest.approx(-H * math.tan(math.radians(10)), rel=1e-6) and abs(de) < 1e-9


def test_roll_compensation():
    # sağa 10° yatış (roll +10°): kamera sola bakar -> merkez piksel yerde solda (batıda)
    dn, de = pixel_to_ground(SHAPE, 640, 360, H, math.radians(10), 0, 0)
    assert de == pytest.approx(-H * math.tan(math.radians(10)), rel=1e-6) and abs(dn) < 1e-9


def test_horizon_returns_none():
    # burun 89° aşağıda: kamera neredeyse yatay geriye bakar; görüntünün ALT kenarı
    # (gövdenin arkası) ufkun üstüne bakar -> yerde kesişim yok
    assert pixel_to_ground(SHAPE, 640, 719, H, 0, math.radians(-89), 0) is None


def test_range_to_agl():
    import math as _m
    from vehicle import range_to_agl
    assert abs(range_to_agl(5.0, 0.2, 12.0, 0.0, 0.0) - 5.0) < 1e-9
    assert abs(range_to_agl(5.0, 0.2, 12.0, _m.radians(10), 0.0) - 5.0 * _m.cos(_m.radians(10))) < 1e-9
    assert range_to_agl(15.0, 0.2, 12.0, 0, 0) is None        # menzil dışı
    assert range_to_agl(float("nan"), 0.2, 12.0, 0, 0) is None
    assert range_to_agl(5.0, 0.2, 12.0, _m.radians(40), 0) is None   # çok eğik
