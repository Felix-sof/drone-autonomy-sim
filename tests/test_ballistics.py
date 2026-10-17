"""Düşüş hesabı ve rüzgar tahmini (simülatörsüz)."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mission"))
import ballistics as b  # noqa: E402


def test_no_wind_falls_straight_and_drag_slows_it():
    dn, de, t, v = b.simulate_fall(25.0)
    assert abs(dn) < 1e-9 and abs(de) < 1e-9
    t_vac = math.sqrt(2 * 25 / b.G)
    assert t > t_vac                                   # hava direnci düşüşü uzatır
    assert v[2] < math.sqrt(2 * b.G * 25)              # ve çarpma hızını düşürür


def test_wind_drifts_downwind_more_from_higher():
    d25 = b.simulate_fall(25.0, (0.0, 5.0))           # doğuya esen 5 m/s
    d50 = b.simulate_fall(50.0, (0.0, 5.0))
    assert d25[1] > 0.3 and abs(d25[0]) < 1e-9
    assert d50[1] > d25[1]


def test_release_offset_cancels_drift():
    off = b.release_offset(25.0, (3.0, -2.0))
    dn, de, _, _ = b.simulate_fall(25.0, (3.0, -2.0))
    assert abs(off[0] + dn) < 1e-9 and abs(off[1] + de) < 1e-9
    assert off[0] < 0 and off[1] > 0                  # rüzgarın geldiği yöne kaydır


def test_fit_cd_a_recovers_parameter():
    true = 0.2
    dn, de, _, _ = b.simulate_fall(25.0, (4.0, 0.0), cd_a=true, dt=0.01)
    assert b.fit_cd_a(25.0, (4.0, 0.0), math.hypot(dn, de)) == pytest.approx(true, rel=0.02)


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.0])
def test_wind_from_tilt_roundtrip(yaw):
    # rüzgar kuzeyden güneye 6 m/s (wn = -6): drone kuzeye (rüzgara) eğilir = burun aşağı (yaw 0'da)
    k = b.calibrate_k(math.radians(8), 6.0)
    tilt = math.radians(8)
    # itki yönü kuzey: gövdede (bx, by) = R(-yaw) * (1, 0)
    bx, by = math.cos(-yaw), math.sin(-yaw)
    pitch = -math.asin(bx * math.sin(tilt))
    roll = math.asin(by * math.sin(tilt) / math.cos(pitch))
    wn, we = b.wind_from_tilt(roll, pitch, yaw, k)
    assert wn == pytest.approx(-6.0, abs=0.3) and we == pytest.approx(0.0, abs=0.3)
