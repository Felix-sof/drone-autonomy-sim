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


def test_wind_matrix_fit_recovers_anisotropy():
    # drone kuzey-güney yönünde daha az eğiliyor + 8° dönüklük: matris bunu öğrenmeli
    true_M = [[1.25 * math.cos(0.14), -1.25 * math.sin(0.14)], [0.9 * math.sin(0.14), 0.9 * math.cos(0.14)]]
    import numpy as np
    Minv = np.linalg.inv(np.array(true_M))
    samples = []
    for ang in range(0, 360, 45):
        for v in (5.0, 8.0):
            w = np.array([v * math.cos(math.radians(ang)), v * math.sin(math.radians(ang))])
            samples.append((tuple(Minv @ w), tuple(w)))
    M = b.fit_wind_matrix(samples)
    assert np.allclose(M, true_M, atol=1e-6)


def _attitude_for(wn, we, yaw, k_of, d_of):
    """Sahte drone: rüzgar (wn, we) için k(θ) ve yön sapması d(θ) olan eğim üretir (gövde açısı θ)."""
    v = math.hypot(wn, we)
    th_true = math.atan2(we, wn) - yaw
    th_tilt = th_true - d_of(th_true)
    tilt = math.atan(k_of(th_true) * v * v)
    # itki yönü rüzgarın tersi (gövdede): (bx, by) = -(cos, sin) th_tilt
    bx, by = -math.cos(th_tilt), -math.sin(th_tilt)
    pitch = -math.asin(bx * math.sin(tilt))
    roll = math.asin(by * math.sin(tilt) / math.cos(pitch))
    return roll, pitch, yaw


def test_harmonic_wind_calibration_recovers_x_frame_response():
    k_of = lambda t: 0.00062 + 0.00010 * math.cos(4 * t)          # eksenlerde fazla, çaprazda az eğilir
    d_of = lambda t: 0.12 * math.sin(4 * t)                       # çaprazda ~7° yön sapması
    rows = []
    for s in (5.0, 8.0):
        for a in range(0, 360, 30):          # 45°'nin katları sin4θ'yı hep sıfır yapar: 30°'de bir
            wn, we = s * math.cos(math.radians(a)), s * math.sin(math.radians(a))
            rows.append((*_attitude_for(wn, we, 0.0, k_of, d_of), wn, we))
    prm = b.fit_wind_harmonic(rows)
    for wn, we in [(-3.0, 4.0), (5.0, 0.0), (2.0, -6.0), (-4.5, -4.5)]:   # kalibrasyonda olmayan rüzgarlar
        en, ee = b.wind_from_tilt_harmonic(*_attitude_for(wn, we, 0.0, k_of, d_of), prm)
        assert math.hypot(en - wn, ee - we) < 0.3


def test_lut_wind_calibration_handles_sharp_pattern():
    # keskin, 180° periyotlu sapma (gerçek kalibrasyondaki gibi 60°'de -14°, 90°'de +6°); yaw -7°
    def d_of(t):                        # gerçek açıya bağlı sapma (radyan), gövde çerçevesi
        deg = math.degrees(t) % 180
        return math.radians(-14.0 if 50 <= deg <= 70 else (6.0 if deg < 50 or deg > 140 else -12.0))
    k_of = lambda t: 0.0006 + 0.0001 * math.cos(4 * t)
    yaw = math.radians(-7.3)
    rows = []
    for a in range(0, 360, 10):
        wn, we = 5 * math.cos(math.radians(a)), 5 * math.sin(math.radians(a))
        rows.append((*_attitude_for(wn, we, yaw, k_of, d_of), wn, we))
    lut = b.fit_wind_lut(rows)
    errs = []
    for a in range(5, 360, 10):          # ölçüm noktalarının tam ortası: en zor durum
        wn, we = 5.5 * math.cos(math.radians(a)), 5.5 * math.sin(math.radians(a))
        en, ee = b.wind_from_tilt_lut(*_attitude_for(wn, we, yaw, k_of, d_of), lut)
        errs.append(math.hypot(en - wn, ee - we))
    assert sorted(errs)[len(errs) // 2] < 0.3            # tipik hata küçük
