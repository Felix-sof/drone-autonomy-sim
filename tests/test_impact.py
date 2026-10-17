"""Paket çarpma ölçümü: Unreal'in aynı fizik karesini tekrar döndürmesi 'çarptı' sanılmamalı."""
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mission"))
import drop_test  # noqa: E402
import fly_mission  # noqa: E402


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def time(self):
        return self.t

    def sleep(self, s):
        self.t += s


class FallingClient:
    """h m'den serbest düşen paket; konum ~45 Hz güncellenir (50 Hz okumada tekrarlar olur)."""

    def __init__(self, clock, h, ground=0.9):
        self.clock, self.z0, self.ground = clock, ground - h, ground

    def simGetObjectPose(self, name):
        t = int(self.clock.t * 45) / 45                      # fizik karesine yuvarla
        z = min(self.z0 + 0.5 * 9.81 * t * t, self.ground)
        return SimpleNamespace(position=SimpleNamespace(x_val=1.0, y_val=2.0, z_val=z))


@pytest.mark.parametrize("h", [5.0, 25.0])
def test_impact_point_and_speed_with_repeated_frames(monkeypatch, h):
    clock = FakeClock()
    monkeypatch.setattr(time, "time", clock.time)
    monkeypatch.setattr(time, "sleep", clock.sleep)
    monkeypatch.setattr(fly_mission, "SHOW_PREVIEW", False)
    rest, impact = drop_test.wait_until_rest(FallingClient(clock, h), "pkg", None)
    assert impact[2] > 0.9 - 0.6                              # yere değdiği an, havada değil
    v_true = (2 * 9.81 * h) ** 0.5
    assert abs(drop_test.LAST_IMPACT_SPEED - v_true) < 0.15 * v_true
