"""Pilot onayı: sessizlik = bırakma yok; eski satırlar onay sayılmaz."""
import builtins
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "mission"))
import fly_mission  # noqa: E402
import run_real  # noqa: E402

IMG = np.zeros((10, 10, 3), np.uint8)


def scripted_input(monkeypatch, lines):
    """input() sırayla (bekleme_s, satır) döndürür, sonra sonsuza kadar bekler."""
    it = iter(lines)
    block = threading.Event()

    def fake_input(*_):
        try:
            wait, line = next(it)
        except StopIteration:
            block.wait()
        time.sleep(wait)
        return line
    monkeypatch.setattr(builtins, "input", fake_input)
    monkeypatch.setattr(run_real, "_stdin_q", None)
    monkeypatch.setattr(fly_mission, "SHOW_PREVIEW", False)


def test_silence_means_no(monkeypatch):
    scripted_input(monkeypatch, [])
    assert run_real.console_approve(IMG, "test", timeout=0.2) is False


def test_yes_approves(monkeypatch):
    scripted_input(monkeypatch, [(0.1, "e")])
    assert run_real.console_approve(IMG, "test", timeout=2) is True


def test_stale_line_is_not_approval(monkeypatch):
    scripted_input(monkeypatch, [(0.0, "e")])   # soru gelmeden yazılmış
    run_real._stdin_q = None
    time.sleep(0.05)
    assert run_real.console_approve(IMG, "ilk", timeout=0.3) in (True, False)  # okuyucu başlar
    assert run_real.console_approve(IMG, "ikinci", timeout=0.2) is False   # eski 'e' tekrar sayılmaz
