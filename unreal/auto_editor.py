"""Editör açılışında çalışır (-ExecutePythonScript): insansız otomasyon için.

- C:/UE/flags/setup varsa önce setup_forest.py'yi çalıştırır (dosyayı siler)
- Sonra Play'i (PIE) başlatır
- Arka planda bayrakları izler:
    C:/UE/flags/play  -> Play kapalıysa yeniden başlat
    C:/UE/flags/quit  -> Play'i durdur ve editörü düzgünce kapat
Bayrak dosyalarını dışarıdaki orkestratör (mission/overnight.sh) oluşturur.

Başlatma:
  UnrealEditor.exe C:/UE/Sa/Sa.uproject -ExecutePythonScript="C:/UE/auto_editor.py"
"""
import runpy
import time
from pathlib import Path

import unreal

FLAGS = Path("C:/UE/flags")
SETUP = Path(r"C:\Users\sanbe\OneDrive\Masaüstü\D\unreal\setup_forest.py")
STARTUP_DELAY_S = 25  # harita ve shader'lar yerleşsin

state = {"t0": time.time(), "started": False, "last_check": 0.0}


def les():
    return unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)


def take(flag):
    p = FLAGS / flag
    if p.exists():
        p.unlink()
        return True
    return False


def begin_play():
    unreal.log("auto_editor: Play başlatılıyor")
    les().editor_request_begin_play()


def tick(_dt):
    now = time.time()
    if not state["started"]:
        if now - state["t0"] < STARTUP_DELAY_S:
            return
        state["started"] = True
        if take("setup"):
            unreal.log("auto_editor: setup_forest çalışıyor")
            runpy.run_path(str(SETUP), run_name="__main__")
        begin_play()
        return
    if now - state["last_check"] < 1.0:
        return
    state["last_check"] = now
    if take("quit"):
        unreal.log("auto_editor: kapanıyor")
        if les().is_in_play_in_editor():
            les().editor_request_end_play()
        unreal.SystemLibrary.quit_editor()
    elif take("play") and not les().is_in_play_in_editor():
        begin_play()


FLAGS.mkdir(parents=True, exist_ok=True)
_handle = unreal.register_slate_post_tick_callback(tick)
unreal.log("auto_editor: hazır")
