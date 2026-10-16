#!/usr/bin/env bash
# Gece otomasyonu: veri topla -> Unreal'i kapat -> eğit -> Unreal'i aç -> modelle paket bırakma testi.
# Unreal'i C:/UE/flags üzerinden yönetir (unreal/auto_editor.py + Content/Python/init_unreal.py).
# Kullanım: bash mission/overnight.sh [tur_sayisi] [deneme_sayisi] [model_adi] [epoch]
set -u
ROUNDS=${1:-20}
TRIALS=${2:-20}
D="/c/Users/sanbe/OneDrive/Masaüstü/D"
FLAGS=/c/UE/flags
UE='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe'
RUN=${3:-car_forest2}
EPOCHS=${4:-60}
MODEL="C:/UE/drone_runs/$RUN/weights/best.pt"
export PYTHONIOENCODING=utf-8
cd "$D"

log() { echo "[$(date +%H:%M:%S)] $*"; }
play_up() { netstat -ano | grep ":41451" | grep LISTEN | grep -q .; }
editor_up() { tasklist | grep -qi "UnrealEditor.exe"; }
wait_play() {  # Play'in kararlı açılmasını bekle (en fazla ~15 dk)
  local ok=0
  for _ in $(seq 1 450); do
    if play_up; then ok=$((ok + 1)); else ok=0; fi
    [ $ok -ge 3 ] && return 0
    sleep 2
  done
  return 1
}
start_editor() {
  rm -f "$FLAGS/quit" "$FLAGS/play"   # önceki adımdan kalan bayrak yeni editörü hemen kapatmasın
  touch "$FLAGS/auto"
  powershell -NoProfile -Command "Start-Process -FilePath '$UE' -ArgumentList '\"C:\UE\Sa\Sa.uproject\"'"
}
ensure_play() {  # editör kapalıysa aç, Play kapalıysa başlat
  play_up && return 0
  if editor_up; then touch "$FLAGS/play"; else start_editor; fi
  wait_play
}
quit_editor() {
  editor_up || return 0                # kapalıyken bayrak bırakma (sonraki açılışı kapatır)
  touch "$FLAGS/quit"
  for _ in $(seq 1 90); do editor_up || return 0; sleep 2; done
  log "editör kapanmadı, zorla kapatılıyor"
  taskkill //IM UnrealEditor.exe //F >/dev/null 2>&1
  sleep 5
}
count_rounds() { ls -d dataset/raw/*/ 2>/dev/null | wc -l; }

# --- 1) Veri toplama (Play düşerse kaldığı yerden devam) ---
mkdir -p "$FLAGS" dataset/raw
start_count=$(count_rounds)
for attempt in 1 2 3 4; do
  done_rounds=$(( $(count_rounds) - start_count ))
  left=$((ROUNDS - done_rounds))
  [ $left -le 0 ] && break
  log "veri toplama: $left tur kaldı (deneme $attempt)"
  ensure_play || { log "Play açılamadı"; continue; }
  timeout 7200 python -u mission/collect_data.py --turlar $left 2>&1 \
    | grep --line-buffered -E "Tur [0-9]+:|Duruyorum|Toplam|Error|Traceback"
done
log "veri toplama bitti: $(( $(count_rounds) - start_count )) tur"

# --- 2) Eğitim (Unreal kapalıyken) ---
quit_editor
python training/prepare_dataset.py
log "eğitim başlıyor"
python -u training/train.py --model yolo11s.pt --epochs $EPOCHS --name $RUN > /c/UE/train_${RUN}_log.txt 2>&1
tr '\r' '\n' < "/c/UE/train_${RUN}_log.txt" | grep -E "^\s+all " | tail -1
[ -f "$MODEL" ] || { log "model oluşmadı, duruyorum"; exit 1; }
log "eğitim bitti"

# --- 3) Modelle paket bırakma testi ---
start_editor
wait_play || { log "Play açılamadı"; exit 1; }
timeout 7200 python -u mission/drop_test.py --model "$MODEL" --deneme "$TRIALS" --irtifa 25 --gizle 2>&1 \
  | grep --line-buffered -E "İSABET|görülmedi|SONUÇ|İsabet|Ek:|İlk temas|Deneme başına|Kaydedildi|Error"
# --- 4) Rotasız otonom görev (2 görev, her biri 3 teslimat) ---
for k in 1 2; do
  ensure_play || break
  timeout 1800 python -u mission/autonomous_mission.py --model "$MODEL" --teslimat 3 --irtifa 25 2>&1     | grep --line-buffered -E "Araba bulundu|paket|kayboldu|OTONOM|Teslimat|Toplam süre|Kaydedildi|Error|Traceback"
done
log "gece akışı bitti"
