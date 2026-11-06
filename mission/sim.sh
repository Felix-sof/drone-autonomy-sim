#!/usr/bin/env bash
# Simülasyonu tek komutla başlat/durdur.
#   bash mission/sim.sh px4      # AirSim'i PX4 moduna al, Unreal'i aç + Play, PX4 SITL'i WSL'de başlat
#   bash mission/sim.sh airsim   # AirSim'in kendi uçuş kontrolcüsü (SimpleFlight)
#   bash mission/sim.sh video    # airsim + takip kamerası (record_video.py için; GPU'ya ek yük)
#   bash mission/sim.sh stop     # PX4'ü durdur, Unreal'i kapat
#   bash mission/sim.sh px4 setup   # açarken haritayı da yeniden kur (unreal/setup_forest.py)
# Gerekenler: WSL Ubuntu'da ~/PX4-Autopilot (v1.15.4, make px4_sitl_default), .wslconfig'te
# networkingMode=mirrored (PX4 ve AirSim aynı localhost'u görsün).
set -u
D="/c/Users/sanbe/OneDrive/Masaüstü/D"
A="/c/Users/sanbe/OneDrive/Belgeler/AirSim"
FLAGS=/c/UE/flags
UE='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe'
mode=${1:-px4}
setup=${2:-}

editor_up() { tasklist | grep -qi "UnrealEditor.exe"; }
# netstat (WSL mirrored ağ modunda) AirSim'in dinlediği portu artık göstermiyor; PowerShell görüyor.
# Port gerçekten kabul ediyor mu diye bağlanmayı denemek en güvenlisi.
port_up() { powershell -NoProfile -Command "\$c = New-Object Net.Sockets.TcpClient; try { \$c.Connect('127.0.0.1', $1); exit 0 } catch { exit 1 } finally { \$c.Close() }" >/dev/null 2>&1; }
# 4560'a deneme bağlantısı AÇMA: AirSim tek bağlantı kabul eder, deneme onu PX4 sanar. Kayda bak:
# son Play'den sonra "Waiting for TCP connection on port 4560" yazdıysa AirSim PX4'ü bekliyor.
airsim_waits_px4() { sed -n '/auto_editor: Play/,$p' /c/UE/Sa/Saved/Logs/Sa.log 2>/dev/null | grep -q "Waiting for TCP connection on port 4560"; }
px4_stop() { wsl.exe -d Ubuntu -u sanberk -- bash -lc 'pkill -f "bin/px4" 2>/dev/null; true'; }
editor_stop() {
  editor_up || return 0
  touch "$FLAGS/quit"
  for _ in $(seq 1 60); do editor_up || break; sleep 2; done
  editor_up && taskkill //IM UnrealEditor.exe //F >/dev/null 2>&1
  rm -f "$FLAGS/quit"
}
px4_start() {
  # Bağımsız gizli bir wsl.exe süreci: başlatan oturum kapanınca PX4 de kapanmasın
  # (wsl.exe ... '... &' ile başlatınca oturum bitince "PX4 Exiting..." oluyordu)
  wsl.exe -d Ubuntu -u sanberk -- bash -lc 'rm -f /tmp/px4.log'
  powershell -NoProfile -Command "Start-Process -WindowStyle Hidden -FilePath wsl.exe -ArgumentList '-d Ubuntu -u sanberk -- bash -lc \"cd ~/PX4-Autopilot && PX4_SIM_MODEL=none_iris PX4_SIM_HOST_ADDR=127.0.0.1 ./build/px4_sitl_default/bin/px4 -d > /tmp/px4.log 2>&1\"'"
}
editor_start() {   # auto modunda aç: init_unreal.py Play'i kendisi başlatır
  mkdir -p "$FLAGS"; rm -f "$FLAGS/quit" "$FLAGS/play"; touch "$FLAGS/auto"
  powershell -NoProfile -Command "Start-Process -FilePath '$UE' -ArgumentList '\"C:\UE\Sa\Sa.uproject\"'"
}

case "$mode" in
  stop)
    px4_stop; editor_stop; rm -f "$FLAGS/auto"; echo "durduruldu" ;;
  airsim|px4|video)
    px4_stop; editor_stop
    if [ "$mode" = px4 ]; then cp "$D/airsim/settings_px4.json" "$A/settings.json"
    elif [ "$mode" = video ]; then cp "$D/airsim/settings_video.json" "$A/settings.json"   # + takip kamerası
    else cp "$D/airsim/settings.json" "$A/settings.json"; fi
    [ "$setup" = setup ] && touch "$FLAGS/setup"
    editor_start
    if [ "$mode" = px4 ]; then
      for _ in $(seq 1 300); do airsim_waits_px4 && break; sleep 2; done   # AirSim PX4'ü bekliyor
      airsim_waits_px4 || { echo "AirSim 4560'ta beklemiyor"; exit 1; }
      px4_start
      for _ in $(seq 1 60); do
        wsl.exe -d Ubuntu -u sanberk -- bash -lc 'grep -q "Ready for takeoff" /tmp/px4.log' && { echo "PX4 hazır (MAVSDK: udp://:14550)"; break; }
        sleep 2
      done
    fi
    for _ in $(seq 1 300); do port_up 41451 && break; sleep 2; done
    rm -f "$FLAGS/auto"   # sonraki elle açılışlar normal olsun
    echo "simülasyon hazır ($mode)" ;;
  *) echo "kullanım: bash mission/sim.sh [px4|airsim|stop]"; exit 1 ;;
esac
