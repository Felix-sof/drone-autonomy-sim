# Otonom Drone Simülasyonu: Tespit + Paket Bırakma

Unreal Engine 5.8 + [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) + **PX4** üzerinde döner kanatlı
bir drone: alanı kendisi tarar, alt kamerasıyla arabaları tespit eder, hedefin üstüne hizalanıp
kargo paketi bırakır ve **ArUco işaretçili helipad'e sadece kamerayla** iner.

Görev kodu simülatörden bağımsız yazıldı (araç / kamera / paket arayüzleri): aynı `autonomy.py`
AirSim'in uçuş kontrolcüsüyle, PX4 SITL ile ve — arka uçları değiştirerek — gerçek drone'da
çalışır. Gerçek drone'a geçiş: [docs/GERCEK_DRONE.md](docs/GERCEK_DRONE.md).

```
            ┌──────────────── autonomy.py (ara/tespit/hizala/bırak/in, güvenlik) ───────────────┐
 vehicle.py │ AirSimVehicle (SimpleFlight)  │  Px4Vehicle ── MAVSDK/MAVLink ──> PX4 (SITL / Pixhawk) │
 kamera     │ AirSim simGetImages           │  camera.OpenCVCamera (USB / CSI / RTSP)               │
 paket      │ payload.SimPayload (fizik)    │  payload.GripperPayload (PX4 gripper)                 │
            └────────────────────────────────────────────────────────────────────────────────────┘
```

## Sonuçlar (şimdiye kadar)

| Deney | Sonuç |
|---|---|
| **Rotasız tam otonom görev** (ara → YOLO ile bul → hizalan → bırak → in), 3 görev × 3 paket | **9/9 isabet, 9 farklı araba, 0 yanlış alarm**, görev başına ~6 dk |
| Paket bırakma, **hedefi YOLO buluyor**, 25 m, 20 deneme | **14/20 hedef araba**, 2 yan araba, 0 ağaç/yer, 4 göremedi; deneme başına 56 s |
| Paket bırakma, kusursuz (Unreal) tespit, **10 m**, 10 deneme | **10/10**, ilk temas merkeze ort. 3 cm, 64 s/deneme |
| Paket bırakma, kusursuz (Unreal) tespit, **25 m**, 10 deneme | **10/10**, ilk temas merkeze ort. 3 cm, **49 s/deneme** (%23 daha hızlı) |
| YOLO11s araba tespiti, orman, 2237 görüntü / 50 tur | **mAP50 0.84**, precision 0.88, recall 0.75 — doğrulama **hiç görülmemiş 8 turda** |
| **Görüntüyle hassas iniş** (ArUco, eğim telafili, simülatör bilgisi yok), 25 m, 2-6 m kaydırarak | iniş hatası **0-1 cm** |
| Aynı modelin **gerçek** drone görüntülerinde (VisDrone val, 548 görüntü, 16k araç) başarısı | mAP50 **0.02** — sadece simülasyonla eğitilen model gerçekte çalışmıyor → karma eğitim |
| **PX4 SITL ile tam otonom görev** (MAVLink/MAVSDK, gimbal'lı kamera, karma model), 4 görev × 3 paket | **12/12 isabet, 12 farklı araba, 0 yanlış alarm**; ArUco iniş hatası ort. **7 cm**; görev ~2.5 dk |
| PX4 tam görev **rüzgarda** (5 m/s ×2, 8 m/s ×1) | **9/9 isabet**, ArUco iniş 3-10 cm — *not: AirSim rüzgarı drone'u etkiliyor, düşen paketi etkilemiyor; gerçekte paket sürüklenir* |
| Karma model (simülasyon + VisDrone) gerçek drone görüntülerinde | mAP50 **0.86** (simülasyonda 0.82) |

İsabet = paketin **ilk temas** noktası hedef arabanın üstten görünen ayak izinde mi
(başka araba / ağaç / yer ayrı sayılır). Kavisli tavandan sekme ayrıca ölçülür.

### Öğrenilenler
- **Kare bazında train/val bölmek yalan söyler:** art arda kareler neredeyse aynı; ilk orman
  modeli val'de 0.81 gösterip canlı sahnede ortadaki arabayı göremedi. Tur bazında bölününce
  gerçek başarı ölçülebildi; daha çok sahne + bırakma açısından (drone durgun, araba merkezde)
  çekimlerle yeniden eğitilen model canlıda çalıştı.
- **Hizalama takip etmeli:** "görüntü merkezine en yakın kutu" seçimi, hedef bir kare kaybolunca
  yandaki arabaya kayıyor (bir görevde 3 paket aynı arabaya gitti). Önceki karedeki konuma en
  yakın kutuyu seçip sıçramayı sınırlamak + birkaç kare kayıp toleransı bunu çözdü.
- **Kamera eğimi telafi edilmeli:** 2 m/s kayan drone ~10° eğilir; 25 m'de bu 4-5 m sahte sapma.
  Telafisiz ArUco inişinde drone işaretçinin etrafında salınıp uzaklaştı; ışını roll/pitch ile
  yataya döndürüp NED hız komutu vermek 0-1 cm iniş verdi.
- **Simülasyon ≠ gerçek:** simülasyonda 0.84 olan model gerçek görüntüde 0.02. Gerçek veri şart:
  VisDrone ile karma eğitim gerçekte 0.86'ya çıkardı, simülasyondaki başarıyı koruyarak.
- **Gövdeye sabit kamera + MAVLink telemetrisi = senkron sorunu:** PX4'te yönelim ile kamera karesi
  farklı anlarda geliyor; eğim telafisi 25 m'de ±5 m sahte hata üretti, drone salındı. Gimbal'lı
  kamera (pitch -89.9° — tam -90° gimbal kilidi yapıyor —, yaw kuzeye kilitli) ölçümü ±0.2 m'ye
  indirdi ve PX4 görevleri 12/12 oldu.
- **Kalan darboğaz tespit:** bırakılan her paket bir arabaya isabet ediyor; kayıplar modelin
  bazı arabaları hiç görmemesinden (recall 0.75). Sonraki adım daha çok/çeşitli veri.

## Klasörler

```
airsim/settings.json            AirSim ayarları -> Belgeler\AirSim\settings.json
unreal/setup_level.py           Editörde çalışır: zemin, rastgele arabalar, helipad, paket modeli
unreal/setup_level_launcher.py  Türkçe karaktersiz yoldan çağırmak için (C:\UE\setup_level.py)
unreal/config/                  Unreal projesinin ayar dosyalarının kopyası (C:\UE\Sa\Config)
mission/autonomy.py             TAŞINABİLİR görev çekirdeği (ara, tespit, hizala, bırak, in, güvenlik)
mission/vehicle.py              Araç arayüzü: AirSimVehicle, Px4Vehicle (MAVSDK)
mission/perception.py           YOLO tespiti + eğim telafili piksel->yer geometrisi
mission/aruco_land.py           ArUco işaretçisiyle görüntüyle hassas iniş
mission/payload.py, camera.py   Paket bırakıcı (sim/gripper), gerçek kamera (OpenCV)
mission/run_autonomy.py         Görevi simülasyonda çalıştır (--backend airsim|px4)
mission/run_real.py             Gerçek drone: bench / connect / mission (onaylı)
mission/sim.sh                  Simülasyonu tek komutla başlat/durdur (px4|airsim|stop)
airsim/settings_px4.json        AirSim'i PX4 SITL'e bağlayan ayarlar (TCP 4560, lockstep)
training/visdrone_to_yolo.py    Gerçek drone veri seti (VisDrone) -> YOLO; evaluate.py ölçüm
docs/GERCEK_DRONE.md            Donanım, PX4 ayarları, test merdiveni, güvenlik
mission/sim_client.py           Ortak yardımcılar: bağlantı, kamera, tespit, yumuşak iniş
mission/fly_mission.py          Waypoint görevi + veri kaydı + helipad'e hassas iniş
mission/collect_data.py         Çok turlu otomatik veri toplama (her turda rastgele sahne)
mission/drop_test.py            Paket bırakma ve isabet ölçümü
mission/camera_check.py         Kamera sağlık testi (siyah/donmuş görüntü)
mission/diag_centering.py       Hizalama teşhisi
training/prepare_dataset.py     dataset/raw -> C:\UE\drone_data\yolo (train/val)
training/train.py               YOLO eğitimi
results/                        Deney sonuçları (CSV)
```

## Çalıştırma

1. Unreal'de `C:\UE\Sa\Sa.uproject`'i aç (masaüstündeki "Drone Sim").
2. Orman haritasını (yeniden) kurmak için Output Log'a: `py "C:/UE/setup_forest.py"`
   (eski düz harita: `setup_level.py`)
3. **▶ Play**, sonra:

```
python mission/collect_data.py --turlar 10 --goster     # veri topla
python training/prepare_dataset.py                       # veri setini hazırla
python training/train.py --model yolo11s.pt --epochs 60 --name car_forest2   # eğit (Unreal kapalıyken)
python mission/drop_test.py --deneme 10 --irtifa 25      # paket bırakma (kusursuz tespit)
python mission/drop_test.py --model C:/UE/drone_runs/car_forest2/weights/best.pt --irtifa 25   # hedefi YOLO bulsun
python mission/autonomous_mission.py --model C:/UE/drone_runs/car_forest2/weights/best.pt --teslimat 3 --goster
```

İnsansız (gece) çalıştırma: `bash mission/overnight.sh 30 20 car_forest2 60` — Unreal'i
`C:/UE/flags` bayraklarıyla kendisi açar, Play'e basar, veri toplar, kapatır, eğitir, yeniden
açıp testleri yapar. `C:/UE/flags/auto` dosyası varken `Content/Python/init_unreal.py`
editör açılışında `unreal/auto_editor.py`'yi devreye sokar; iş bitince bu dosya silinir.
Diğer bayraklar: `setup` = haritayı yeniden kur, `play` = Play'i başlat, `quit` = editörü kapat.

## Bilinen tuzaklar (UE 5.8 + Cosys-AirSim 3.5)

- AirSim kamerası **siyah** çeker: Unreal Orta kalitede (scalability) açılırsa, Lumen ya da
  Virtual Shadow Maps kapalıysa. Epic + Lumen + VSM açık kalmalı.
- Unreal çalışırken render ayarı değiştirmek (`simRunConsoleCommand`) kamerayı o oturum boyunca bozar.
- `settings.json` içindeki `SubWindows` alt kamerayı dondurur; önizleme için `--goster` kullan.
- Donanımsal ışın izleme açıkken NVIDIA sürücüsü çöküyor: `r.RayTracing=False`.
- Alt kamera gövdenin içindeyse tespit çizgisini drone'un kendisi keser: kamera Z=0.3.
- `cv2.imwrite` Türkçe karakterli yollara (Masaüstü) sessizce yazamıyor.

## Yol haritası

- [x] Waypoint görevi, otomatik etiketli veri, YOLO eğitimi, hassas iniş
- [x] Havada durarak paket bırakma
- [x] Orman haritası, gerçekçi araba modeli, her turda rastgele boya/konum/hava
- [x] Hedefi YOLO ile bulup bırakma (20 deneme: 14 hedef, 0 ağaç/yer)
- [x] Rotasız tam otonom görev (3 görev, 9/9 isabet)
- [x] İnsansız gece otomasyonu (Unreal'i bayrak dosyalarıyla aç/Play/kapat)
- [ ] Tespiti güçlendirme: daha çok sahne, recall 0.75 → 0.9
- [ ] Hareket halinde bırakma, rüzgar
- [ ] Arama-kurtarma: kişiyi bul, güvenli mesafeye yardım paketi bırak
- [ ] Sabit kanatlı versiyon
