# Otonom Drone Simülasyonu: Tespit + Paket Bırakma

Unreal Engine 5.8 + [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) üzerinde döner kanatlı bir drone:
alanı gezer, alt kamerasıyla arabaları tespit eder, otomatik etiketli eğitim verisi toplar,
hedefin üstüne hizalanıp kargo paketi bırakır ve helipad'e hassas iniş yapar.

## Sonuçlar (şimdiye kadar)

| Deney | Sonuç |
|---|---|
| YOLO11n araba tespiti (201 sim görüntüsü, 798 kutu) | mAP50 0.83, precision 0.79, recall 0.80, 2 ms/kare (RTX 4060 Laptop) |
| Havada durarak paket bırakma, 10 m, kusursuz tespit | **5/5 isabet**, hizalama ~3 cm, paketin merkeze uzaklığı ort. 0.52 m |

## Klasörler

```
airsim/settings.json            AirSim ayarları -> Belgeler\AirSim\settings.json
unreal/setup_level.py           Editörde çalışır: zemin, rastgele arabalar, helipad, paket modeli
unreal/setup_level_launcher.py  Türkçe karaktersiz yoldan çağırmak için (C:\UE\setup_level.py)
unreal/config/                  Unreal projesinin ayar dosyalarının kopyası (C:\UE\Sa\Config)
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
2. Harita değiştiyse Output Log'a: `py "C:/UE/setup_level.py"`
3. **▶ Play**, sonra:

```
python mission/collect_data.py --turlar 10 --goster     # veri topla
python training/prepare_dataset.py                       # veri setini hazırla
python training/train.py                                 # eğit (Unreal Play kapalıyken)
python mission/drop_test.py --deneme 5 --irtifa 10       # paket bırakma testi
python mission/drop_test.py --model C:/UE/drone_runs/car/weights/best.pt   # hedefi YOLO bulsun
```

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
- [ ] Gerçekçi araba modelleri + farklı zeminler, daha fazla veri
- [ ] Hedefi YOLO ile bulup bırakma (30+ deneme)
- [ ] Rotasız tam otonom görev (ara → tespit → bırak → in)
- [ ] Hareket halinde bırakma, rüzgar
- [ ] Arama-kurtarma: kişiyi bul, güvenli mesafeye yardım paketi bırak
- [ ] Sabit kanatlı versiyon
