# Otonom Drone Simülasyonu: Tespit + Paket Bırakma

Unreal Engine 5.8 + [Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) üzerinde döner kanatlı bir drone:
alanı gezer, alt kamerasıyla arabaları tespit eder, otomatik etiketli eğitim verisi toplar,
hedefin üstüne hizalanıp kargo paketi bırakır ve helipad'e hassas iniş yapar.

## Sonuçlar (şimdiye kadar)

| Deney | Sonuç |
|---|---|
| **Rotasız tam otonom görev** (ara → YOLO ile bul → hizalan → bırak → in), 3 görev × 3 paket | **9/9 isabet, 9 farklı araba, 0 yanlış alarm**, görev başına ~6 dk |
| Paket bırakma, **hedefi YOLO buluyor**, 25 m, 20 deneme | **14/20 hedef araba**, 2 yan araba, 0 ağaç/yer, 4 göremedi; deneme başına 56 s |
| Paket bırakma, kusursuz (Unreal) tespit, **10 m**, 10 deneme | **10/10**, ilk temas merkeze ort. 3 cm, 64 s/deneme |
| Paket bırakma, kusursuz (Unreal) tespit, **25 m**, 10 deneme | **10/10**, ilk temas merkeze ort. 3 cm, **49 s/deneme** (%23 daha hızlı) |
| YOLO11s araba tespiti, orman, 2237 görüntü / 50 tur | **mAP50 0.84**, precision 0.88, recall 0.75 — doğrulama **hiç görülmemiş 8 turda** |

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
- **Kalan darboğaz tespit:** bırakılan her paket bir arabaya isabet ediyor; kayıplar modelin
  bazı arabaları hiç görmemesinden (recall 0.75). Sonraki adım daha çok/çeşitli veri.

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
