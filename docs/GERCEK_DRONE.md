# Gerçek Drone'a Geçiş Rehberi (döner kanat, PX4)

Simülasyondaki görev kodu (`mission/autonomy.py`) değişmeden gerçek drone'da çalışacak şekilde
yazıldı. Değişen yalnızca arka uçlar:

| Arayüz | Simülasyon | Gerçek drone |
|---|---|---|
| Araç (`vehicle.py`) | `AirSimVehicle` / `Px4Vehicle` (SITL) | `Px4Vehicle("serial:///dev/ttyTHS1:921600")` |
| Kamera | AirSim `simGetImages` | `camera.OpenCVCamera` (USB/CSI/RTSP) |
| Paket | `payload.SimPayload` (fizik nesnesi) | `payload.GripperPayload` (PX4 gripper komutu) |
| İniş | ArUco işaretçisi (Unreal'de doku) | Basılı ArUco işaretçisi (DICT_4X4_50, id 0) |

## 1. Donanım (öneri)

- **Gövde + motor + ESC:** PX4 desteği iyi olan bir geliştirme kiti (ör. Holybro X500 sınıfı, 450-550 mm).
- **Uçuş kontrol kartı:** Pixhawk sınıfı, PX4 v1.15 ile uyumlu (ör. Pixhawk 6C).
- **GPS + pusula** modülü (M10 sınıfı).
- **Mesafe sensörü (önerilir):** aşağı bakan lidar (TF-Luna/LightWare sınıfı). Barometre birkaç
  metre sapar; hassas bırakma ve iniş için zeminden yükseklik önemli (`height_agl()`).
- **Yardımcı bilgisayar:** NVIDIA Jetson Orin Nano sınıfı (YOLO'yu TensorRT ile gerçek zamanlı
  çalıştırır). Pixhawk'a TELEM2 seri portundan bağlanır.
- **Kamera + gimbal (ŞART):** aşağı bakan kamera **stabilizasyonlu gimbal**'da, yaw kilitli
  ("yaw lock", görüntünün üstü kuzey). ~90° yatay görüş (farklıysa `HFOV_DEG`'i güncelle).
  Neden: PX4 SITL'de gövdeye sabit kamerayla, MAVLink'ten gelen yönelim ile kamera karesi farklı
  anlarda geldiği için eğim telafisi 25 m'de ±5 m sahte hata üretti ve drone hedef üstünde
  salındı. Gimbal'lı kamerayla ölçülen sapma gerçeğe ±0.2 m oturdu, görev 3/3 isabetle bitti.
  Gimbal yoksa `perception.CAMERA_STABILIZED=False` ile eğim telafisi kullanılır ama kamera
  karesi ile yönelim zaman damgasıyla eşleştirilmelidir.
- **Paket bırakıcı:** servo kancası/tutucu, Pixhawk AUX çıkışına.
- **Kumanda (RC) + telemetri telsizi:** her an manuel devralma için ŞART.

## 2. PX4 ayarları (QGroundControl)

| Parametre | Değer | Neden |
|---|---|---|
| `MAV_1_CONFIG` | TELEM 2 | Yardımcı bilgisayar MAVLink bağlantısı |
| `SER_TEL2_BAUD` | 921600 | Hızlı telemetri/komut |
| `PD_GRIPPER_EN` | 1 | Tutucu (gripper) etkin; çıkışını Actuators sekmesinde ata |
| `GF_ACTION` | Return (RTL) | PX4'ün kendi geofence'i (kodumuzdakine ek) |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` | ör. 120 m / 40 m | Test alanına göre |
| `RTL_RETURN_ALT` | ağaçların üstü (ör. 30 m) | Eve dönüşte çarpmasın |
| `COM_LOW_BAT_ACT` | Return | Düşük pilde eve dön |
| `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` | 1 s / Position | Yardımcı bilgisayar susarsa yerinde dur |
| `COM_RC_IN_MODE` | RC zorunlu | Kumanda olmadan kalkmasın |

Kamera kalibrasyonu: OpenCV satranç tahtası kalibrasyonuyla odak uzaklığı ve bozulmayı ölç;
`perception.pixel_ray_level` ideal iğne deliği (pinhole) kamera varsayıyor.

## 3. Model

- Sadece simülasyonla eğitilen model gerçek görüntüde çalışmıyor (VisDrone'da mAP50 **0.02**).
- Karma eğitim (simülasyon + VisDrone gerçek görüntüleri) bunun için: `training/mixed_data.yaml`.
- Kendi drone'unla test alanında birkaç uçuş görüntüsü çek, etiketle, eğitime ekle — en büyük
  kazanç buradan gelir.
- Jetson için: `yolo export model=best.pt format=engine half=True imgsz=960` (cihazın üstünde).
- Ekran kartsız işlemcide (ONNX, laptop CPU) ölçüm: 640 px'de ~3 kare/s, 960 px'de ~1.6 kare/s —
  canlı hizalama için YETERSİZ. GPU/NPU'lu yardımcı bilgisayar gerekli (Jetson sınıfı).
- Model durumu (`car_mixed2`, simülasyon + VisDrone): gerçek drone görüntülerinde (VisDrone val)
  mAP50 0.863, simülasyonda 0.823. Sadece simülasyonla eğitilen model gerçekte 0.02'ydi.

## 3b. Bırakma irtifası: alçal, sonra bırak

Ticari sistemler paketi seyir irtifasından atmaz: Amazon MK30 ~4 m'ye alçalıp bırakır, Wing ~7 m'de
durup paketi iple (vinç) indirir, Skydio Dropper kılavuzu kurtarma şamandırası için ~7.5 m önerir.
Görev kodu da aynı şekilde çalışır (`--birakma-irtifa`, dropzone 5 m, SAR 8 m):
seyir irtifasında işaretçiye hizalan → işaretçiye ortalı kalarak dikey alçal (1.5 m/s) → altta
kişi görülürse vazgeç → bırak → seyir irtifasına dikey çık.

Gerçek drone'da bunun için:
- **Aşağı bakan mesafe sensörü ŞART.** PX4'te `height_agl()` kalkış noktasına göre irtifadır;
  eğimli arazide 5 m hedef, yerden 2 m ya da 8 m olabilir. `Px4Vehicle.height_agl()` PX4'ün
  yayınladığı `DISTANCE_SENSOR` mesajını otomatik kullanır (son 0.5 s içinde, menzil içinde ve
  eğim < 30° ise, eğim düzeltmeli); yoksa kalkış noktasına göre irtifaya düşer. Lidar menzili
  (TF-Luna ~8 m, LightWare SF11 ~100 m) bırakma irtifasından büyük olmalı. PX4'te sensörü
  tanıt (`SENS_EN_TFLUNA` vb.) ve `EKF2_RNG_CTRL` ile EKF'e de ver. Not: sensör paketi bırakırken
  teslimat noktasındaki nesneyi (araba tavanı, çalı) de ölçer; noktalar düz zeminde olmalı.
- **Alçalma koridoru boş olmalı:** ağaç dalı, kablo, direk. Kod bunları görmez; teslimat
  noktaları açık alanda seçilmeli (simülasyonda açıklıklarda).
- Alçalırken pervane rüzgârı hafif paketleri savurur, toz kaldırır; paket kapalı ve ağırlığı
  dengeli olmalı.
- Wing tarzı vinç (ipli indirme) drone'u daha yüksekte tutar ve çarpma hızını sıfıra yaklaştırır;
  donanım karmaşıklığı karşılığında en güvenli seçenektir.

## 4. Test merdiveni (her basamağı geçmeden sonrakine geçme)

1. `python mission/run_real.py bench --camera 0 --goster` — kamera + tespit, drone yok.
2. Pervaneler SÖKÜLÜ: `run_real.py connect --px4 ...` — telemetri, yönelim, pil doğru mu.
3. Pervaneler sökülü, gripper testi: `GripperPayload.release()` servo hareket ediyor mu.
4. Manuel uçuş (PX4 Position modu), kayıt alarak: kamera görüntüsü titreşimsiz mi.
5. Offboard'a hafif geçiş: düşük irtifa, sadece `goto` / `hover`; pilot her an devralmaya hazır.
6. ArUco iniş testi (paketsiz).
7. Tek hedef, paketsiz "bırakma" (tutucu boş açılır) — hizalama doğruluğunu ölç.
8. Yumuşak, hafif, kırılmayan bir paketle, sabit bir hedefe (yere serilmiş branda/işaret) bırakma.

## 5. Güvenlik ve yasal

- Türkiye'de İHA kaydı, pilot yetkinliği ve uçuş izinleri SHGM kurallarına tabidir; havadan cisim
  bırakmak ayrıca kısıtlıdır. Güncel mevzuatı kontrol et, sadece izinli alanda uçur.
- Uçuş alanında insan olmasın; hedef olarak sadece kendi aracın ya da yere serilmiş bir işaret.
- Bir gözlemci + kumandası elinde bir pilot olmadan otonom görev başlatma.
- `run_real.py mission` açık onay (`--onay`) olmadan çalışmaz.
- **Her bırakmadan önce pilot onayı:** drone bırakma noktasında (alçalmış hâlde) havada durur,
  canlı görüntüyü gösterir ve konsolda `e` bekler; 20 s içinde onay gelmezse paketi bırakmadan
  seyir irtifasına döner. dropzone modunda boş test alanı için `--onaysiz-birak` ile kapatılabilir,
  `--mod-gorev sar`'da kapatılamaz.

## 6. Arama-kurtarma (SAR) için ek güvenlik kuralları

Gerçek kişilerin bulunduğu bir ortamda (`run_autonomy.py --mod sar`, `mission/sar.py`):

- **Paket asla kişinin üstüne bırakılmaz.** Kod, bırakma noktasını kişiden en az `SAFE_MIN_M`
  (5 m) uzakta, diğer kişi/araçlardan uzak ve görüntüde açık zemin görünen bir yerden seçer;
  bırakmadan hemen önce altını yeniden kontrol eder, kişi yaklaşmışsa bırakmaz.
- Tespit modeli kişileri **kaçırabilir** (küçük, ağaç altında, sıra dışı duruş). Bu yüzden
  "altında kimse görünmüyor" kesinlik değildir: gerçek uçuşta bırakmadan önce **pilot onayı**
  (canlı görüntüyle) zorunludur — `run_real.py --mod-gorev sar` bunu her pakette ister ve
  kapatılamaz; otomatik bırakma sadece simülasyonda.
- Paket hafif, yumuşak (köpük/kumaş) ya da paraşütlü olmalı; drone bırakma noktasına
  alçalarak bırakır (SAR varsayılanı 8 m; alçalırken her karede alt yeniden kontrol edilir,
  kişi görülürse bırakmadan yükselir). Seyir irtifası ağaç tepelerinin üstünde kalır (25 m).
- Kişinin konumu ve görüntüsü kişisel veridir: kayıtları güvenli sakla, görev bitince paylaşma.
- Gerçek bir arama-kurtarma operasyonunda bu sistem yetkili ekiplerin koordinasyonunda,
  onların kurallarına göre kullanılmalıdır; tek başına bir kurtarma aracı değildir.
