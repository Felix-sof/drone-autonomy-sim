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
- **Kamera:** aşağı bakan, gövdeye sabit; ~90° yatay görüş (simülasyonla aynı; farklıysa
  `perception.py`/`aruco_land.py`'deki `HFOV_DEG`'i güncelle). Mümkünse global shutter.
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
