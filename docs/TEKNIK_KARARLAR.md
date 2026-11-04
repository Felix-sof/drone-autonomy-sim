# Teknik Kararlar

Bu belge projedeki önemli kararları ve **neden** öyle verildiklerini kayıt altına alır. Her karar
aynı kalıpta yazılır:

- **Sorun:** neyi çözmeye çalışıyorduk
- **Seçenekler:** masadaki yollar
- **Karar ve neden**
- **Kanıt:** ölçülen sonuç (varsa)
- **Bedel:** bu kararın getirdiği kısıt ya da maliyet

Kararlar kronolojik sırada. Yeni karar verildikçe sona eklenir; geçersiz olanlar silinmez,
"yerini aldı: K-xx" diye işaretlenir.

---

## Genel bakış

| # | Karar | Kısaca neden |
|---|---|---|
| K-01 | Unreal Engine 5.8 + Cosys-AirSim | Görüntü işleme için fotogerçekçi kamera |
| K-02 | Render: Epic + Lumen + VSM, çalışırken render ayarı yok | Aksi hâlde AirSim kamerası siyah/bozuk |
| K-03 | Etiketleri Unreal üretir | Elle etiketleme yok, sınırsız veri |
| K-04 | Train/val ayrımı tur bazında | Kare bazında ayrım başarıyı şişirdi |
| K-05 | Simülasyon + gerçek veri karma eğitim | Sadece simülasyon: gerçekte mAP50 0.02 |
| K-06 | Taşınabilir görev çekirdeği | Aynı kod simülasyonda ve gerçek drone'da |
| K-07 | PX4 SITL + MAVSDK | Gerçek drone'ların uçuş yazılımı |
| K-08 | Stabilize gimbal, yaw kuzeye kilitli | Telemetri-görüntü senkron hatası ±5 m |
| K-09 | Kapalı döngü hız kontrolüyle hizalama + takip | Konum komutu erken duruyor, hedef sıçrıyordu |
| K-10 | Görüntüyle hassas iniş (ArUco) | GPS'ten bağımsız, cm hassasiyet |
| K-11 | İsabet ölçütü: ilk temas noktası | Paketin son durduğu yer yanıltıcı |
| K-12 | Kapsam: teslimat noktası ve arama-kurtarma | Araç/kişi üstüne nişanlama yapılmaz |
| K-13 | Varsayılan bırakma: seyir irtifasından (25 m) | En zor koşulda doğrulama |
| K-14 | Bırakmadan önce ince hizalama | 25 m'de hata 0.50 → 0.048 m |
| K-15 | Çok katmanlı güvenlik | Tek bir hata kazaya dönüşmesin |
| K-16 | Biyom rastgeleleştirme + 32 kişi | Kişi tespiti simülasyonda 0.19 → 0.56 |
| K-17 | HERIDAL gerçek SAR verisi, 2×2 parçalı | Ölçek drone'umuzla eşleşsin |
| K-18 | Rüzgar ve hava direnci modeli | Gerçekte 25 m'de metrelerce kayma |
| K-19 | Simülatörsüz kapalı döngü testler | Mantık hataları dakikalar değil saniyeler içinde |
| K-20 | Ölçüm altyapısının kendisini test etmek | İki kez yanlış ölçen kod yakalandı |
| K-21 | Yeni davranış isteğe bağlı eklenir | Kullanıcının seçtiği akış sessizce değişmez |
| K-22 | Ana mod: aracın tavanına bırakma (+ ince hizalama) | Hedef arabaya doğrudan isabet; 25 m'den tavana 10 cm |

---

## K-01 · Simülatör: Unreal Engine 5.8 + Cosys-AirSim

**Sorun:** Drone'un aşağı bakan kamerasıyla nesne tespiti yapacağız. Simülasyondaki görüntü
gerçeğe ne kadar benzerse, eğitilen model gerçekte o kadar iyi çalışır.

**Seçenekler:** Gazebo (PX4'ün varsayılanı, görsel olarak zayıf) · NVIDIA Isaac Sim (güçlü ama
ağır, 8 GB ekran kartı için zorlayıcı) · Unreal + AirSim.

**Karar ve neden:** Unreal + Cosys-AirSim. Fotogerçekçi render (Lumen ışıklandırma), hazır
kamera/tespit API'si, PX4 desteği. Orijinal Microsoft AirSim artık geliştirilmiyor; Cosys
sürümü UE 5.8'i destekliyor.

**Bedel:** Kurulum ve hata ayıklama zor (K-02); Unreal ekran kartının ~5 GB'ını kullanıyor, YOLO
eğitimi simülatör açıkken yapılamıyor.

## K-02 · Render ayarları sabit: Epic + Lumen + Virtual Shadow Maps

**Sorun:** AirSim kamerası bazen siyah, bazen mavi görüntü veriyordu.

**Bulgular:** Unreal "Orta" kaliteye düşünce, Lumen ya da VSM kapanınca kamera siyah çekiyor.
Çalışırken render ayarı değiştirmek (`simRunConsoleCommand`) bir iş parçacığı hatasına yol açıp
kamerayı o oturum boyunca bozuyor. Donanımsal ışın izleme NVIDIA sürücüsünü çökertiyor.

**Karar:** Epic kalite + Lumen + VSM açık, `r.RayTracing=False`. Render ayarları **sadece açılışta**,
asla çalışırken değiştirilmez. Mavi görüntü ayrı bir hataydı: Cosys RGB veriyor, OpenCV BGR
bekliyor; dönüşüm eklendi.

**Bedel:** Yüksek ekran kartı yükü; FPS düşük. Ağaçların rüzgârda sallanması kapatıldı, aksi
hâlde VSM bütün ormanın gölgesini her karede yeniden çiziyordu.

## K-03 · Etiketleri Unreal üretir (otomatik etiketleme)

**Sorun:** Binlerce görüntüyü elle etiketlemek imkânsız.

**Karar:** AirSim'in tespit API'si sahnedeki nesnelerin ekran kutularını verir; bunlar doğrudan YOLO
etiketine çevrilir. 12 pikselden küçük ve kadraj kenarında kırpılmış kutular atılır (öğrenilemezler,
skoru haksız düşürürler). Siyah ve donmuş kareler kaydedilmez.

**Bedel:** Etiket kalitesi Unreal'in bilgisine bağlı; ağaç altında görünmeyen bir nesne yine de
etiketlenebilir (açık konu).

## K-04 · Train/val ayrımı kare değil tur bazında

**Sorun:** İlk orman modeli doğrulamada mAP50 0.81 gösterdi, ama canlı uçuşta ortadaki arabayı
göremedi.

**Neden:** Art arda kareler neredeyse aynı. Kareleri rastgele bölünce doğrulama seti eğitimin
kopyası oldu, başarı olduğundan yüksek göründü (veri sızıntısı).

**Karar:** Ayrım **uçuş turu** bazında; bir turun bütün kareleri ya eğitimde ya doğrulamada.
Sonradan bir iyileştirme daha: tur, adının özetine göre val'e düşer. Yeni tur eklemek eski
turların yerini değiştirmez, böylece eski ve yeni modeller aynı doğrulama turlarında kıyaslanır.

**Kanıt:** Tur bazında ölçülen başarı canlı davranışla örtüştü; model daha çok sahne ve bırakma
açısından çekimle yeniden eğitilince canlıda çalıştı.

## K-05 · Simülasyon + gerçek veri ile karma eğitim

**Sorun:** Simülasyonda 0.84 alan model gerçek görüntüde işe yarar mı?

**Kanıt:** Gerçek drone görüntülerinde (VisDrone, 548 görüntü, 16 bin araç) mAP50 **0.02**. Sadece
simülasyonla eğitilen model gerçekte çalışmıyor.

**Karar:** Simülasyon + VisDrone birlikte eğitim. Sonuç: gerçekte **0.86**, simülasyonda 0.82 (simülasyon
başarısı korunarak). Kişi sınıfı için ayrıca HERIDAL eklendi (K-17).

**Bedel:** VisDrone ve HERIDAL araştırma/ticari olmayan kullanım içindir; proje ticari değil.
En büyük kazanç ileride kendi drone'umuzla çekilecek gerçek görüntülerden gelecek.

## K-06 · Taşınabilir görev çekirdeği

**Sorun:** Simülasyonda yazılan kodun gerçek drone'a taşınabilmesi.

**Karar:** `autonomy.py` simülatörü hiç bilmez; sadece arayüzleri kullanır:

| Arayüz | Simülasyon | Gerçek drone |
|---|---|---|
| Araç | `AirSimVehicle` / `Px4Vehicle` (SITL) | `Px4Vehicle` (seri port) |
| Kamera | AirSim görüntüsü | `OpenCVCamera` |
| Paket | `SimPayload` / `DragPayload` | `GripperPayload` (PX4 gripper) |

Simülasyona özel olan her şey (isabet ölçümü gibi) dışarıdan `on_drop` geri çağrısıyla verilir.

**Kanıt:** Aynı görev kodu hem AirSim'in kendi uçuş kontrolcüsüyle hem PX4 ile değişmeden çalıştı.

## K-07 · PX4 SITL + MAVSDK

**Sorun:** AirSim'in kendi uçuş kontrolcüsü (SimpleFlight) gerçek bir drone'da yok.

**Karar:** PX4 v1.15.4, WSL Ubuntu'da derlendi, AirSim'le lockstep. Python'dan MAVSDK
(`mavsdk<4`, sürüm 4'ün API'si farklı). Ağ: WSL `networkingMode=mirrored`, MAVSDK `udp://:14550`.

**Kanıt:** PX4 ile tam görev 12/12, 5-8 m/s rüzgarda 9/9; ArUco iniş ort. ~7 cm.

**Bedel:** MAVSDK'nin sağlık bayrakları bir görevden hemen sonra dakikalarca "hazır değil"
kalabiliyor (PX4 o sırada sağlıklı). Bayraklar bilgi amaçlı sayıldı: en fazla 30 s beklenir, asıl
kontrol PX4'ün arm reddidir (tekrar denenir).

## K-08 · Stabilize gimbal, yaw kuzeye kilitli

**Sorun:** PX4'te drone hedefin üstünde salınıyor, tahminler ±5 m sapıyordu.

**Neden:** Gövdeye sabit kamerada pikselden yere dönüşüm drone'un o anki eğimine ihtiyaç duyar.
MAVLink'ten gelen eğim ile kamera karesi farklı anlarda geliyordu; 25 m'de birkaç derecelik
uyumsuzluk metrelerce hataya dönüşüyor.

**Karar:** Stabilize gimbal: kamera her zaman tam aşağı bakar, görüntünün üstü kuzeydir. Pitch
**-89.9°**: tam -90°'de gimbal kilidi oluşuyor, görüntü her karede 0° ile 180° arasında dönüyordu.

**Kanıt:** Ölçüm hatası ±0.2 m'ye indi, PX4 görevleri 12/12 oldu.

**Bedel:** Gerçek drone'da gimbal **şart** (donanım maliyeti). Gimbal yoksa kare ile yönelim zaman
damgasıyla eşleştirilmeli.

## K-09 · Kapalı döngü hız kontrolüyle hizalama ve hedef takibi

**Sorun:** "Hedefin konumuna git" komutu erken duruyor, ölçek hataları birikiyordu. Bir görevde
3 paket aynı arabaya gitti: hedef bir kare kaybolunca drone yandaki arabaya kaydı.

**Karar:** Her karede hedefin yere izdüşümü hesaplanır, aradaki farka orantılı yumuşatılmış NED hız
komutu verilir. Takip: önceki karedeki kutuya en yakın kutu seçilir, ekran genişliğinin %12'sinden
büyük sıçrama reddedilir, birkaç kare kayıp tolere edilir. Gidilmiş/bırakılmış yerler hatırlanır.

**Kanıt:** Aynı arabaya tekrar bırakma ve komşu arabaya kayma bitti; 9/9 farklı araba.

## K-10 · Görüntüyle hassas iniş (ArUco işaretçisi)

**Sorun:** GPS'le iniş birkaç metre sapar; simülatörün bildiği konumu kullanmak hile olur.

**Karar:** Helipad'de 4 m'lik ArUco (DICT_4X4_50, id 0). Drone işaretçiyi kamerayla görür,
eğim telafili pikselden yere hesapla NED hızıyla ortalayarak alçalır.

**Kanıt:** AirSim'de 0-1 cm, PX4'te ort. ~7 cm. Eğim telafisi olmadan drone işaretçinin etrafında
salınıp uzaklaşıyordu (2 m/s kayan drone ~10° eğilir, 25 m'de 4-5 m sahte sapma).

## K-11 · İsabet ölçütü: ilk temas noktası

**Sorun:** Paket kavisli tavandan sekip yere düşebilir; son durduğu yere bakmak yanıltıcı.

**Karar:** İsabet = paketin **ilk temas** noktası hedefin ayak izinde mi. Ağaç, başka araç ve yer
ayrı sayılır.

**Düzeltme (2026-10-17):** Unreal nesne konumunu ~45 Hz günceller; daha sık okununca aynı konum
tekrar gelir. Eski kod bunu "hız sıfır = çarptı" sanıp ilk teması bazen düşüşün ortasında
işaretliyordu. Tekrar eden okumalar artık atlanıyor. O tarihten önceki tavan modu "ağaç/yer"
ayrımı yaklaşıktır.

## K-12 · Kapsam: teslimat noktaları ve arama-kurtarma; araç/kişi üstüne nişanlama yok

**Sorun:** Paket bırakma sistemi, nereye ve neye bırakılacağına göre çok farklı bir şeye dönüşebilir.

**Karar:**
- Hareket hâlindeki drone'dan bir araca isabet ettirmek için bırakma anını hesaplayan (balistik
  nişan) bir özellik **geliştirilmez**.
- Teslimat, araçların yanındaki **numaralı ArUco teslimat noktalarına** yapılır; işaretçi yoksa
  bırakılmaz.
- Arama-kurtarmada paket kişinin üstüne değil, **en az 5 m uzağa**, açık zemine bırakılır.
  Bırakmadan hemen önce alt yeniden kontrol edilir; kişi yaklaşmışsa bırakılmaz.
- Selde ya da mahsur kalmış kişi için ileride **ipli uzatma** (Wing tarzı vinç) var: drone kişinin
  yanında durur, paket ip ucunda kol mesafesine yavaşça iner. Kod hazır, varsayılan değil.

**Kanıt:** Teslimat noktası modu 12/12; işaretçilerin %40'ı kaldırılınca işaretçisiz araca
bırakma 0.

## K-13 · Varsayılan bırakma seyir irtifasından (25 m)

**Sorun:** Paket nereden bırakılmalı?

**Araştırma:** Ticari sistemler alçalıp bırakır: Amazon MK30 ~4 m, Wing ~7 m'de ipli vinç,
Skydio Dropper ~7.5 m.

**Karar:** Varsayılan **25 m'den**, alçalmadan bırakmak. Amaç sistemi en zor koşulda doğrulamak:
1 piksellik hata 25 m'de 5 m'dekinin 5 katı. En yüksekten tutturan sistem alçakta çok daha iyi
olur. Plan: 25 m oturunca 50 m, sonra 75 m. Alçalarak bırakma isteğe bağlıdır
(`--birakma-irtifa 5`).

**Kanıt (karşılaştırma, PX4, 9'ar paket):**

| Bırakma | İsabet | Merkeze ort. hata | Görev süresi |
|---|---|---|---|
| 25 m (ilk hâl) | 9/9 | 0.50 m | ~171 s |
| 5 m'ye alçalarak | 9/9 | 0.05 m | ~232 s |
| 25 m + ince hizalama (K-14) | 9/9 | **0.048 m** | ~198 s |

**Bedel:** Gerçekte rüzgarın etkisi yüksekten bırakmada çok daha büyük (K-18).

## K-14 · Bırakmadan önce ince hizalama

**Sorun:** 25 m'de hata hep ~0.55 m civarındaydı. Rastgele değil, toplanmış bir hata.

**Teşhis:** Kayıtlara hatanın bileşenleri eklendi: bırakma anında drone işaretçiye göre neredeydi,
paket drone'a göre nereye düştü. Sonuç: düşüş kayması ~0, sabit bir sapma yok. Hatanın tamamı
drone'un konumundan geliyordu: hizalama toleransı 30 cm'ydi ve drone tam durmadan bırakıyordu.

**Karar:** Drone durunca işaretçi 8 kare boyunca ölçülür; ortalama sapma 12 cm'den büyükse tekrar
hizalanır, sonra bırakılır. Sabit sapma çıkarsa diye bir telafi ayarı da var
(`release_bias_ne_m`), gerek kalmadı.

**Kanıt:** 0.50 → **0.048 m** (en kötü 0.08 m). 25 m'de bir piksel yerde ~2.6 cm; yani ~2 piksel.

## K-15 · Çok katmanlı güvenlik

Tek bir hatanın kazaya dönüşmemesi için katmanlar:

1. **Geofence:** görev alanının 15 m dışına çıkılırsa görev iptal, eve dönüş.
2. **Pil ve süre sınırı:** %25 pil ya da 900 s'de eve dönüş.
3. **Güvenli mesafe:** kişiye 5 m'den yakın bırakma yok; alçalırken her karede kontrol.
4. **Gerçek uçuş onayı:** `run_real.py mission` açık onay (`--onay UCUS-ALANI-BOS`) olmadan çalışmaz.
5. **Her bırakmada pilot onayı:** canlı görüntü + konsolda `e`. 20 s içinde cevap yoksa bırakılmaz
   (sessizlik = hayır). Arama-kurtarmada kapatılamaz.
6. PX4'ün kendi geofence/RTL ayarları bunlara ektir (bkz. GERCEK_DRONE.md).

## K-16 · Biyom rastgeleleştirme ve 32 kişi

**Sorun:** İki sınıflı model simülasyondaki kişileri görüyordu ama emin değildi (güven ≤ 0.32,
eşik 0.45). Arama-kurtarma görevleri kimseyi bulamadan bitti. Teşhis uçuşu: kişiler kadrajdaydı
(ayakta ~42 px, yatan ~90 px).

**Neden:** Simülasyonda az ve tekdüze kişi verisi: 8 beyaz manken, tek zemin, 1397 etiket.
VisDrone'da 106 bin kişi vardı ama görünüşleri farklı.

**Karar:**
- 5 biyom: yaz ormanı, kış ormanı (karlı ladin), seyrek orman, çöl (palmiye, çöl ağacı, çalı),
  kayalık. 12 zemin malzemesi; dokular renk çarpanlarıyla çeşitlendirildi.
- Her turda açıklıkların yeri ve ağaç sıklığı değişir; kullanılmayan bitki havuzu görüş dışına
  park edilir.
- 32 kişi: ayakta, yürüyen, koşan (animasyonun rastgele bir anında dondurulmuş) ve yatan;
  12 kıyafet rengi, %20'si beyaz manken.

**Kanıt:** Simülasyon kişi etiketi 1397 → **9641**. Yeni model (`carperson3`: simülasyon 5 biyom +
VisDrone + HERIDAL, 1280 px, 12 epoch) eski modelle aynı doğrulama setlerinde (mAP50):

| Veri seti | Sınıf | Eski | Yeni |
|---|---|---|---|
| Simülasyon (görülmemiş turlar) | kişi | 0.192 | **0.561** (recall 0.21 → 0.62) |
| HERIDAL (gerçek SAR) | kişi | 0.256 | **0.683** |
| VisDrone (gerçek) | kişi | 0.594 | **0.617** |
| VisDrone (gerçek) | araba | 0.892 | 0.894 |
| Simülasyon | araba | 0.870 | 0.879 |

Kişi tespiti ~3 kat iyileşti, araba tespitinde kayıp yok.

**Görevde (2026-11-04, AirSim, 3 biyom, 3 görev × 3 paket):** 9/9 güvenli teslimat, 0 tehlikeli bırakma;
paketler kişiye 5.4–6.6 m (hedef 6.5 m). Eski model aynı görevde kimseyi bulamıyordu. Pay dar: bırakma
noktasına konuma göre gidiliyor, kişiye göre görüntüyle son düzeltme yok (±1 m) → yol haritasına eklendi.

**Eğitim tuzağı:** 1280 px'de batch 4, 8 GB ekran kartını aşıp paylaşılan belleğe taşınıyor ve eğitim
~25 kat yavaşlıyor (kart %100 görünür ama ~28 W çeker). Batch 3 ile ~5.5 GB'ta kalıyor.

**Bedel/tuzak:** AirSim'in `simSetObjectMaterial`'ı iskeletli modelde `True` döndürüp hiçbir şey
yapmıyor. Kıyafet rengi bu yüzden harita kurulumunda kalıcı veriliyor.

## K-17 · HERIDAL gerçek arama-kurtarma verisi, 2×2 parçalı

**Sorun:** Kişi için gerçek, yukarıdan, kırsal alanda çekilmiş veri.

**Karar:** HERIDAL (Hırvatistan, 40-65 m, 1650 görüntü, sadece kişi). Görüntüler 4000×3000, kişiler
~60 px. Tüm görüntü 1280'e küçültülse kişi ~20 px'e iner. 2×2 parçaya bölünce eğitimde ~38 px
olur; bu, drone'umuzun 25 m'den gördüğü boyuta (40-90 px) yakın. Kişisiz parçaların %25'i tutulur.

**Kanıt:** 2735 eğitim + 243 doğrulama parçası, 3356 kişi kutusu; etiketler görsel olarak doğrulandı.

## K-18 · Rüzgar ve hava direnci modeli

**Sorun:** Simülasyonda paket rüzgardan ve hava direncinden etkilenmiyor; 4.8 cm'lik sonuç bu
yüzden gerçeğe doğrudan taşınmaz.

**Hesap (1 kg, 40×30 cm kutu, Cd ~1.05; hava direnciyle ~11 m/s'de sabitleniyor):**

| Bırakma | 3 m/s rüzgar | 5 m/s | 8 m/s |
|---|---|---|---|
| 25 m | 4.4 m kayma | 7.9 m | 14.0 m |
| 50 m | 10.7 m | 18.5 m | 31.0 m |
| 75 m | 17.3 m | 29.5 m | 48.7 m |

**Karar:**
- Düşüş adım adım hesaplanır (yerçekimi + hava direnci + rüzgar).
- Simülasyonda paketin hareketini bu hesap sürer (`--paket-fizigi gercekci`).
- Rüzgar drone'un havada dururken eğiminden tahmin edilir (PX4 EKF2 "drag fusion" ile aynı fikir).
- Bırakma noktası rüzgarın geldiği yöne kaydırılır (`--ruzgar-telafi bilinen|egim`). Sadece
  havada dururken ve işaretli teslimat noktalarında.
- Paketin sürüklenme katsayısı birkaç test bırakmasıyla ölçülür.

**Kalibrasyon (2026-11-04, AirSim):** drone 25 m'de 8 s oturup 4 s ölçtüğünde `tan(eğim) = k·v²` modeli
tutuyor (doğu rüzgarında 3, 5, 8 m/s için k = 0.00072); k = 0.00065 ile tahmin hatası 0.05–0.6 m/s, yön
çoğunlukla ≤ 6° (çapraz rüzgarda en kötü 14°). Kodda varsayılan 0.0045'ti: ~7 kat yanlış.

**Karşılaştırma (doğudan 5 m/s, 25 m'den tavana, gerçekçi paket fiziği, 3'er paket):**

| Telafi | Tavana isabet | Merkeze |
|---|---|---|
| yok | 0/3 | 5.9 / 8.2 / 8.1 m |
| gerçek rüzgarla | **3/3** | **0.11 / 0.07 / 0.03 m** |
| eğimden tahminle | 1/3 | 1.8 / 5.9 / 4.7 m |

Hesap doğru; zayıf halka görev içindeki rüzgar tahmini: hizalanma manevrasından hemen sonra 1.5 s ölçülüyor,
kalan manevra eğimi tahmini 3 m/s ve 45°'ye kadar saptırıyor. **Sonraki adım:** bırakmadan önce birkaç
saniye oturup daha uzun ölçmek ya da rüzgarı arama boyunca sürekli ölçüp ortalamak.

**Bedel:** Paket tasarımı da hassasiyetin parçası: daha ağır ve küçük paket kabaca yarı kayma.

## K-19 · Simülatörsüz kapalı döngü testler

**Sorun:** Her değişikliği simülatörde denemek 5-15 dakika.

**Karar:** Sahte drone (hız komutunu gecikmeyle izleyen nokta kütle) + sahte kamera/tespit ile görev
mantığı test edilir: hizalama yakınsıyor mu, başka arabaya sıçrıyor mu, kişiye güvenli mesafe,
alçalırken kişi görünce vazgeçme, ince hizalama, rüzgar hesabı, pilot onayı.

**Kanıt:** 42 test, ~1.5 saniyede.

## K-20 · Ölçüm altyapısının kendisini test etmek

**Sorun:** Yanlış ölçen bir sistem, çalışmayan bir şeyi çalışıyor gösterir.

**Yakalananlar:**
1. Kare bazında train/val ayrımı (K-04).
2. Çarpma anı tekrar eden okumalarda yanlış işaretleniyordu (K-11).
3. Çarpma hızı tek aralıktan hesaplanınca ~%30 yüksek çıkıyordu. Artık ~0.15 s'nin ortalaması
   alınıp serbest düşüş düzeltmesi (`g·Δt/2`) ekleniyor.

**Karar:** Ölçüm kodu da birim testine tabi (`tests/test_impact.py`); şüpheli sonuçlar fizikle
karşılaştırılır (ör. 25 m'den çarpma ~22 m/s olmalı).

## K-21 · Yeni davranış isteğe bağlı eklenir

**Sorun:** Araştırmaya dayanarak varsayılan bırakmayı 5 m'ye çevirmiştim; bu, kullanıcının seçtiği
akışı sessizce değiştirdi.

**Karar:** Yeni fikirler **seçenek olarak** eklenir (`--birakma-irtifa`, `--teslim ip`,
`--paket-fizigi`, `--ruzgar-telafi`); sonuçlar ölçülüp raporlanır, varsayılan yapılmadan önce sorulur.


## K-22 · Ana mod: aracın tavanına bırakma, ince hizalamayla

**Sorun:** Teslimat noktası modunda paket aracın 3.8 m yanındaki işaretçiye düşüyor. Dışarıdan bakan
biri bunu "arabayı bulmuş ama ıskalamış" diye okuyor (kullanıcı geri bildirimi, 2026-11-04).

**Karar (kullanıcının seçimi):** Varsayılan görev modu `roof`: paket havada sabit duran drone'dan, park
hâlindeki hedef arabanın tavanına bırakılır. Teslimat noktası modu `--mod dropzone` ile seçilebilir.
Teslimat noktasındaki ince hizalama (K-14) tavan moduna da eklendi: araba kutusunun merkezi 8 karenin
ortalamasıyla ölçülür, 12 cm'den uzaksa sıkı toleransla yeniden hizalanılır.

**Değişmeyenler:** Kişilerin üstüne bırakma yok (arama-kurtarmada en az 5 m, K-12); hareket hâlinde
bırakma / balistik nişan yok.

**Kanıt:** AirSim'de 25 m'den iki deneme: ilk temas hedef arabanın tavanında, merkeze 12 cm ve 10 cm.
Demo videosu: `docs/videos/arac_ustu.mp4`.

---

## Açık konular

- PX4 SITL bağlantısı 2026-11-04'te güvenilmez hâle geldi (Windows/WSL güncellemesi sonrası; netstat AirSim
  portlarını göstermiyor, MAVLink UDP verisi gelmiyor). Görevler şimdilik `--backend airsim` ile; `wsl --shutdown`/yeniden
  başlatma sonrası PX4 tekrar denenecek.

- Rüzgar telafisinin simülasyonda doğrulanması; 50 m ve 75 m bırakma.
- 75 m'de 2 m'lik işaretçi ~25 px: daha büyük işaretçi ya da iki aşamalı hizalama.
- 50-75 m'de kişi tespiti için görüntüyü parçalayarak tarama (tiling).
- Ağaç altında görünmeyen ama etiketlenen nesneler (etiket kalitesi).
- Gerçek donanım: kamera kalibrasyonu, titreşim, gecikme, gerçek veri.

## Yol haritası (2026-11-04, sırayla)

1. **Arama-kurtarma testleri** yeni modelle, rastgele biyomlarda (kişi bulma oranı, bırakma isabeti,
   tehlikeli bırakma = 0).
2. **Rüzgar telafisi:** kalibrasyon ve karşılaştırma yapıldı (K-18): hesap doğru (bilinen rüzgarla 3/3, 3–11 cm);
   görev içi eğim tahmini iyileştirilecek (oturma süresi / sürekli ortalama).
3. **50 m ve 75 m bırakma** (önce araçların yanındaki teslimat noktalarında; büyük işaretçi ya da iki
   aşamalı hizalama).
4. **Hedef filtresi:** görevde `--hedef araba | insan | ikisi`.
5. **Haritadan görev seçme:** haritanın üstten görüntüsü üzerinde arama alanı, nokta ve teslimat
   noktası çizen bir arayüz → görev dosyası → drone bu dosyayla uçar (gerçek drone'da da aynı dosya).
5b. **SAR bırakma noktasında ince hizalama:** kişiye göre görüntüyle son düzeltme (şu an ±1 m; 5 m sınırına
   pay ~0.4 m).
6. **İnsanlara yardım ulaştırma:** araçlarda başarı oturduktan sonra ana hedef kişiler. Paket kişiye
   nişanlanmaz: en az 5 m uzağa açık zemine, sonra ipli teslimatla kol mesafesine (K-12).
7. **Sabit kanat:** havada duramaz; hedef üstünde dönerek uçar, paraşütlü bırakma, işaretli teslimat
   noktaları. Görev dosyası, tespit ve güvenlik katmanları taşınır.
