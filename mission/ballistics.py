"""Havada duran drone'dan bırakılan paketin düşüşü: yerçekimi + hava direnci + rüzgar.

Teslimat noktasına (işaretli levha) bırakırken rüzgarın paketi ne kadar sürükleyeceğini hesaplar;
drone bırakma noktasını bu kaymanın tersine kaydırır (ticari teslimat drone'larının yaptığı gibi).
Sadece drone havada DURURKEN ve işaretli teslimat noktaları için kullanılır.

Paket modeli: kütle m, sürüklenme alanı cd_a = Cd * A (m^2). Kuvvet:
    m dv/dt = m g - 0.5 * rho * cd_a * |v - w| (v - w)       (NED, w = rüzgar)
cd_a paketin kendi özelliğidir: birkaç test bırakmasıyla bir kez ölçülür (fit_cd_a).

Rüzgar tahmini (wind_from_tilt): drone havada sabit dururken rüzgarın ittiği yöne karşı eğilir.
Yatay denge: m_d g tan(eğim) = k_d * v_w^2 -> v_w = sqrt(m_d g tan(eğim) / k_d).
k_d drone'a özgüdür; bilinen rüzgarda bir kez ayarlanır (calibrate_k).
"""
import math

G = 9.81
RHO = 1.225          # deniz seviyesi hava yoğunluğu (kg/m^3)

# simülasyondaki paket (0.4 x 0.3 x 0.25 m karton kutu, ~1 kg); gerçek pakette ölçülmeli
PACKAGE_MASS_KG = 1.0
PACKAGE_CD_A = 1.05 * 0.4 * 0.3     # kutu Cd ~1.05, üstten kesit 0.12 m^2


def simulate_fall(height, wind_ne=(0.0, 0.0), mass=PACKAGE_MASS_KG, cd_a=PACKAGE_CD_A,
                  v0=(0.0, 0.0, 0.0), dt=0.005, max_t=30.0):
    """Paket height m yüksekten bırakılır. (Δn, Δe, düşüş_süresi, çarpma_hızı_vektörü) döndürür.

    v0: bırakma anındaki paket hızı (drone'un hızı, NED). Rüzgarın dikey bileşeni yok sayılır.
    """
    wn, we = wind_ne
    n = e = 0.0
    d = -height            # NED: zemin d = 0
    vn, ve, vd = v0
    t = 0.0
    k = 0.5 * RHO * cd_a / mass
    while d < 0.0 and t < max_t:
        rn, re, rd = vn - wn, ve - we, vd
        s = math.sqrt(rn * rn + re * re + rd * rd)
        an, ae, ad = -k * s * rn, -k * s * re, G - k * s * rd
        # yarı-örtük Euler (kararlı, dt küçük)
        vn += an * dt
        ve += ae * dt
        vd += ad * dt
        n += vn * dt
        e += ve * dt
        d += vd * dt
        t += dt
    return n, e, t, (vn, ve, vd)


def release_offset(height, wind_ne, **kw):
    """Drone'un hedefin ne kadar (Δn, Δe) yanında bırakması gerektiği: kaymanın tersi."""
    dn, de, _, _ = simulate_fall(height, wind_ne, **kw)
    return -dn, -de


def fit_cd_a(height, wind_ne, measured_drift_m, mass=PACKAGE_MASS_KG, lo=1e-4, hi=2.0):
    """Bilinen rüzgarda ölçülen kaymaya (m, rüzgar yönünde) uyan cd_a (ikiye bölme)."""
    for _ in range(60):
        mid = math.sqrt(lo * hi)
        dn, de, _, _ = simulate_fall(height, wind_ne, mass=mass, cd_a=mid, dt=0.01)
        drift = math.hypot(dn, de)
        if drift < measured_drift_m:
            lo = mid
        else:
            hi = mid
    return math.sqrt(lo * hi)


def tilt_vector_ne(roll, pitch, yaw):
    """Gövdenin eğim yönü (yatay itki bileşeni), NED (n, e) ve eğim açısı (rad).

    Pozitif pitch burun yukarı (itki geriye), pozitif roll sağ kanat aşağı (itki sağa).
    Küçük açılar için itkinin yatay bileşeni gövdede (x: -sin(pitch), y: sin(roll)*cos(pitch)).
    """
    bx = -math.sin(pitch)
    by = math.sin(roll) * math.cos(pitch)
    n = bx * math.cos(yaw) - by * math.sin(yaw)
    e = bx * math.sin(yaw) + by * math.cos(yaw)
    tilt = math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch))))
    return n, e, tilt


def wind_from_tilt(roll, pitch, yaw, k_d):
    """Havada sabit duran drone'un eğiminden rüzgar (wn, we) m/s.

    Drone rüzgara DOĞRU eğilir (itki rüzgara karşı); rüzgar itkinin yatay bileşeninin tersine eser.
    k_d: drone'un m_d*g'ye bölünmüş sürüklenme katsayısı (tan(eğim) = k_d * v^2).
    """
    n, e, tilt = tilt_vector_ne(roll, pitch, yaw)
    h = math.hypot(n, e)
    if h < 1e-6 or tilt < 1e-4:
        return 0.0, 0.0
    v = math.sqrt(math.tan(tilt) / k_d)
    return -n / h * v, -e / h * v


def tilt_feature(roll, pitch, yaw):
    """Eğimden 'ham rüzgar' vektörü u (rüzgar yönünde, büyüklüğü sqrt(tan(eğim))).
    Tek katsayılı modelde rüzgar = u / sqrt(k); yöne bağlı modelde rüzgar = M @ u."""
    n, e, tilt = tilt_vector_ne(roll, pitch, yaw)
    h = math.hypot(n, e)
    if h < 1e-6 or tilt < 1e-4:
        return 0.0, 0.0
    m = math.sqrt(math.tan(tilt))
    return -n / h * m, -e / h * m


def wind_from_tilt_matrix(roll, pitch, yaw, M):
    """Yöne bağlı kalibrasyonla rüzgar: (wn, we) = M @ u. M 2x2 (kalibrasyonla bulunur).
    Drone'un eğilme tepkisi yöne göre farklıysa (gövde şekli, kontrolcü) tek katsayı çapraz
    rüzgarda ~0.5 m/s ve ~10° sapıyordu (2026-11-06)."""
    un, ue = tilt_feature(roll, pitch, yaw)
    return M[0][0] * un + M[0][1] * ue, M[1][0] * un + M[1][1] * ue


def fit_wind_matrix(samples):
    """samples: [((un, ue), (wn, we)), ...] -> en küçük kareler 2x2 M (rüzgar = M @ u)."""
    import numpy as np
    U = np.array([s[0] for s in samples], float)
    W = np.array([s[1] for s in samples], float)
    X, *_ = np.linalg.lstsq(U, W, rcond=None)      # U @ X = W  ->  M = X^T
    return X.T.tolist()


def calibrate_k(tilt_rad, wind_speed):
    """Bilinen rüzgar hızında ölçülen eğimden k_d (tan(eğim) = k_d v^2)."""
    return math.tan(tilt_rad) / (wind_speed ** 2)


# ---- açıya bağlı kalibrasyon (X gövde: 90°'de bir tekrarlayan tepki) ----------------------------------
# Ölçüm (2026-11-06, AirSim): drone doğu/kuzey rüzgarında 5 m/s'de ~1.0°, çapraz (45°) rüzgarda ~0.7° eğiliyor.
# Tek katsayı ve 2x2 matris (elips) bunu temsil edemez. Model, θ = rüzgarın GÖVDEYE göre yönü:
#   δ(θe)  = d0 + dc·cos4θe + ds·sin4θe          gerçek yön θ = eğimden çıkan yön θe + δ(θe)
#   k(θ)   = c0 + c4·cos4θ + s4·sin4θ            tan(eğim) = k(θ)·v²   (gerçek yönde)

def _harm_rows(thetas):
    import numpy as np
    t = np.asarray(thetas, float)
    return np.column_stack([np.ones_like(t), np.cos(4 * t), np.sin(4 * t)])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def fit_wind_harmonic(rows):
    """rows: [(roll, pitch, yaw, wn, we), ...] bilinen rüzgarlarda ortalama yönelim -> parametre sözlüğü."""
    import numpy as np
    th_t, ks, deltas = [], [], []
    for r, p, y, wn, we in rows:
        n, e, tilt = tilt_vector_ne(r, p, y)
        th_tilt = _wrap(math.atan2(-e, -n) - y)            # eğimden çıkan rüzgar yönü, gövdeye göre
        th_true = _wrap(math.atan2(we, wn) - y)
        th_t.append(th_tilt)
        ks.append(math.tan(tilt) / (wn * wn + we * we))
        deltas.append(_wrap(th_true - th_tilt))
    cd, *_ = np.linalg.lstsq(_harm_rows(th_t), np.array(deltas), rcond=None)        # δ: gözlenen yöne göre
    th_true = [_wrap(t + d) for t, d in zip(th_t, deltas)]
    ck, *_ = np.linalg.lstsq(_harm_rows(th_true), np.array(ks), rcond=None)         # k: gerçek yöne göre
    return {"k": [float(x) for x in ck], "d": [float(x) for x in cd]}


def wind_from_tilt_harmonic(roll, pitch, yaw, prm):
    """Açıya bağlı kalibrasyonla rüzgar (wn, we)."""
    n, e, tilt = tilt_vector_ne(roll, pitch, yaw)
    if math.hypot(n, e) < 1e-6 or tilt < 1e-4:
        return 0.0, 0.0
    th = _wrap(math.atan2(-e, -n) - yaw)
    th += float(sum(a * b for a, b in zip(prm["d"], _harm_rows([th])[0])))         # önce yönü düzelt
    k = max(1e-6, float(sum(a * b for a, b in zip(prm["k"], _harm_rows([th])[0]))))  # sonra o yöndeki k
    v = math.sqrt(math.tan(tilt) / k)
    return v * math.cos(th + yaw), v * math.sin(th + yaw)


# ---- tablo (look-up) kalibrasyonu: sık ölçüm + doğrusal ara değer --------------------------------------
# Kalibrasyon (2026-11-06) sapmanın 180°'de bir tekrarladığını ama düzgün olmadığını gösterdi (60°: -14°,
# 90°: +6°). Harmonik model ölçüm aralarında yanlış tahmin etti (315°'de ~27° hata, 0/3 isabet). Tablo:
# gövde açısına göre (θe, eğimden çıkan yön) sapma δ ve katsayı k; ara açılar komşulardan doğrusal.

def fit_wind_lut(rows, period=180.0):
    """rows: [(roll, pitch, yaw, wn, we), ...] -> {"period", "theta_e" (derece, artan), "delta" (derece), "k"}."""
    pts = []
    for r, p, y, wn, we in rows:
        n, e, tilt = tilt_vector_ne(r, p, y)
        th_e = math.degrees(_wrap(math.atan2(-e, -n) - y)) % period
        th_t = math.degrees(_wrap(math.atan2(we, wn) - y))
        th_obs = math.degrees(_wrap(math.atan2(-e, -n) - y))
        delta = (th_t - th_obs + 180.0) % 360.0 - 180.0
        pts.append((th_e, delta, math.tan(tilt) / (wn * wn + we * we)))
    pts.sort()
    merged = []                                     # aynı açıdaki ölçümleri (ör. 5 ve 8 m/s) ortala
    for th, dl, k in pts:
        if merged and abs(th - merged[-1][0]) < 1.0:
            m = merged[-1]
            c = m[3] + 1
            merged[-1] = ((m[0] * m[3] + th) / c, (m[1] * m[3] + dl) / c, (m[2] * m[3] + k) / c, c)
        else:
            merged.append((th, dl, k, 1))
    return {"period": period, "theta_e": [m[0] for m in merged], "delta": [m[1] for m in merged],
            "k": [m[2] for m in merged]}


def _lut_interp(lut, th):
    P = lut["period"]
    xs, n = lut["theta_e"], len(lut["theta_e"])
    th %= P
    for i in range(n):
        x0, x1 = xs[i - 1], xs[i]
        lo = x0 - P if i == 0 else x0               # dairesel: son nokta ilk noktaya bağlanır
        if lo <= th <= x1 or (i == 0 and th >= x0):
            t_ = th if th <= x1 else th - P
            w = (t_ - lo) / (x1 - lo) if x1 != lo else 0.0
            return ((1 - w) * lut["delta"][i - 1] + w * lut["delta"][i],
                    (1 - w) * lut["k"][i - 1] + w * lut["k"][i])
    return lut["delta"][-1], lut["k"][-1]


def wind_from_tilt_lut(roll, pitch, yaw, lut):
    """Tablo kalibrasyonuyla rüzgar (wn, we)."""
    n, e, tilt = tilt_vector_ne(roll, pitch, yaw)
    if math.hypot(n, e) < 1e-6 or tilt < 1e-4:
        return 0.0, 0.0
    th_obs = math.degrees(_wrap(math.atan2(-e, -n) - yaw))
    delta, k = _lut_interp(lut, th_obs)
    th = math.radians(th_obs + delta) + yaw
    v = math.sqrt(math.tan(tilt) / max(k, 1e-6))
    return v * math.cos(th), v * math.sin(th)
