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


def calibrate_k(tilt_rad, wind_speed):
    """Bilinen rüzgar hızında ölçülen eğimden k_d (tan(eğim) = k_d v^2)."""
    return math.tan(tilt_rad) / (wind_speed ** 2)
