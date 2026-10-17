"""Paket bırakıcı arayüzü: release() paketi bırakır (ipli olanlarda yere indirip bırakır).

    SimPayload      -> AirSim'de drone'un altında fizikli bir kutu oluşturur (serbest düşüş)
    TetherPayload   -> AirSim'de paketi ip ucunda yavaşça (0.5 m/s) yere indirir (vinç)
    GripperPayload  -> PX4'e MAVLink gripper "bırak" komutu (gerçek drone; servo/elektromıknatıs)
    WinchPayload    -> MAVLink vinç "teslim et" komutu (gerçek drone; ipli indirme)

Gerçek drone'da PX4 tarafında tutucunun tanımlı olması gerekir (PD_GRIPPER_EN=1 ve
uygun çıkış/servo ataması); komut aynıdır.
"""
from drop_test import PACKAGE_SIZE


class SimPayload:
    def __init__(self, client):
        from sim_client import airsim
        self.airsim = airsim
        self.c = client
        self.count = 0

    def release(self):
        """Paketi drone'un 0.7 m altında fizikli olarak oluşturur; nesne adını döndürür."""
        p = self.c.simGetVehiclePose().position  # AirSim koordinatı (PX4 yerel orijininden bağımsız)
        self.count += 1   # her pakete ayrı ad: silinen bir adla hemen yeniden oluşturmak Unreal'i çökertiyor
        return self.c.simSpawnObject(
            f"Package_{self.count}", "SM_Package",
            self.airsim.Pose(self.airsim.Vector3r(p.x_val, p.y_val, p.z_val + 0.7), self.airsim.Quaternionr()),
            self.airsim.Vector3r(*PACKAGE_SIZE), physics_enabled=True)


class GripperPayload:
    def __init__(self, px4_vehicle, instance=1):
        self.v = px4_vehicle
        self.instance = instance

    def release(self):
        self.v._run(self.v.drone.gripper.release(self.instance), timeout=5)
        return None


class TetherPayload(SimPayload):
    """İpli indirme (Wing tarzı vinç): paket drone'un altından SPEED m/s ile yere iner.

    Fizik kapalı oluşturulup konumu adım adım indirilir (ip gerilimi altında sallanmayan yük);
    yere değince kanca bırakır. Kişiye uzatılan yardım paketinde çarpma hızı ~SPEED olur.
    """
    SPEED = 0.5

    def __init__(self, client, ground_z):
        super().__init__(client)
        self.ground_z = ground_z
        self.last_speed = None
        self.line_m = None

    def release(self):
        import time
        p = self.c.simGetVehiclePose().position
        self.count += 1
        name = self.c.simSpawnObject(
            f"Package_{self.count}", "SM_Package",
            self.airsim.Pose(self.airsim.Vector3r(p.x_val, p.y_val, p.z_val + 0.7), self.airsim.Quaternionr()),
            self.airsim.Vector3r(*PACKAGE_SIZE), physics_enabled=False)
        z, z_end = p.z_val + 0.7, self.ground_z - PACKAGE_SIZE[2] / 2
        dt = 0.05
        while z < z_end:
            z = min(z_end, z + self.SPEED * dt)
            self.c.simSetObjectPose(name, self.airsim.Pose(self.airsim.Vector3r(p.x_val, p.y_val, z),
                                                           self.airsim.Quaternionr()), True)
            time.sleep(dt)
        self.last_speed = self.SPEED
        self.line_m = round(z_end - p.z_val, 2)
        return name


class WinchPayload:
    """Gerçek drone: MAVLink vinç (MAVSDK winch.deliver: ipi salar, yere değince bırakır, sarar).

    Not: vinç desteği otopilota bağlı (ArduPilot'ta yerleşik; PX4'te payload_deliverer +
    uygun donanım/sürücü gerekir). Uçuştan önce yerde, pervanesiz denenmeli.
    """

    def __init__(self, px4_vehicle, instance=1):
        self.v = px4_vehicle
        self.instance = instance

    def release(self):
        self.v._run(self.v.drone.winch.deliver(self.instance), timeout=180)
        return None


class DragPayload(SimPayload):
    """Serbest düşüş + hava direnci + rüzgar (AirSim'in fiziği pakete rüzgar uygulamıyor).

    Paket fizik kapalı oluşturulur, konumu ballistics.py ile gerçek zamanlı adım adım sürülür;
    zemine (ya da bir aracın tavanına) değince durur. wind_ne: simülasyondaki rüzgar (m/s).
    car_tops: [(isim, tavan_z_ned)] — tavana çarpma kontrolü için (isteğe bağlı).
    """

    def __init__(self, client, ground_z, wind_ne=(0.0, 0.0), car_hit=None):
        super().__init__(client)
        self.ground_z = ground_z
        self.wind = wind_ne
        self.car_hit = car_hit          # car_hit(x, y, z) -> True ise o noktada araca çarptı
        self.last_speed = None
        self.impact = None

    def release(self):
        import math
        import time

        import ballistics as b
        pose = self.c.simGetVehiclePose().position
        vel = self.c.getMultirotorState().kinematics_estimated.linear_velocity
        self.count += 1
        name = self.c.simSpawnObject(
            f"Package_{self.count}", "SM_Package",
            self.airsim.Pose(self.airsim.Vector3r(pose.x_val, pose.y_val, pose.z_val + 0.7), self.airsim.Quaternionr()),
            self.airsim.Vector3r(*PACKAGE_SIZE), physics_enabled=False)
        n, e, d = pose.x_val, pose.y_val, pose.z_val + 0.7
        vn, ve, vd = vel.x_val, vel.y_val, vel.z_val
        k = 0.5 * b.RHO * b.PACKAGE_CD_A / b.PACKAGE_MASS_KG
        wn, we = self.wind
        z_end = self.ground_z - PACKAGE_SIZE[2] / 2
        dt, t_prev = 0.005, time.time()
        while True:
            now = time.time()
            steps = max(1, int((now - t_prev) / dt))   # gerçek zamanla eşit ilerle
            t_prev = now
            hit = False
            for _ in range(steps):
                rn, re, rd = vn - wn, ve - we, vd
                s = math.sqrt(rn * rn + re * re + rd * rd)
                vn -= k * s * rn * dt
                ve -= k * s * re * dt
                vd += (b.G - k * s * rd) * dt
                n, e, d = n + vn * dt, e + ve * dt, d + vd * dt
                if d >= z_end or (self.car_hit is not None and self.car_hit(n, e, d)):
                    hit = True
                    break
            self.c.simSetObjectPose(name, self.airsim.Pose(self.airsim.Vector3r(n, e, min(d, z_end)),
                                                           self.airsim.Quaternionr()), True)
            if hit:
                break
            time.sleep(0.02)
        self.impact = (n, e, min(d, z_end))
        self.last_speed = round(math.sqrt(vn * vn + ve * ve + vd * vd), 1)
        return name
