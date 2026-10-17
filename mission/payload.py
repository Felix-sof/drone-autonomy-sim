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
