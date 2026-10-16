"""Paket bırakıcı arayüzü: release() paketi bırakır.

    SimPayload      -> AirSim'de drone'un altında fizikli bir kutu oluşturur (simülasyon)
    GripperPayload  -> PX4'e MAVLink gripper "bırak" komutu (gerçek drone; servo/elektromıknatıs)

Gerçek drone'da PX4 tarafında tutucunun tanımlı olması gerekir (PD_GRIPPER_EN=1 ve
uygun çıkış/servo ataması); komut aynıdır.
"""
from drop_test import PACKAGE_SIZE


class SimPayload:
    def __init__(self, client):
        from sim_client import airsim
        self.airsim = airsim
        self.c = client

    def release(self):
        """Paketi drone'un 0.7 m altında fizikli olarak oluşturur; nesne adını döndürür."""
        p = self.c.simGetVehiclePose().position  # AirSim koordinatı (PX4 yerel orijininden bağımsız)
        return self.c.simSpawnObject(
            "Package", "SM_Package",
            self.airsim.Pose(self.airsim.Vector3r(p.x_val, p.y_val, p.z_val + 0.7), self.airsim.Quaternionr()),
            self.airsim.Vector3r(*PACKAGE_SIZE), physics_enabled=True)


class GripperPayload:
    def __init__(self, px4_vehicle, instance=1):
        self.v = px4_vehicle
        self.instance = instance

    def release(self):
        self.v._run(self.v.drone.gripper.release(self.instance), timeout=5)
        return None
