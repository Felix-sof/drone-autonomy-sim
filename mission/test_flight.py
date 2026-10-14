"""İlk test: bağlan, kalk, 10 m yüksel, yumuşak bir kare çiz, yavaşça in."""
from sim_client import airsim, connect, safe_land, wait_until_still

client = connect()
print("Bağlandı. Kalkış...")
client.takeoffAsync().join()
client.moveToZAsync(-10, 2).join()

# Noktalar tek bir yol olarak verilince köşelerde durup kalkmaz, akıcı döner.
path = [airsim.Vector3r(x, y, -10) for x, y in [(10, 0), (10, 10), (0, 10), (0, 0)]]
print("Kare çiziliyor...")
client.moveOnPathAsync(path, 3, 120, airsim.DrivetrainType.MaxDegreeOfFreedom,
                       airsim.YawMode(False, 0), -1, 1).join()
wait_until_still(client)

print("İniş...")
safe_land(client)
client.enableApiControl(False)
print("Test tamam!")
