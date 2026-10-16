"""PX4 bağlantı testi: MAVSDK ile kalk, git, hız komutu, in. (Unreal Play + PX4 SITL açıkken)

Gerçek drone'da sadece adres değişir (ör. serial:///dev/ttyACM0:57600 veya udp://:14550).
"""
import math
import sys
import time

from vehicle import Px4Vehicle

addr = sys.argv[1] if len(sys.argv) > 1 else "udp://:14550"
print(f"PX4'e bağlanılıyor: {addr}")
v = Px4Vehicle(addr)
print("bağlandı, konum:", tuple(round(x, 2) for x in v.position()))

t = time.time()
v.arm_takeoff(15)
print(f"kalkış tamam ({time.time() - t:.0f} s), konum:", tuple(round(x, 2) for x in v.position()))

t = time.time()
v.goto(20, 10, -15, 5)
print(f"(20, 10)'a gidildi ({time.time() - t:.0f} s):", tuple(round(x, 2) for x in v.position()))

r, p, y = v.attitude()
print(f"yönelim: roll {math.degrees(r):.1f}°, pitch {math.degrees(p):.1f}°, yaw {math.degrees(y):.1f}°")

a = v.position()
v.velocity_ned(0, -2, 0)  # 3 s batıya
time.sleep(3)
v.wait_still()
b = v.position()
print(f"hız komutu (batıya 2 m/s, 3 s): Δn {b[0] - a[0]:+.2f}, Δe {b[1] - a[1]:+.2f}, Δd {b[2] - a[2]:+.2f}")

v.goto(0, 0, -15, 5)
t = time.time()
v.land()
print(f"iniş tamam ({time.time() - t:.0f} s), konum:", tuple(round(x, 2) for x in v.position()))
