"""Araç arayüzü: görev kodu uçuşu sadece buradan yönetir, böylece aynı kod simülasyonda ve
gerçek drone'da çalışır.

    AirSimVehicle  -> AirSim'in kendi basit uçuş kontrolcüsü (SimpleFlight)
    Px4Vehicle     -> PX4 otopilotu, MAVSDK/MAVLink üzerinden (SITL'de de gerçek drone'da da aynı)

Bütün konumlar yerel NED (metre): n = kuzey, e = doğu, d = aşağı (yukarısı negatif).
Orijin: kalkış noktası. Hızlar m/s. Gövde ekseni: x ileri, y sağ, z aşağı.

Ortak arayüz:
    arm_takeoff(alt_m)                    kalk ve alt_m yüksekliğe çık (bekler)
    goto(n, e, d, speed, tol=1.0)         noktaya git (bekler)
    start_goto(n, e, d, speed)            noktaya gitmeye başla (beklemez)
    velocity_body(vx, vy, vz)             gövde ekseninde hız komutu (yenilenene kadar sürer)
    velocity_ned(vn, ve, vd)              dünya ekseninde hız komutu (eğimden etkilenmez)
    attitude() -> (roll, pitch, yaw)      radyan; kamera eğim telafisi için
    height_agl() -> m                     zeminden yükseklik (PX4: aşağı bakan mesafe sensörü varsa
                                          ondan, yoksa kalkış noktasına göre — düz arazi varsayımı)
    battery() -> 0..1 ya da None          kalan pil oranı
    hover()                               olduğu yerde dur
    land()                                in, motorları kapat (bekler)
    position() -> (n, e, d)
    velocity() -> (vn, ve, vd)
    wait_still(max_speed=0.2, timeout=10)
"""
import asyncio
import math
import threading
import time


RANGE_MAX_AGE_S = 0.5   # bundan eski mesafe sensörü ölçümü kullanılmaz


def range_to_agl(r, r_min, r_max, roll, pitch):
    """Aşağı bakan mesafe sensörü okumasını (eğik ışın) dikey yüksekliğe çevirir; geçersizse None."""
    if r is None or not math.isfinite(r) or not (r_min < r < r_max):
        return None   # menzil dışı: ölçüm yok ya da güvenilmez
    if abs(roll) > math.radians(30) or abs(pitch) > math.radians(30):
        return None   # çok eğik: ışın zemine değil yana bakıyor olabilir
    return r * math.cos(roll) * math.cos(pitch)


def quat_to_euler(w, x, y, z):
    """NED gövde kuaterniyonu -> (roll, pitch, yaw), radyan (ZYX / Tait-Bryan)."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Vehicle:
    def wait_still(self, max_speed=0.2, timeout=10.0):
        self.hover()
        t0 = time.time()
        while time.time() - t0 < timeout:
            vn, ve, _ = self.velocity()
            if math.hypot(vn, ve) < max_speed:
                return True
            time.sleep(0.1)
        return False

    def distance_to(self, n, e, d=None):
        pn, pe, pd = self.position()
        return math.dist((pn, pe) if d is None else (pn, pe, pd), (n, e) if d is None else (n, e, d))


# --------------------------------------------------------------------------------------------
class AirSimVehicle(Vehicle):
    """AirSim SimpleFlight. Kamera ve sahne için aynı istemci (client) kullanılabilir."""

    def __init__(self, client, ground_z=0.0):
        """ground_z: zeminin AirSim NED z'si. AirSim orijini PlayerStart (zeminin ~0.9 m üstü);
        bu sınıf d'yi zemine göre verir/alır ki PX4 (orijin = yerdeki kalkış noktası) ile aynı olsun."""
        from sim_client import airsim
        self.airsim = airsim
        self.c = client
        self.ground_z = ground_z

    def arm_takeoff(self, alt_m):
        self.c.enableApiControl(True)
        self.c.armDisarm(True)
        self.c.takeoffAsync().join()
        self.c.moveToZAsync(-alt_m + self.ground_z, 3).join()

    def goto(self, n, e, d, speed, tol=1.0):
        self.c.moveToPositionAsync(n, e, d + self.ground_z, speed).join()

    def start_goto(self, n, e, d, speed):
        self.c.moveToPositionAsync(n, e, d + self.ground_z, speed)

    def velocity_body(self, vx, vy, vz):
        self.c.moveByVelocityBodyFrameAsync(vx, vy, vz, 0.5)

    def velocity_ned(self, vn, ve, vd):
        self.c.moveByVelocityAsync(vn, ve, vd, 0.5)

    def attitude(self):
        q = self.c.getMultirotorState().kinematics_estimated.orientation
        return quat_to_euler(q.w_val, q.x_val, q.y_val, q.z_val)

    def hover(self):
        self.c.hoverAsync()

    def land(self):
        self.c.landAsync().join()
        t0 = time.time()
        while self.c.getMultirotorState().landed_state != self.airsim.LandedState.Landed and time.time() - t0 < 15:
            time.sleep(0.2)
        time.sleep(1.0)
        self.c.armDisarm(False)

    def position(self):
        p = self.c.getMultirotorState().kinematics_estimated.position
        return p.x_val, p.y_val, p.z_val - self.ground_z

    def velocity(self):
        v = self.c.getMultirotorState().kinematics_estimated.linear_velocity
        return v.x_val, v.y_val, v.z_val

    def height_agl(self):
        return -self.position()[2]

    def battery(self):
        return None  # SimpleFlight pil modellemiyor

    def release(self):
        self.c.enableApiControl(False)


# --------------------------------------------------------------------------------------------
class Px4Vehicle(Vehicle):
    """PX4 + MAVSDK (offboard). SITL'de udp://:14540, gerçek drone'da seri port/telemetri.

    MAVSDK asenkron; burada arka planda bir olay döngüsü (event loop) çalışır ve her komut
    senkron bir fonksiyon olarak sunulur, görev kodu asyncio bilmek zorunda kalmaz.
    Offboard modunda MAVSDK son hedefi otopilota sürekli (20 Hz) yeniden gönderir.
    """

    def __init__(self, address="udp://:14550", connect_timeout=180):
        from mavsdk import System
        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()
        self.drone = System()
        self._pos = (0.0, 0.0, 0.0)
        self._vel = (0.0, 0.0, 0.0)
        self._in_air = False
        self._att = (0.0, 0.0, 0.0)
        self._got_pos = False
        self._battery = None
        self._range = None        # (okuma_m, min_m, max_m, zaman)
        self._goto_task = None
        self._run(self._connect(address), timeout=connect_timeout)

    # -- yardımcılar ------------------------------------------------------------------------
    def _run(self, coro, timeout=None):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout)

    async def _connect(self, address):
        await self.drone.connect(system_address=address)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
        # simülatör yeni açıldıysa PX4'ün EKF'si / ev noktası 1-2 dk sürebiliyor
        print("  PX4 bağlandı, konum kestirimi ve ev noktası bekleniyor...", flush=True)

        async def healthy():
            async for h in self.drone.telemetry.health():
                if h.is_local_position_ok and h.is_home_position_ok:
                    return
        # MAVSDK'nin sağlık bayrakları önceki görevden hemen sonra dakikalarca False kalabiliyor
        # (PX4 o sırada sağlıklı ve konum akıyor). Bayraklar bilgi amaçlı; asıl kontrol PX4'ün
        # arm reddi (arm_takeoff tekrar dener). 30 s'de gelmezse konum akışıyla devam.
        try:
            await asyncio.wait_for(healthy(), 30)
        except asyncio.TimeoutError:
            print("  uyarı: sağlık bayrakları gelmedi, konum akışıyla devam ediliyor", flush=True)
        self._loop.create_task(self._watch_position())
        self._loop.create_task(self._watch_in_air())
        self._loop.create_task(self._watch_attitude())
        self._loop.create_task(self._watch_battery())
        self._loop.create_task(self._watch_range())
        while not self._got_pos:      # ilk telemetri gelmeden konum (0, 0, 0) görünmesin
            await asyncio.sleep(0.05)

    async def _watch_position(self):
        async for pv in self.drone.telemetry.position_velocity_ned():
            self._pos = (pv.position.north_m, pv.position.east_m, pv.position.down_m)
            self._vel = (pv.velocity.north_m_s, pv.velocity.east_m_s, pv.velocity.down_m_s)
            self._got_pos = True

    async def _watch_attitude(self):
        async for q in self.drone.telemetry.attitude_quaternion():
            self._att = quat_to_euler(q.w, q.x, q.y, q.z)

    async def _watch_battery(self):
        async for b in self.drone.telemetry.battery():
            self._battery = b.remaining_percent / 100.0 if b.remaining_percent > 1.0 else b.remaining_percent

    async def _watch_range(self):
        """Aşağı bakan lidar/mesafe sensörü (gerçek drone; SITL none_iris'te yok, akış boş kalır)."""
        try:
            async for d in self.drone.telemetry.distance_sensor():
                self._range = (d.current_distance_m, d.minimum_distance_m, d.maximum_distance_m, time.time())
        except Exception:
            self._range = None

    async def _watch_in_air(self):
        async for in_air in self.drone.telemetry.in_air():
            self._in_air = in_air

    async def _ensure_offboard(self):
        from mavsdk.offboard import OffboardError, VelocityNedYaw
        try:
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await self.drone.offboard.start()
        except OffboardError:
            pass  # zaten offboard

    def _cancel_goto(self):
        if self._goto_task is not None and not self._goto_task.done():
            self._loop.call_soon_threadsafe(self._goto_task.cancel)
        self._goto_task = None

    async def _goto(self, n, e, d, speed, tol):
        """Hedefe doğru hızla ilerleyen bir 'havuç' noktası gönderir: hız sınırlı, yumuşak."""
        from mavsdk.offboard import PositionNedYaw
        await self._ensure_offboard()
        while True:
            pn, pe, pd = self._pos
            dn, de, dd = n - pn, e - pe, d - pd
            dist = math.sqrt(dn * dn + de * de + dd * dd)
            if dist < tol:
                await self.drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
                return
            step = min(dist, speed * 1.0)  # 1 s ileride bir nokta
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(pn + dn / dist * step, pe + de / dist * step, pd + dd / dist * step, 0.0))
            await asyncio.sleep(0.1)

    # -- arayüz -----------------------------------------------------------------------------
    def arm_takeoff(self, alt_m):
        async def go():
            if self._in_air:  # önceki görevden havada kalmış: kalkışı atla, irtifaya çık
                return
            from mavsdk.action import ActionError
            await self.drone.action.set_takeoff_altitude(min(alt_m, 10.0))
            for attempt in range(30):   # PX4 hazır değilse arm'ı reddeder: bekleyip tekrar dene
                try:
                    await self.drone.action.arm()
                    break
                except ActionError as ex:
                    if attempt == 29:
                        raise
                    if attempt % 5 == 0:
                        print(f"  arm reddedildi ({ex}), tekrar deneniyor...", flush=True)
                    await asyncio.sleep(3)
            await self.drone.action.takeoff()
            while -self._pos[2] < min(alt_m, 10.0) * 0.9:
                await asyncio.sleep(0.2)
        self._run(go(), timeout=200)
        n, e, _ = self._pos
        self.goto(n, e, -alt_m, 3.0)

    def goto(self, n, e, d, speed, tol=1.0):
        self._cancel_goto()
        self._run(self._goto(n, e, d, speed, tol), timeout=600)

    def start_goto(self, n, e, d, speed):
        self._cancel_goto()
        self._goto_task = asyncio.run_coroutine_threadsafe(self._goto(n, e, d, speed, 0.5), self._loop)

    def velocity_body(self, vx, vy, vz):
        from mavsdk.offboard import VelocityBodyYawspeed
        self._cancel_goto()

        async def go():
            await self._ensure_offboard()
            await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
        self._run(go(), timeout=5)

    def velocity_ned(self, vn, ve, vd):
        from mavsdk.offboard import VelocityNedYaw
        self._cancel_goto()
        yaw_deg = math.degrees(self._att[2])

        async def go():
            await self._ensure_offboard()
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_deg))
        self._run(go(), timeout=5)

    def attitude(self):
        return self._att

    def hover(self):
        self.velocity_ned(0.0, 0.0, 0.0)

    def land(self):
        self._cancel_goto()

        async def go():
            from mavsdk.offboard import OffboardError
            try:
                await self.drone.offboard.stop()
            except OffboardError:
                pass
            await self.drone.action.land()
            while self._in_air:
                await asyncio.sleep(0.3)
            await asyncio.sleep(1.0)
            try:
                await self.drone.action.disarm()
            except Exception:
                pass  # PX4 inişte kendisi de disarm eder
        self._run(go(), timeout=180)

    def position(self):
        return self._pos

    def velocity(self):
        return self._vel

    def height_agl(self):
        if self._range is not None and time.time() - self._range[3] < RANGE_MAX_AGE_S:
            h = range_to_agl(*self._range[:3], *self._att[:2])
            if h is not None:
                return h
        return -self._pos[2]  # yedek: yerel orijin kalkış noktası (düz arazi varsayımı)

    def battery(self):
        return self._battery

    def release(self):
        pass
