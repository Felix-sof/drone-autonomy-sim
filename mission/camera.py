"""Gerçek kamera: OpenCV ile USB kamera, video dosyası, RTSP ya da GStreamer hattı.

Arka planda sürekli okur, grab() her zaman EN SON kareyi verir. Görüntü işleme kameradan
yavaşsa eski kareler birikmez (aksi halde drone saniyeler önceki görüntüye göre hareket eder).

Örnek kaynaklar:
    0                                         ilk USB kamera
    "rtsp://192.168.144.25:8554/main.264"     IP kamera / gimbal yayını
    "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1280,height=720 ! nvvidconv ! "
    "video/x-raw,format=BGRx ! videoconvert ! appsink"   Jetson CSI kamera (GStreamer)
    "test.mp4"                                video dosyası (masa başı test)
"""
import threading
import time

import cv2


class OpenCVCamera:
    def __init__(self, source, width=1280, height=720, loop_file=True):
        src = int(source) if isinstance(source, str) and source.isdigit() else source
        api = cv2.CAP_GSTREAMER if isinstance(src, str) and "!" in src else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(src, api)
        if not self.cap.isOpened():
            raise RuntimeError(f"Kamera açılamadı: {source}")
        if isinstance(src, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.is_file = isinstance(src, str) and not src.startswith(("rtsp", "http")) and "!" not in src
        self.loop_file = loop_file
        self._frame, self._t = None, 0.0
        self._lock = threading.Lock()
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()
        t0 = time.time()
        while self._frame is None and time.time() - t0 < 10:
            time.sleep(0.05)
        if self._frame is None:
            raise RuntimeError("Kameradan kare gelmiyor")

    def _reader(self):
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30
        while not self._stop:
            ok, f = self.cap.read()
            if not ok:
                if self.is_file and self.loop_file:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                time.sleep(0.01)
                continue
            with self._lock:
                self._frame, self._t = f, time.time()
            if self.is_file:
                time.sleep(1.0 / fps)  # dosyayı gerçek zamanlı oynat

    def grab(self):
        with self._lock:
            return self._frame.copy()

    def age(self):
        """Son karenin yaşı (s): büyükse kamera takılmış demektir."""
        return time.time() - self._t

    def close(self):
        self._stop = True
        self.cap.release()
