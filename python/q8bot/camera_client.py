'''
카메라 클라이언트: XIAO ESP32-S3가 제공하는 MJPEG 스트림
(multipart/x-mixed-replace, boundary=frame — firmware/kangyangi/src/main.cpp 참조)을
표준 라이브러리(urllib)만으로 수신해 최신 JPEG 프레임(pygame.Surface)만 유지한다.
GUI를 블로킹하지 않도록 별도 스레드에서 동작하고, 연결 실패 시 자동 재시도한다.
'''

import io
import threading
import time
import urllib.request

import pygame


class CameraClient:
    """백그라운드 스레드에서 MJPEG 스트림을 읽어 최신 프레임만 보관."""

    RETRY_INTERVAL = 2.0  # 연결 실패 시 재시도 간격(초)
    READ_TIMEOUT = 5

    def __init__(self, url="http://192.168.4.1/"):
        self.url = url
        self._surface = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_frame(self):
        """최신 프레임(pygame.Surface) 반환. 아직 없으면 None."""
        with self._lock:
            return self._surface

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            try:
                self._stream_once()
            except Exception:
                pass  # 연결/파싱 실패 시 재시도로 처리
            if self._stop.is_set():
                break
            time.sleep(self.RETRY_INTERVAL)

    def _stream_once(self):
        with urllib.request.urlopen(self.url, timeout=self.READ_TIMEOUT) as resp:
            buf = b""
            while not self._stop.is_set():
                chunk = resp.read(4096)
                if not chunk:
                    return
                buf += chunk
                buf = self._extract_frames(buf)

    def _extract_frames(self, buf):
        """buf에서 완성된 JPEG 프레임을 모두 꺼내 최신 프레임으로 갱신하고
        아직 완성되지 않은 나머지 바이트를 반환."""
        while True:
            marker = buf.find(b"Content-Length:")
            if marker == -1:
                return buf
            line_end = buf.find(b"\r\n", marker)
            header_end = buf.find(b"\r\n\r\n", marker)
            if line_end == -1 or header_end == -1:
                return buf
            try:
                length = int(buf[marker:line_end].split(b":", 1)[1].strip())
            except (ValueError, IndexError):
                return buf
            frame_start = header_end + 4
            frame_end = frame_start + length
            if len(buf) < frame_end:
                return buf
            self._set_frame(buf[frame_start:frame_end])
            buf = buf[frame_end:]

    def _set_frame(self, jpeg_bytes):
        try:
            surface = pygame.image.load(io.BytesIO(jpeg_bytes))
        except Exception:
            return  # 손상된 프레임은 무시하고 이전 프레임 유지
        with self._lock:
            self._surface = surface
