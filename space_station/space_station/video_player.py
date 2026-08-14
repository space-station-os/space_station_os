"""Qt-based fullscreen video splash.

Frames are decoded with OpenCV's VideoCapture (always available) and shown in a
fullscreen Qt QLabel driven by a QTimer -- we do NOT use cv2.imshow, which is
absent in headless OpenCV builds and was silently failing. play() blocks (via a
local QEventLoop) until the clip finishes, so it can be used as an intro splash
before the main window and as an exit splash on close. Press Esc or click to
skip. Crash-proof: any failure just returns.
"""

import os

import cv2
from PyQt5.QtWidgets import QLabel, QApplication
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer, QEventLoop


class _SplashLabel(QLabel):
    def __init__(self, on_skip):
        super().__init__()
        self._on_skip = on_skip
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setStyleSheet("background-color: black;")
        self.setAlignment(Qt.AlignCenter)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self._on_skip()

    def mousePressEvent(self, event):
        self._on_skip()


class VideoPlayer:
    def __init__(self, video_path: str, on_finished_callback=None):
        self.video_path = video_path
        self.on_finished_callback = on_finished_callback

    def play(self):
        try:
            self._play()
        except Exception:
            pass
        if self.on_finished_callback:
            try:
                self.on_finished_callback()
            except Exception:
                pass

    def _play(self):
        if QApplication.instance() is None or not os.path.exists(self.video_path):
            return

        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            return

        fps = cap.get(cv2.CAP_PROP_FPS)
        if not fps or fps <= 0:
            fps = 30.0
        interval_ms = max(10, int(1000.0 / fps))

        loop = QEventLoop()
        done = {"v": False}

        def finish():
            if done["v"]:
                return
            done["v"] = True
            timer.stop()
            cap.release()
            label.close()
            loop.quit()

        label = _SplashLabel(finish)
        label.showFullScreen()

        def next_frame():
            ret, frame = cap.read()
            if not ret:
                finish()
                return
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            pix = QPixmap.fromImage(img).scaled(
                label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            label.setPixmap(pix)

        timer = QTimer()
        timer.timeout.connect(next_frame)
        timer.start(interval_ms)

        loop.exec_()
