"""BedLoadBar: a bordered vertical container with a fill from the bottom.

Fill height = loading fraction (0..1). Fill color is accent for an adsorbing
bed, amber for a desorbing bed. The displayed fill eases smoothly toward the
target each frame, so coarse/jumpy telemetry (e.g. under accelerated sim time)
animates continuously instead of snapping. Drawn with a custom paintEvent.
"""

from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QRectF, QTimer

from space_station import theme


class BedLoadBar(QWidget):
    def __init__(self, fraction=0.0, mode="ADS", parent=None):
        super().__init__(parent)
        self._target = max(0.0, min(1.0, fraction))
        self._fraction = self._target
        self._mode = mode
        self.setMinimumHeight(54)

        # Smooth animation toward the target (~30 Hz easing).
        self._anim = QTimer(self)
        self._anim.timeout.connect(self._ease)
        self._anim.start(33)

    def set_loading(self, fraction):
        self._target = max(0.0, min(1.0, float(fraction)))

    def set_mode(self, mode):
        self._mode = (mode or "ADS").upper()
        self.update()

    def _ease(self):
        # Exponential approach; snap when close enough to avoid idle repaints.
        diff = self._target - self._fraction
        if abs(diff) < 0.002:
            if self._fraction != self._target:
                self._fraction = self._target
                self.update()
            return
        self._fraction += diff * 0.18
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        rect = QRectF(0.5, 0.5, self.width() - 1, self.height() - 1)
        radius = 4.0

        # Track
        p.setPen(QPen(QColor(theme.color("line2")), 1))
        p.setBrush(QColor(theme.color("panel2")))
        p.drawRoundedRect(rect, radius, radius)

        # Fill from the bottom
        if self._fraction > 0:
            fill_h = (rect.height() - 4) * self._fraction
            fill_rect = QRectF(
                rect.left() + 2,
                rect.bottom() - 2 - fill_h,
                rect.width() - 4,
                fill_h,
            )
            fill_color = QColor(
                theme.color("amber") if self._mode in ("DES", "VAC")
                else theme.color("accent")
            )
            p.setPen(Qt.NoPen)
            p.setBrush(fill_color)
            p.drawRoundedRect(fill_rect, 3.0, 3.0)

        p.end()
