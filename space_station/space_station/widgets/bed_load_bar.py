"""BedLoadBar: a bordered vertical container with a fill from the bottom.

Fill height = loading fraction (0..1). Fill color is accent for an adsorbing
bed, amber for a desorbing bed. Drawn with a custom paintEvent.
"""

from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QRectF

from space_station import theme


class BedLoadBar(QWidget):
    def __init__(self, fraction=0.0, mode="ADS", parent=None):
        super().__init__(parent)
        self._fraction = max(0.0, min(1.0, fraction))
        self._mode = mode
        self.setMinimumHeight(54)

    def set_loading(self, fraction):
        self._fraction = max(0.0, min(1.0, float(fraction)))
        self.update()

    def set_mode(self, mode):
        self._mode = (mode or "ADS").upper()
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
                theme.color("amber") if self._mode == "DES" else theme.color("accent")
            )
            p.setPen(Qt.NoPen)
            p.setBrush(fill_color)
            p.drawRoundedRect(fill_rect, 3.0, 3.0)

        p.end()
