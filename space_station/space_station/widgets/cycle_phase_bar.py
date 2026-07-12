"""CyclePhaseBar: the 4BMS half-cycle timeline.

A horizontal bar split into three proportional segments:
    AIR-SAVE (10 min) | ADSORB-DESORB (60 min) | VACUUM (10 min)
with uppercase labels centered in each segment and an optional moving marker
showing the current position within the half-cycle.
"""

from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QPen, QFont
from PyQt5.QtCore import Qt, QRectF

from space_station import theme

SEGMENTS = [
    ("AIR-SAVE", 10.0),
    ("ADSORB-DESORB", 60.0),
    ("VACUUM", 10.0),
]


class CyclePhaseBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._total = sum(s[1] for s in SEGMENTS)  # 80 min
        self._elapsed = 0.0
        self._has_marker = False
        self.setMinimumHeight(46)

    def set_phase(self, elapsed_min, total_min=None):
        if total_min:
            self._total = float(total_min)
        self._elapsed = max(0.0, min(self._total, float(elapsed_min)))
        self._has_marker = True
        self.update()

    def clear_marker(self):
        self._has_marker = False
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        w = self.width()
        h = self.height()
        bar = QRectF(0.5, 0.5, w - 1, h - 1)

        line = QColor(theme.color("line2"))
        panel = QColor(theme.color("panel2"))
        txt = QColor(theme.color("txt2"))

        # Outer border
        p.setPen(QPen(line, 1))
        p.setBrush(panel)
        p.drawRoundedRect(bar, 6, 6)

        f = QFont(theme.FONT_SANS, 8)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 1.5)
        p.setFont(f)

        # Segments
        x = bar.left()
        for i, (label, span) in enumerate(SEGMENTS):
            seg_w = (span / self._total) * bar.width()
            seg_rect = QRectF(x, bar.top(), seg_w, bar.height())
            if i > 0:
                p.setPen(QPen(line, 1))
                p.drawLine(int(x), int(bar.top() + 4),
                           int(x), int(bar.bottom() - 4))
            p.setPen(txt)
            p.drawText(seg_rect, Qt.AlignCenter, label)
            x += seg_w

        # Marker
        if self._has_marker and self._total > 0:
            mx = bar.left() + (self._elapsed / self._total) * bar.width()
            p.setPen(QPen(QColor(theme.color("accent")), 2))
            p.drawLine(int(mx), int(bar.top() + 2),
                       int(mx), int(bar.bottom() - 2))

        p.end()
