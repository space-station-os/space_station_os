"""MeterBar: a labelled horizontal bar with an eased fill and a value readout.

Used for OGS / WRS live metrics so they animate like the ARS bed bars. The fill
eases smoothly toward its target each frame. set_value(value_text, fraction).
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QRectF, QTimer

from space_station import theme


class _Track(QWidget):
    def __init__(self, color_key="accent", parent=None):
        super().__init__(parent)
        self._target = 0.0
        self._fraction = 0.0
        self._color_key = color_key
        self.setMinimumHeight(10)
        self.setMaximumHeight(10)
        self._anim = QTimer(self)
        self._anim.timeout.connect(self._ease)
        self._anim.start(33)

    def set_fraction(self, f):
        self._target = max(0.0, min(1.0, float(f)))

    def set_color(self, color_key):
        self._color_key = color_key

    def _ease(self):
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
        p.setPen(QPen(QColor(theme.color("line2")), 1))
        p.setBrush(QColor(theme.color("panel2")))
        p.drawRoundedRect(rect, 4, 4)
        if self._fraction > 0:
            fw = (rect.width() - 4) * self._fraction
            fr = QRectF(rect.left() + 2, rect.top() + 2, fw, rect.height() - 4)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(theme.color(self._color_key)))
            p.drawRoundedRect(fr, 3, 3)
        p.end()


class MeterBar(QWidget):
    def __init__(self, label, unit="", color_key="accent", parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(4)

        head = QHBoxLayout()
        head.setContentsMargins(0, 0, 0, 0)
        self._label = QLabel(label.upper())
        self._label.setProperty("class", "label")
        self._label.setFont(theme.label_font(10, tracking=2.0))
        head.addWidget(self._label)
        head.addStretch()
        self._value = QLabel("—")
        self._value.setProperty("class", "value")
        self._value.setFont(theme.mono_font(14))
        head.addWidget(self._value)
        self._unit = unit
        v.addLayout(head)

        self._track = _Track(color_key)
        v.addWidget(self._track)

    def set_value(self, value_text, fraction, color_key=None):
        suffix = f" {self._unit}" if self._unit else ""
        self._value.setText(f"{value_text}{suffix}")
        if color_key:
            self._track.set_color(color_key)
        self._track.set_fraction(fraction)
