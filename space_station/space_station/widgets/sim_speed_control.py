"""SimSpeedControl: a compact set of speed chips (1x / 5x / 15x / 30x / 60x)
that lets the operator accelerate sim time from the GUI. Emits speed_changed(x);
the shell forwards it to the simulation_controller's `time_scale` parameter.
"""

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import pyqtSignal

from space_station import theme


class SimSpeedControl(QWidget):
    speed_changed = pyqtSignal(float)
    SPEEDS = [1, 5, 15, 30, 60]

    def __init__(self, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(8, 0, 16, 0)
        row.setSpacing(6)

        lab = QLabel("SIM SPEED")
        lab.setProperty("class", "label")
        lab.setFont(theme.label_font(9, tracking=2.0))
        row.addWidget(lab)

        self._buttons = {}
        for s in self.SPEEDS:
            b = QPushButton(f"{s}x")
            b.setProperty("class", "nav-tab")
            b.setCursor(b.cursor())
            b.clicked.connect(lambda _checked=False, sp=s: self.set_speed(sp))
            self._buttons[s] = b
            row.addWidget(b)

        self.set_speed(1, emit=False)

    def set_speed(self, speed, emit=True):
        for sp, b in self._buttons.items():
            active = (sp == speed)
            b.setProperty("active", "true" if active else "false")
            b.style().unpolish(b)
            b.style().polish(b)
        if emit:
            self.speed_changed.emit(float(speed))
