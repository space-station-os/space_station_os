"""StatusBar: the top mission-control status bar.

Left:  3x3 dot-grid logo + "SPACE STATION OS" (uppercase, wide-tracked).
Right: a row of [LABEL / value] stat blocks separated by hairlines:
       STATION (state pill w/ colored dot), CREW, ORBIT, COMMS,
       MISSION ELAPSED (live mono clock, ticks at 1 Hz).
"""

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QFrame
from PyQt5.QtGui import QPainter, QColor
from PyQt5.QtCore import Qt, QTimer, QSize, QTime

from space_station import theme

_STATE_COLOR = {
    "INIT": "txt3",
    "NOMINAL": "green",
    "DEGRADED": "amber",
    "RECOVERY": "amber",
    "SAFE": "red",
}


class _DotGrid(QWidget):
    """A 3x3 grid of dots drawn via paintEvent (the SSOS mark)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(QSize(26, 26))

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setPen(Qt.NoPen)
        c = QColor(theme.color("txt"))
        r = 2.2
        step = 9
        x0 = 2
        y0 = 2
        for row in range(3):
            for col in range(3):
                # fade a couple dots for a subtle constellation feel
                c.setAlpha(255 if (row + col) % 2 == 0 else 130)
                p.setBrush(c)
                cx = x0 + col * step
                cy = y0 + row * step
                p.drawEllipse(int(cx), int(cy), int(r * 2), int(r * 2))
        p.end()


class _StatBlock(QWidget):
    """A [tiny uppercase label] / [value] stacked block."""

    def __init__(self, label, value="—", with_dot=False, parent=None):
        super().__init__(parent)
        v = QVBoxLayout(self)
        v.setContentsMargins(16, 0, 16, 0)
        v.setSpacing(2)

        self._label = QLabel(label.upper())
        self._label.setProperty("class", "label")
        self._label.setFont(theme.label_font(8, tracking=2.0))
        v.addWidget(self._label)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        self._dot = None
        if with_dot:
            self._dot = QLabel()
            self._dot.setFixedSize(8, 8)
            self._set_dot_color(theme.color("txt3"))
            row.addWidget(self._dot, 0, Qt.AlignVCenter)
        self._value = QLabel(value)
        self._value.setProperty("class", "value")
        self._value.setFont(theme.mono_font(12))
        row.addWidget(self._value)
        row.addStretch()
        v.addLayout(row)

    def _set_dot_color(self, hexcolor):
        if self._dot is not None:
            self._dot.setStyleSheet(
                f"background:{hexcolor}; border-radius:4px;"
            )

    def set_value(self, value):
        self._value.setText(str(value))

    def set_dot(self, hexcolor):
        self._set_dot_color(hexcolor)


class StatusBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(60)
        self.setStyleSheet(
            f"StatusBar {{ background-color: {theme.color('bg2')};"
            f" border-bottom: 1px solid {theme.color('line')}; }}"
        )

        root = QHBoxLayout(self)
        root.setContentsMargins(18, 0, 0, 0)
        root.setSpacing(12)

        # ---- Left: logo + wordmark ----
        root.addWidget(_DotGrid())
        word = QLabel("SPACE STATION OS")
        word.setFont(theme.label_font(12, tracking=3.0, bold=True))
        word.setStyleSheet(f"color: {theme.color('txt')};")
        root.addWidget(word)
        root.addStretch()

        # ---- Right: stat blocks ----
        self.station = _StatBlock("Station", "INIT", with_dot=True)
        self.crew = _StatBlock("Crew", "4")
        self.orbit = _StatBlock("Orbit", "—")
        self.comms = _StatBlock("Comms", "—")
        self.clock = _StatBlock("Mission Elapsed", "000/00:00:00")

        for i, block in enumerate(
            [self.station, self.crew, self.orbit, self.comms, self.clock]
        ):
            root.addWidget(self._divider())
            root.addWidget(block)

        # Mission elapsed clock (1 Hz)
        self._elapsed_secs = 0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(1000)

    def _divider(self):
        d = QFrame()
        d.setProperty("class", "vline")
        d.setFixedWidth(1)
        d.setFixedHeight(36)
        return d

    def _tick(self):
        self._elapsed_secs += 1
        days = self._elapsed_secs // 86400
        rem = self._elapsed_secs % 86400
        t = QTime(0, 0, 0).addSecs(rem)
        self.clock.set_value(f"{days:03d}/{t.toString('HH:mm:ss')}")

    # ---- public API ----
    def set_state(self, state_str):
        s = (state_str or "INIT").upper()
        self.station.set_value(s)
        self.station.set_dot(theme.color(_STATE_COLOR.get(s, "txt3")))

    def set_crew(self, n):
        self.crew.set_value(str(n))

    def set_orbit(self, text):
        self.orbit.set_value(str(text))

    def set_comms(self, text):
        self.comms.set_value(str(text))
