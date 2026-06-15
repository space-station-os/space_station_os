"""SubsystemRoster: a right-sidebar list of subsystems with a status dot,
name, and status word. set_status(name, status) updates one row.

status is one of: nominal | degraded | fault | offline (mapped to colors).
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt5.QtCore import Qt

from space_station import theme

_STATUS_COLOR = {
    "nominal": "green",
    "ok": "green",
    "active": "green",
    "degraded": "amber",
    "warning": "amber",
    "fault": "red",
    "failure": "red",
    "critical": "red",
    "offline": "txt3",
    "unknown": "txt3",
    "stale": "txt3",
}


class _RosterRow(QWidget):
    def __init__(self, name, parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(14, 7, 14, 7)
        row.setSpacing(8)

        self._dot = QLabel()
        self._dot.setFixedSize(8, 8)
        self._set_dot("txt3")
        row.addWidget(self._dot, 0, Qt.AlignVCenter)

        self._name = QLabel(name.upper())
        self._name.setFont(theme.label_font(9, tracking=1.5))
        self._name.setStyleSheet(f"color: {theme.color('txt2')};")
        row.addWidget(self._name)
        row.addStretch()

        self._status = QLabel("—")
        self._status.setFont(theme.mono_font(9))
        self._status.setStyleSheet(f"color: {theme.color('txt3')};")
        row.addWidget(self._status)

    def _set_dot(self, color_key):
        self._dot.setStyleSheet(
            f"background:{theme.color(color_key)}; border-radius:4px;"
        )

    def set_status(self, status):
        key = (status or "unknown").lower()
        color_key = _STATUS_COLOR.get(key, "txt3")
        self._set_dot(color_key)
        self._status.setText(status.upper())
        self._status.setStyleSheet(f"color: {theme.color(color_key)};")


class SubsystemRoster(QWidget):
    def __init__(self, names=None, parent=None):
        super().__init__(parent)
        names = names or ["ECLSS", "GNC", "EPS", "Thermal", "Comms"]

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        header = QLabel("SUBSYSTEMS")
        header.setProperty("class", "label")
        header.setFont(theme.label_font(9, tracking=2.0))
        header.setContentsMargins(14, 12, 14, 8)
        root.addWidget(header)

        self._rows = {}
        for name in names:
            row = _RosterRow(name)
            self._rows[name.upper()] = row
            root.addWidget(row)
            divider = QFrame()
            divider.setProperty("class", "hline")
            root.addWidget(divider)

    def set_status(self, name, status):
        row = self._rows.get(str(name).upper())
        if row is not None:
            row.set_status(status)
