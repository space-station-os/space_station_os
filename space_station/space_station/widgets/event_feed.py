"""EventFeed: a scrolling list of events/alerts (newest on top).

Each entry shows [SUBSYSTEM TAG] [time] on the top line and the message
below, separated by hairline borders. Warning/critical tags are colored.
"""

import datetime

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QScrollArea, QFrame
)
from PyQt5.QtCore import Qt

from space_station import theme

_SEV_COLOR = {
    "info": "txt3",
    "warning": "amber",
    "critical": "red",
    "emergency": "red",
}


class EventFeed(QWidget):
    def __init__(self, max_events=20, parent=None):
        super().__init__(parent)
        self._max = max_events

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        header = QLabel("EVENT FEED")
        header.setProperty("class", "label")
        header.setFont(theme.label_font(11, tracking=2.0))
        header.setContentsMargins(14, 12, 14, 8)
        root.addWidget(header)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self._container = QWidget()
        self._list = QVBoxLayout(self._container)
        self._list.setContentsMargins(0, 0, 0, 0)
        self._list.setSpacing(0)
        self._list.addStretch()
        self.scroll.setWidget(self._container)
        root.addWidget(self.scroll, 1)

    def add_event(self, subsystem, message, severity="info"):
        entry = self._make_entry(subsystem, message, severity)
        # insert at top (index 0), keep trailing stretch at the end
        self._list.insertWidget(0, entry)

        # trim
        while self._list.count() - 1 > self._max:
            item = self._list.takeAt(self._list.count() - 2)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _make_entry(self, subsystem, message, severity):
        frame = QFrame()
        frame.setStyleSheet(
            f"QFrame {{ border: none; border-bottom: 1px solid {theme.color('line')}; }}"
        )
        v = QVBoxLayout(frame)
        v.setContentsMargins(14, 8, 14, 8)
        v.setSpacing(2)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        tag = QLabel(str(subsystem).upper())
        tag.setFont(theme.label_font(10, tracking=1.5))
        sev_color = theme.color(_SEV_COLOR.get(severity, "txt3"))
        tag.setStyleSheet(f"color: {sev_color}; border: none;")
        top.addWidget(tag)
        top.addStretch()
        ts = QLabel(datetime.datetime.now().strftime("%H:%M:%S"))
        ts.setFont(theme.mono_font(10))
        ts.setStyleSheet(f"color: {theme.color('txt3')}; border: none;")
        top.addWidget(ts)
        v.addLayout(top)

        msg = QLabel(str(message))
        msg.setWordWrap(True)
        msg.setFont(theme.sans_font(12))
        msg.setStyleSheet(f"color: {theme.color('txt2')}; border: none;")
        v.addWidget(msg)
        return frame
