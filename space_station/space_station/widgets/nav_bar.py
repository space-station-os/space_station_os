"""NavBar: a horizontal row of subsystem nav tabs.

Flat uppercase wide-tracked buttons. The active tab gets txt color + a 1px
accent underline; inactive tabs are txt3, hover -> txt2. Emits tab_changed(index)
to drive the content QStackedWidget.
"""

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton, QButtonGroup
from PyQt5.QtCore import pyqtSignal

from space_station import theme


class NavBar(QWidget):
    tab_changed = pyqtSignal(int)

    def __init__(self, tabs=None, parent=None):
        super().__init__(parent)
        self.setFixedHeight(48)
        self.setStyleSheet(
            f"NavBar {{ background-color: {theme.color('bg2')};"
            f" border-bottom: 1px solid {theme.color('line')}; }}"
        )

        tabs = tabs or ["Overview", "ECLSS", "GNC", "EPS", "Thermal", "Comms"]
        self._buttons = []

        row = QHBoxLayout(self)
        row.setContentsMargins(14, 0, 14, 0)
        row.setSpacing(4)

        self._group = QButtonGroup(self)
        self._group.setExclusive(True)

        for i, name in enumerate(tabs):
            btn = QPushButton(name.upper())
            btn.setProperty("class", "nav-tab")
            btn.setProperty("active", "false")
            btn.setCheckable(True)
            btn.setFont(theme.label_font(12, tracking=2.0))
            btn.setCursor(btn.cursor())
            btn.clicked.connect(lambda _checked, idx=i: self._on_click(idx))
            self._group.addButton(btn, i)
            self._buttons.append(btn)
            row.addWidget(btn)

        row.addStretch()
        self.set_active(0)

    def _on_click(self, idx):
        self.set_active(idx)
        self.tab_changed.emit(idx)

    def set_active(self, idx):
        for i, btn in enumerate(self._buttons):
            active = (i == idx)
            btn.setChecked(active)
            btn.setProperty("active", "true" if active else "false")
            btn.style().unpolish(btn)
            btn.style().polish(btn)
