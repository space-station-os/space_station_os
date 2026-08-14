"""MetricCard: a spec-sheet metric tile.

Layout (top to bottom):
    LABEL            (tiny uppercase, wide-tracked, txt3)
    123.4 unit       (big mono value + small unit suffix)
    footer status    (colored: green/amber/red/muted)
"""

from PyQt5.QtWidgets import QFrame, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt

from space_station import theme


class MetricCard(QFrame):
    def __init__(self, label, value="—", unit="", footer="", footer_color="muted",
                 parent=None):
        super().__init__(parent)
        self.setProperty("class", "card")
        self.setMinimumWidth(150)

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(8)

        # Label (uppercase wide-tracked)
        self.label = QLabel(str(label).upper())
        self.label.setProperty("class", "label")
        self.label.setFont(theme.label_font(10, tracking=2.0))
        root.addWidget(self.label)

        # Value + unit row
        value_row = QHBoxLayout()
        value_row.setSpacing(4)
        value_row.setContentsMargins(0, 0, 0, 0)

        self.value = QLabel(str(value))
        self.value.setProperty("class", "metric-value")
        self.value.setFont(theme.mono_font(24))
        value_row.addWidget(self.value, 0, Qt.AlignBottom)

        self.unit = QLabel(unit)
        self.unit.setProperty("class", "label")
        self.unit.setFont(theme.label_font(11, tracking=1.0))
        value_row.addWidget(self.unit, 0, Qt.AlignBottom)
        value_row.addStretch()
        root.addLayout(value_row)

        # Footer status line
        self.footer = QLabel(footer)
        self.footer.setFont(theme.sans_font(11))
        root.addWidget(self.footer)
        root.addStretch()

        self.set_footer(footer, footer_color)

    # ---- live updates ----
    def set_value(self, value, unit=None):
        self.value.setText(str(value))
        if unit is not None:
            self.unit.setText(unit)

    def set_footer(self, text, color="muted"):
        """color is one of: green, amber, red, muted."""
        self.footer.setText(text)
        cls = {
            "green": "footer-green",
            "amber": "footer-amber",
            "red": "footer-red",
            "muted": "footer-muted",
        }.get(color, "footer-muted")
        self.footer.setProperty("class", cls)
        # re-polish so the new dynamic-property QSS applies
        self.footer.style().unpolish(self.footer)
        self.footer.style().polish(self.footer)

    def set_no_data(self):
        self.value.setText("—")
        self.set_footer("NO DATA", "muted")
