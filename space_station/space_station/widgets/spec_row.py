"""SpecRow: a single bordered strip holding N MetricCards flush in a row,
separated by 1px hairline dividers (matches the mockup's spec strip)."""

from PyQt5.QtWidgets import QFrame, QHBoxLayout


class SpecRow(QFrame):
    def __init__(self, cards, parent=None):
        super().__init__(parent)
        self.setProperty("class", "flush")
        self.cards = list(cards)

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)

        for i, card in enumerate(self.cards):
            # Cards sit flush inside one outer border -> strip their own border.
            card.setProperty("class", "")
            card.setStyleSheet("QFrame { border: none; background: transparent; }")
            row.addWidget(card, 1)
            if i < len(self.cards) - 1:
                divider = QFrame()
                divider.setProperty("class", "vline")
                divider.setFixedWidth(1)
                row.addWidget(divider)
