"""page_header: the shared subsystem page-header pattern.

Big title + uppercase wide-tracked subtitle on the left, a mono tag on the
right, and a bottom hairline border. Used by every subsystem panel so the
whole app shares one visual rhythm.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame
from PyQt5.QtCore import Qt

from space_station import theme


def page_header(title, subtitle, tag=""):
    wrap = QWidget()
    v = QVBoxLayout(wrap)
    v.setContentsMargins(0, 0, 0, 0)
    v.setSpacing(6)

    row = QHBoxLayout()
    left = QVBoxLayout()
    left.setSpacing(2)

    t = QLabel(title)
    t.setProperty("class", "title")
    t.setFont(theme.sans_font(24, bold=True))
    left.addWidget(t)

    s = QLabel(subtitle.upper())
    s.setProperty("class", "subtitle")
    s.setFont(theme.label_font(9, tracking=2.5))
    left.addWidget(s)
    row.addLayout(left)
    row.addStretch()

    if tag:
        tg = QLabel(tag)
        tg.setProperty("class", "tag")
        tg.setFont(theme.mono_font(9))
        row.addWidget(tg, 0, Qt.AlignTop)

    v.addLayout(row)

    hr = QFrame()
    hr.setProperty("class", "hline")
    v.addWidget(hr)
    return wrap
