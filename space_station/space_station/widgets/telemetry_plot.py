"""TelemetryPlot: a themed pyqtgraph live line plot.

Dark background, hairline grid, accent trace with a subtle fill below, mono
axis ticks in txt3, optional dashed setpoint reference line. Keeps a rolling
buffer and scrolls as points are appended.
"""

from collections import deque

import pyqtgraph as pg
from PyQt5.QtGui import QFont

from space_station import theme


class TelemetryPlot(pg.PlotWidget):
    def __init__(self, max_points=240, y_range=None, setpoint=None,
                 x_label="", y_label="", parent=None):
        super().__init__(parent)
        self._max = max_points
        self._x = deque(maxlen=max_points)
        self._y = deque(maxlen=max_points)

        self.setBackground(theme.color("bg"))

        accent = theme.color("accent")
        line = theme.color("line")
        txt3 = theme.color("txt3")

        self.showGrid(x=True, y=True, alpha=0.15)
        self.getPlotItem().getViewBox().setDefaultPadding(0.02)

        # Axis styling
        tick_font = QFont(theme.FONT_MONO, 7)
        for ax_name in ("left", "bottom"):
            ax = self.getAxis(ax_name)
            ax.setPen(pg.mkPen(line, width=1))
            ax.setTextPen(pg.mkPen(txt3))
            ax.setTickFont(tick_font)
        if x_label:
            self.setLabel("bottom", x_label, color=txt3)
        if y_label:
            self.setLabel("left", y_label, color=txt3)

        # Trace with subtle fill below
        fill = pg.mkColor(accent)
        fill.setAlpha(30)
        self._curve = self.plot(
            pen=pg.mkPen(accent, width=2),
            fillLevel=(y_range[0] if y_range else 0),
            brush=fill,
        )

        if y_range is not None:
            self.setYRange(*y_range)
            self._y_range = y_range
        else:
            self._y_range = None

        # Setpoint reference line (dashed, txt3)
        self._setpoint_line = None
        if setpoint is not None:
            self.set_setpoint(setpoint)

        self.setMenuEnabled(False)
        self.hideButtons()

    def set_setpoint(self, value):
        if self._setpoint_line is not None:
            self.removeItem(self._setpoint_line)
        pen = pg.mkPen(theme.color("txt3"), width=1, style=pg.QtCore.Qt.DashLine)
        self._setpoint_line = self.addLine(y=value, pen=pen)

    def add_point(self, t, y):
        self._x.append(t)
        self._y.append(y)
        self._curve.setData(list(self._x), list(self._y))

    def set_data(self, xs, ys):
        self._x = deque(xs, maxlen=self._max)
        self._y = deque(ys, maxlen=self._max)
        self._curve.setData(list(self._x), list(self._y))

    def clear_data(self):
        self._x.clear()
        self._y.clear()
        self._curve.setData([], [])
