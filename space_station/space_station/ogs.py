#!/usr/bin/env python3
"""OGS panel — Oxygen Generation System (PEM water electrolysis).

Subscribes to the ssos_eclss OGS telemetry and degrades gracefully (shows "—")
when topics are absent:
    /ssos/ogs/o2_kg_day      std_msgs/Float64
    /ssos/ogs/diagnostics    diagnostic_msgs/DiagnosticArray (name="ogs")
        keys: o2_kg_day, h2_mol_s, stack_voltage, stack_power_w, stack_temp_k
"""
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtCore import QTimer

from std_msgs.msg import Float64
try:
    from diagnostic_msgs.msg import DiagnosticArray
    _HAVE_DIAG = True
except Exception:
    DiagnosticArray = None
    _HAVE_DIAG = False

from space_station.widgets import MetricCard, SpecRow, MeterBar, page_header

O2_REQUIRED_KG_DAY = 2.3
OGA_MAX_KG_DAY = 9.25  # AOGA max (46.9 A)


def k_to_c(k):
    return k - 273.15


class OGSWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self._o2 = None
        self._h2 = None
        self._v = None
        self._p = None
        self._t_k = None
        self._build_ui()
        self._init_ros()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(1000)

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(16)
        root.addWidget(page_header(
            "Oxygen Generation System", "PEM Water Electrolysis · OGA", "OGS"))

        self.card_o2 = MetricCard("O₂ Production", "—", "kg/day", "NO DATA", "muted")
        self.card_h2 = MetricCard("H₂ to Sabatier", "—", "mol/s", "NO DATA", "muted")
        self.card_v = MetricCard("Stack Voltage", "—", "V", "NO DATA", "muted")
        self.card_p = MetricCard("Stack Power", "—", "W", "NO DATA", "muted")
        self.card_t = MetricCard("Stack Temp", "—", "°C", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_o2, self.card_h2, self.card_v,
                                self.card_p, self.card_t]))

        bars = QFrame()
        bars.setProperty("class", "card")
        bl = QHBoxLayout(bars)
        bl.setContentsMargins(18, 16, 18, 16)
        bl.setSpacing(24)
        self.meter_o2 = MeterBar("O₂ Output", "kg/day", "accent")
        self.meter_t = MeterBar("Stack Temp", "°C", "amber")
        bl.addWidget(self.meter_o2, 1)
        bl.addWidget(self.meter_t, 1)
        root.addWidget(bars)
        root.addStretch()

    def _init_ros(self):
        try:
            self.node.create_subscription(
                Float64, "/ssos/ogs/o2_kg_day", self._on_o2, 10)
            if _HAVE_DIAG:
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/ogs/diagnostics", self._on_diag, 10)
        except Exception as e:
            self.node.get_logger().warn(f"[ogs] subs failed: {e}")

    def _on_o2(self, msg):
        self._o2 = msg.data

    def _on_diag(self, msg):
        for st in msg.status:
            for kv in st.values:
                try:
                    val = float(kv.value)
                except (TypeError, ValueError):
                    continue
                if kv.key == "o2_kg_day" and self._o2 is None:
                    self._o2 = val
                elif kv.key == "h2_mol_s":
                    self._h2 = val
                elif kv.key == "stack_voltage":
                    self._v = val
                elif kv.key == "stack_power_w":
                    self._p = val
                elif kv.key == "stack_temp_k":
                    self._t_k = val

    def _refresh(self):
        if self._o2 is not None:
            ok = self._o2 >= O2_REQUIRED_KG_DAY
            self.card_o2.set_value(f"{self._o2:.2f}")
            self.card_o2.set_footer(
                f"REQ {O2_REQUIRED_KG_DAY:.1f} KG/DAY", "green" if ok else "amber")
            self.meter_o2.set_value(f"{self._o2:.2f}", self._o2 / OGA_MAX_KG_DAY)
        if self._h2 is not None:
            self.card_h2.set_value(f"{self._h2:.4f}")
            self.card_h2.set_footer("TO SABATIER", "green")
        if self._v is not None:
            self.card_v.set_value(f"{self._v:.1f}")
            self.card_v.set_footer("STACK", "green")
        if self._p is not None:
            self.card_p.set_value(f"{self._p:.0f}")
            self.card_p.set_footer("ELECTRICAL", "green")
        if self._t_k is not None:
            tc = k_to_c(self._t_k)
            self.card_t.set_value(f"{tc:.1f}")
            self.card_t.set_footer("CELL STACK", "green")
            self.meter_t.set_value(f"{tc:.1f}", tc / 80.0)
