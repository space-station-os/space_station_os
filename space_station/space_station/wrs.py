#!/usr/bin/env python3
"""WRS panel — Water Recovery System (UPA + WPA).

Subscribes to the ssos_eclss WRS telemetry and degrades gracefully (shows "—")
when topics are absent:
    /ssos/wrs/potable_kg_day   std_msgs/Float64
    /ssos/wrs/diagnostics      diagnostic_msgs/DiagnosticArray (name="wrs")
        keys: potable_kg_day, conductivity_us, overall_recovery, voc_conversion
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

POTABLE_LIMIT_US = 100.0
POTABLE_NOMINAL_KG_DAY = 9.0


class WRSWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self._potable = None
        self._conduct = None
        self._recovery = None
        self._voc = None
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
            "Water Recovery System", "Urine Processor + Water Processor", "WRS"))

        self.card_potable = MetricCard("Potable Water", "—", "kg/day", "NO DATA", "muted")
        self.card_conduct = MetricCard("Conductivity", "—", "µS/cm", "NO DATA", "muted")
        self.card_recovery = MetricCard("Water Recovery", "—", "%", "NO DATA", "muted")
        self.card_voc = MetricCard("VOC Conversion", "—", "%", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_potable, self.card_conduct,
                                self.card_recovery, self.card_voc]))

        bars = QFrame()
        bars.setProperty("class", "card")
        bl = QHBoxLayout(bars)
        bl.setContentsMargins(18, 16, 18, 16)
        bl.setSpacing(24)
        self.meter_recovery = MeterBar("Water Recovery", "%", "green")
        self.meter_potable = MeterBar("Potable Output", "kg/day", "accent")
        bl.addWidget(self.meter_recovery, 1)
        bl.addWidget(self.meter_potable, 1)
        root.addWidget(bars)
        root.addStretch()

    def _init_ros(self):
        try:
            self.node.create_subscription(
                Float64, "/ssos/wrs/potable_kg_day", self._on_potable, 10)
            if _HAVE_DIAG:
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/wrs/diagnostics", self._on_diag, 10)
        except Exception as e:
            self.node.get_logger().warn(f"[wrs] subs failed: {e}")

    def _on_potable(self, msg):
        self._potable = msg.data

    def _on_diag(self, msg):
        for st in msg.status:
            for kv in st.values:
                try:
                    val = float(kv.value)
                except (TypeError, ValueError):
                    continue
                if kv.key == "potable_kg_day" and self._potable is None:
                    self._potable = val
                elif kv.key == "conductivity_us":
                    self._conduct = val
                elif kv.key == "overall_recovery":
                    self._recovery = val
                elif kv.key == "voc_conversion":
                    self._voc = val

    def _refresh(self):
        if self._potable is not None:
            self.card_potable.set_value(f"{self._potable:.2f}")
            self.card_potable.set_footer("TO POTABLE BUS", "green")
            self.meter_potable.set_value(
                f"{self._potable:.2f}", self._potable / POTABLE_NOMINAL_KG_DAY)
        if self._conduct is not None:
            ok = self._conduct <= POTABLE_LIMIT_US
            self.card_conduct.set_value(f"{self._conduct:.0f}")
            self.card_conduct.set_footer(
                f"LIMIT {POTABLE_LIMIT_US:.0f} µS/CM", "green" if ok else "red")
        if self._recovery is not None:
            self.card_recovery.set_value(f"{self._recovery * 100.0:.1f}")
            self.card_recovery.set_footer("OVERALL", "green")
            self.meter_recovery.set_value(
                f"{self._recovery * 100.0:.1f}", self._recovery)
        if self._voc is not None:
            self.card_voc.set_value(f"{self._voc * 100.0:.1f}")
            self.card_voc.set_footer("CATALYTIC", "green")
