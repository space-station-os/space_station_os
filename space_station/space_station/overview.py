"""OverviewWidget: station-wide landing summary.

Shows the global system state banner, headline life-support vitals from
ssos_eclss and power vitals from ssos_eps, and degrades gracefully when topics
are absent (cards show "—" / "NO DATA"). Reuses the shared mission-control
widgets. Driven by:
    /ssos/cabin/co2_ppm           std_msgs/Float64
    /ssos/cabin/diagnostics       diagnostic_msgs/DiagnosticArray (o2, pressure)
    /ssos/ars/co2_removal_kg_day  std_msgs/Float64
    /ssos/ogs/o2_kg_day           std_msgs/Float64
    /ssos/eps/diagnostics         diagnostic_msgs/DiagnosticArray (status "eps")
plus set_state(state, degraded) called by the shell from /ssos/system_state,
set_lifecycle() on each heartbeat, and an EPS health source (the EPS panel's
health()) polled on refresh.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QSizePolicy
from PyQt5.QtCore import QTimer

from std_msgs.msg import Float64
try:
    from diagnostic_msgs.msg import DiagnosticArray
    _HAVE_DIAG = True
except Exception:
    DiagnosticArray = None
    _HAVE_DIAG = False

from space_station import theme
from space_station.eps import HEALTH_COLORS, level_int, roster_status
from space_station.widgets import MetricCard, SpecRow, SubsystemRoster, page_header

# Pa <-> torr / units
_PA_PER_TORR = 133.322
_DEFAULT_TOTAL_PA = 101325.0


class OverviewWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node

        self._co2_torr = None
        self._o2_pct = None
        self._removal = None
        self._o2_gen = None
        self._cabin_pa = _DEFAULT_TOTAL_PA
        self._eps = {}
        self._eps_level = 0
        self._eps_health_source = None

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
            "Station Overview", "Integrated Life-Support, Power & Vehicle Status",
            "SSOS · MCC"))

        # Global state banner
        self.state_banner = QFrame()
        self.state_banner.setProperty("class", "card")
        bl = QHBoxLayout(self.state_banner)
        bl.setContentsMargins(18, 14, 18, 14)
        lab = QLabel("GLOBAL STATE")
        lab.setProperty("class", "label")
        lab.setFont(theme.label_font(10, tracking=2.0))
        bl.addWidget(lab)
        self.state_value = QLabel("INIT")
        self.state_value.setFont(theme.mono_font(18, bold=True))
        bl.addWidget(self.state_value)
        bl.addStretch()
        self.degraded_label = QLabel("")
        self.degraded_label.setFont(theme.sans_font(12))
        self.degraded_label.setStyleSheet(f"color: {theme.color('txt3')};")
        bl.addWidget(self.degraded_label)
        root.addWidget(self.state_banner)

        # Headline vitals
        root.addWidget(self._section("LIFE SUPPORT · ECLSS"))
        self.card_co2 = MetricCard("CO₂ Partial", "—", "torr", "NO DATA", "muted")
        self.card_o2 = MetricCard("O₂ Level", "—", "%", "NO DATA", "muted")
        self.card_removal = MetricCard("CO₂ Removal", "—", "kg/day", "NO DATA", "muted")
        self.card_o2gen = MetricCard("O₂ Generation", "—", "kg/day", "NO DATA", "muted")
        self.card_press = MetricCard("Cabin Pressure", "—", "kPa", "NO DATA", "muted")
        root.addWidget(self._row([self.card_co2, self.card_o2, self.card_removal,
                                  self.card_o2gen, self.card_press]))

        root.addWidget(self._section("POWER · EPS"))
        self.card_eps_gen = MetricCard("Array Generation", "—", "kW", "NO DATA", "muted")
        self.card_eps_load = MetricCard("Station Load", "—", "kW", "NO DATA", "muted")
        self.card_eps_soc = MetricCard("Battery SOC", "—", "%", "NO DATA", "muted")
        self.card_eps_balance = MetricCard("Power Balance", "—", "kW", "NO DATA", "muted")
        self.card_eps_health = MetricCard("EPS Health", "—", "", "NO DATA", "muted")
        root.addWidget(self._row([self.card_eps_gen, self.card_eps_load, self.card_eps_soc,
                                  self.card_eps_balance, self.card_eps_health]))

        # Roster (expanded) + ECLSS lifecycle roster side by side.
        body = QHBoxLayout()
        body.setSpacing(16)

        roster_card = QFrame()
        roster_card.setProperty("class", "card")
        rl = QVBoxLayout(roster_card)
        rl.setContentsMargins(0, 4, 0, 4)
        self.roster = SubsystemRoster(["ECLSS", "GNC", "EPS", "Thermal", "Comms"])
        rl.addWidget(self.roster)
        body.addWidget(roster_card)

        # Lifecycle state of each managed node (ECLSS units and EPS): its
        # transition mode UNCONFIGURED/INACTIVE/ACTIVE, or FAULT/DEGRADED.
        life_card = QFrame()
        life_card.setProperty("class", "card")
        ll = QVBoxLayout(life_card)
        ll.setContentsMargins(0, 4, 0, 4)
        life_hdr = QLabel("LIFECYCLE")
        life_hdr.setProperty("class", "label")
        life_hdr.setFont(theme.label_font(11, tracking=2.0))
        life_hdr.setContentsMargins(14, 12, 14, 8)
        ll.addWidget(life_hdr)
        self.lifecycle_roster = SubsystemRoster(
            ["ARS", "OGS", "WRS", "Sabatier", "Cabin", "EPS"])
        # Hide the inner "SUBSYSTEMS" header of the nested roster.
        self.lifecycle_roster.layout().itemAt(0).widget().hide()
        ll.addWidget(self.lifecycle_roster)
        for name in ("ARS", "OGS", "WRS", "Sabatier", "Cabin", "EPS"):
            self.lifecycle_roster.set_status(name, "unconfigured")
        body.addWidget(life_card)

        root.addLayout(body)
        root.addStretch()

    @staticmethod
    def _row(cards):
        # The stacked pages share the tallest page's height; keep the card rows
        # at their natural height so the spare space goes below the content.
        row = SpecRow(cards)
        row.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        return row

    def _section(self, text):
        lab = QLabel(text)
        lab.setProperty("class", "label")
        lab.setFont(theme.label_font(12, tracking=2.0))
        return lab

    # ---- lifecycle hook (called by the shell on each heartbeat) ----
    def set_lifecycle(self, name, lifecycle_word, healthy, unhealthy_status="fault"):
        """Update one node's lifecycle/transition state on the overview."""
        label = {"ars": "ARS", "ogs": "OGS", "wrs": "WRS",
                 "sabatier": "Sabatier", "cabin": "Cabin", "eps": "EPS"}.get(str(name).lower())
        if label is None:
            return
        # An active-but-unhealthy node reads as FAULT (or the caller's finer
        # status); otherwise show the transition state.
        status = unhealthy_status if (lifecycle_word == "ACTIVE" and not healthy) \
            else lifecycle_word
        self.lifecycle_roster.set_status(label, status)

    def set_eps_health_source(self, source):
        """source() -> (word, message, lifecycle), e.g. EPSWidget.health."""
        self._eps_health_source = source

    def _init_ros(self):
        try:
            self.node.create_subscription(
                Float64, "/ssos/cabin/co2_ppm", self._on_co2_ppm, 10)
            self.node.create_subscription(
                Float64, "/ssos/ars/co2_removal_kg_day", self._on_removal, 10)
            self.node.create_subscription(
                Float64, "/ssos/ogs/o2_kg_day", self._on_o2_gen, 10)
            if _HAVE_DIAG:
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/cabin/diagnostics", self._on_cabin_diag, 10)
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/eps/diagnostics", self._on_eps_diag, 10)
        except Exception as e:
            self.node.get_logger().warn(f"[overview] subs failed: {e}")

    # ---- callbacks ----
    def _on_co2_ppm(self, msg):
        frac = msg.data * 1e-6
        self._co2_torr = frac * self._cabin_pa / _PA_PER_TORR

    def _on_removal(self, msg):
        self._removal = msg.data

    def _on_o2_gen(self, msg):
        self._o2_gen = msg.data

    def _on_cabin_diag(self, msg):
        for status in msg.status:
            for kv in status.values:
                try:
                    val = float(kv.value)
                except (TypeError, ValueError):
                    continue
                if kv.key == "o2_fraction":
                    self._o2_pct = val * 100.0
                elif kv.key == "total_pressure_kpa":
                    self._cabin_pa = val * 1000.0

    def _on_eps_diag(self, msg):
        for status in msg.status:
            if status.name == "eps":
                self._eps = {kv.key: kv.value for kv in status.values}
                self._eps_level = level_int(status.level)

    # ---- shell hook ----
    def set_state(self, state, degraded):
        self.state_value.setText(state)
        color = {
            "NOMINAL": "green", "DEGRADED": "amber", "RECOVERY": "amber",
            "SAFE": "red", "INIT": "txt3",
        }.get(state, "txt3")
        self.state_value.setStyleSheet(f"color: {theme.color(color)};")
        if degraded:
            self.degraded_label.setText("DEGRADED: " + ", ".join(degraded))
        else:
            self.degraded_label.setText("ALL SUBSYSTEMS NOMINAL")
        degraded_up = {d.upper() for d in degraded}
        for name in ["ECLSS", "GNC", "EPS", "Thermal", "Comms"]:
            self.roster.set_status(
                name, "degraded" if name.upper() in degraded_up else "nominal")

    # ---- refresh ----
    def _refresh(self):
        if self._co2_torr is not None:
            ok = self._co2_torr <= 3.0
            self.card_co2.set_value(f"{self._co2_torr:.2f}")
            self.card_co2.set_footer(
                "WITHIN LIMITS" if ok else "ELEVATED", "green" if ok else "amber")
        if self._o2_pct is not None:
            ok = 19.5 <= self._o2_pct <= 23.5
            self.card_o2.set_value(f"{self._o2_pct:.1f}")
            self.card_o2.set_footer(
                "NOMINAL" if ok else "OUT OF RANGE", "green" if ok else "amber")
        if self._removal is not None:
            ok = self._removal >= 4.16
            self.card_removal.set_value(f"{self._removal:.2f}")
            self.card_removal.set_footer(
                "REQ 4.16 KG/DAY", "green" if ok else "amber")
        if self._o2_gen is not None:
            self.card_o2gen.set_value(f"{self._o2_gen:.2f}")
            self.card_o2gen.set_footer("OGS ONLINE", "green")
        if self._cabin_pa:
            kpa = self._cabin_pa / 1000.0
            ok = 95.0 <= kpa <= 104.0
            self.card_press.set_value(f"{kpa:.1f}")
            self.card_press.set_footer(
                "NOMINAL" if ok else "CHECK", "green" if ok else "amber")
        self._refresh_eps()

    def _refresh_eps(self):
        def num(key):
            try:
                return float(self._eps[key])
            except (KeyError, TypeError, ValueError):
                return None

        gen = num("generation_w")
        load = num("load_demand_w")
        served = num("load_served_w")
        soc = num("battery_soc")
        balance = num("power_balance_w")
        if gen is not None:
            eclipse = self._eps.get("in_eclipse") == "true"
            self.card_eps_gen.set_value(f"{gen / 1000.0:.1f}")
            self.card_eps_gen.set_footer("ECLIPSE" if eclipse else "SUNLIT",
                                         "muted" if eclipse else "green")
        if load is not None and served is not None:
            short = load - served
            self.card_eps_load.set_value(f"{load / 1000.0:.1f}")
            self.card_eps_load.set_footer(
                f"{short / 1000.0:.1f} KW SHORT" if short > 1.0 else "FULLY SUPPLIED",
                "red" if short > 1.0 else "green")
        if soc is not None:
            pct = soc * 100.0
            self.card_eps_soc.set_value(f"{pct:.1f}")
            self.card_eps_soc.set_footer("LOW" if pct < 20.0 else "NOMINAL",
                                         "red" if pct < 20.0 else "green")
        if balance is not None:
            self.card_eps_balance.set_value(f"{balance / 1000.0:+.1f}")
            self.card_eps_balance.set_footer("SURPLUS" if balance >= 0 else "DEFICIT",
                                             "green" if balance >= 0 else "amber")

        if self._eps_health_source is not None:
            word, message, lifecycle = self._eps_health_source()
            color = HEALTH_COLORS.get(word, "txt3")
            footer = {"green": "green", "amber": "amber", "red": "red"}.get(color, "muted")
            self.card_eps_health.set_value(word)
            self.card_eps_health.set_footer((message or word).upper()[:40], footer)
            # Healthy: show the lifecycle state; otherwise the health status.
            self.lifecycle_roster.set_status(
                "EPS", lifecycle if word == "NOMINAL" else roster_status(word))
