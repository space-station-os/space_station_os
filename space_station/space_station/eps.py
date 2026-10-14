"""
EPSWidget — Electrical Power System console (generation and storage).

Telemetry display for the ssos_eps physics package. Subscribes to the /ssos/eps
topics, degrades gracefully ("—" / "NO DATA") when a publisher is absent, and
never crashes on missing data.

============================================================================
TOPIC CONTRACT  (matched against ssos_eps eps_node)
============================================================================
  /ssos/eps/diagnostics        diagnostic_msgs/DiagnosticArray
        status "eps":          generation_w, available_power_w, load_demand_w,
                               load_served_w, unserved_load_w, power_balance_w,
                               battery_power_w, battery_charge_w,
                               battery_discharge_w, shunted_power_w,
                               battery_soc, stored_energy_kwh, in_eclipse
        status "eps/solar":    illumination, beta_deg, gimbal_locked,
                               panel_temperature_c, ssu_voltage_v,
                               ssu_current_a, degradation_factor
        status "eps/bcdu":     charging, discharging, idle, fault, degraded
        status "eps/battery":  max_temperature_c, failed_orus

  /ssos/eps/bcdu/status               space_station_interfaces/BCDUStatus
  /ssos/eps/bcdu/channel_<n>/status   space_station_interfaces/BCDUStatus  (12)
  /ssos/eps/battery/oru_<n>/state     sensor_msgs/BatteryState             (24)
============================================================================
"""

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame
)
from PyQt5.QtCore import QTimer

from sensor_msgs.msg import BatteryState
try:
    from diagnostic_msgs.msg import DiagnosticArray
    _HAVE_DIAG = True
except Exception:
    DiagnosticArray = None
    _HAVE_DIAG = False
try:
    from space_station_interfaces.msg import BCDUStatus
    _HAVE_BCDU = True
except Exception:
    BCDUStatus = None
    _HAVE_BCDU = False

from space_station import theme
from space_station.widgets import (
    MetricCard, SpecRow, TelemetryPlot, MeterBar, page_header
)

N_CHANNELS = 12
N_ORUS = 24
ARRAY_PEAK_KW = 84.0
LOW_SOC_PCT = 20.0

_MODE_TAGS = {
    "charging": ("CHG", "green"),
    "discharging": ("DIS", "amber"),
    "idle": ("IDLE", "txt3"),
    "fault": ("FAULT", "red"),
}


def _section(text):
    lab = QLabel(text)
    lab.setProperty("class", "label")
    lab.setFont(theme.label_font(12, tracking=2.0))
    return lab


def _card_header(text):
    lab = QLabel(text)
    lab.setProperty("class", "label")
    lab.setFont(theme.label_font(10, tracking=2.0))
    return lab


def _stat_line(label, unit):
    row = QHBoxLayout()
    row.setContentsMargins(0, 0, 0, 0)
    lab = QLabel(label.upper())
    lab.setProperty("class", "label")
    lab.setFont(theme.label_font(10, tracking=1.5))
    row.addWidget(lab)
    row.addStretch()
    val = QLabel("—")
    val.setProperty("class", "value")
    val.setFont(theme.mono_font(15))
    row.addWidget(val)
    u = QLabel(unit)
    u.setProperty("class", "label")
    u.setFont(theme.label_font(10))
    row.addWidget(u)
    return row, val


def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


class _Cell(QFrame):
    """A compact tile: title + mode tag on top, a mono readout below."""

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setStyleSheet(
            f"QFrame {{ border: 1px solid {theme.color('line')};"
            f" border-radius: 4px; background: {theme.color('panel')}; }}"
            f" QLabel {{ border: none; background: transparent; }}")
        v = QVBoxLayout(self)
        v.setContentsMargins(10, 8, 10, 8)
        v.setSpacing(4)
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        self.title = QLabel(title)
        self.title.setFont(theme.label_font(10, tracking=1.5, bold=True))
        self.title.setStyleSheet(f"color: {theme.color('txt')};")
        top.addWidget(self.title)
        top.addStretch()
        self.tag = QLabel("—")
        self.tag.setFont(theme.mono_font(9))
        top.addWidget(self.tag)
        v.addLayout(top)
        self.readout = QLabel("—")
        self.readout.setFont(theme.mono_font(13))
        self.readout.setStyleSheet(f"color: {theme.color('txt2')};")
        v.addWidget(self.readout)
        self.set_tag("—", "txt3")

    def set_tag(self, text, color_key):
        self.tag.setText(text)
        self.tag.setStyleSheet(
            f"color: {theme.color(color_key)}; border: 1px solid {theme.color('line')};"
            f" border-radius: 3px; padding: 1px 4px;")

    def set_readout(self, text, color_key="txt2"):
        self.readout.setText(text)
        self.readout.setStyleSheet(f"color: {theme.color(color_key)};")


class EPSWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node

        # latched telemetry (None == no data yet)
        self._eps = {}
        self._solar = {}
        self._bcdu_diag = {}
        self._battery = {}
        self._bcdu = None
        self._channels = [None] * N_CHANNELS
        self._orus = [None] * N_ORUS
        self._plot_t = 0.0

        self._build_ui()
        self._init_ros()

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh)
        self._refresh_timer.start(1000)  # 1 Hz

    # ----------------------------- UI -----------------------------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(16)

        root.addWidget(page_header(
            "Electrical Power", "Solar Arrays · Batteries · BCDU", "EPS"))

        # --- Power balance ---
        root.addWidget(_section("POWER BALANCE · PRIMARY BUS"))
        self.card_gen = MetricCard("Array Generation", "—", "kW", "NO DATA", "muted")
        self.card_load = MetricCard("Station Load", "—", "kW", "NO DATA", "muted")
        self.card_balance = MetricCard("Power Balance", "—", "kW", "NO DATA", "muted")
        self.card_soc = MetricCard("Battery SOC", "—", "%", "NO DATA", "muted")
        self.card_bus = MetricCard("Bus Voltage", "—", "V", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_gen, self.card_load, self.card_balance,
                                self.card_soc, self.card_bus]))

        plots = QHBoxLayout()
        plots.setSpacing(16)
        soc_card = QFrame()
        soc_card.setProperty("class", "card")
        sl = QVBoxLayout(soc_card)
        sl.setContentsMargins(16, 14, 16, 14)
        sl.addWidget(_card_header("BATTERY STATE OF CHARGE"))
        self.plot_soc = TelemetryPlot(max_points=600, y_range=(0, 100),
                                      setpoint=LOW_SOC_PCT, y_label="%")
        sl.addWidget(self.plot_soc)
        plots.addWidget(soc_card, 1)

        gen_card = QFrame()
        gen_card.setProperty("class", "card")
        gl = QVBoxLayout(gen_card)
        gl.setContentsMargins(16, 14, 16, 14)
        gl.addWidget(_card_header("ARRAY GENERATION"))
        self.plot_gen = TelemetryPlot(max_points=600, y_range=(0, ARRAY_PEAK_KW * 1.05),
                                      y_label="kW")
        gl.addWidget(self.plot_gen)
        plots.addWidget(gen_card, 1)
        root.addLayout(plots)

        flows = QFrame()
        flows.setProperty("class", "card")
        fl = QHBoxLayout(flows)
        fl.setContentsMargins(18, 14, 18, 14)
        fl.setSpacing(24)
        self.meter_charge = MeterBar("Battery Charge", "kW", "green")
        self.meter_discharge = MeterBar("Battery Discharge", "kW", "amber")
        self.meter_shunt = MeterBar("Shunted", "kW", "accent")
        fl.addWidget(self.meter_charge, 1)
        fl.addWidget(self.meter_discharge, 1)
        fl.addWidget(self.meter_shunt, 1)
        root.addWidget(flows)

        # --- Solar arrays ---
        root.addWidget(_section("SOLAR ARRAYS · SARJ · SSU"))
        solar = QHBoxLayout()
        solar.setSpacing(16)
        arr_card = QFrame()
        arr_card.setProperty("class", "card")
        al = QVBoxLayout(arr_card)
        al.setContentsMargins(16, 14, 16, 14)
        al.setSpacing(12)
        al.addWidget(_card_header("ARRAY / POINTING"))
        r1, self.val_sun = _stat_line("Sun State", "")
        r2, self.val_illum = _stat_line("Illumination", "%")
        r3, self.val_beta = _stat_line("Beta Angle", "°")
        r4, self.val_gimbal = _stat_line("SARJ", "")
        r5, self.val_degr = _stat_line("Array Output", "% BOL")
        for r in (r1, r2, r3, r4, r5):
            al.addLayout(r)
        al.addStretch()
        solar.addWidget(arr_card, 1)

        ssu_card = QFrame()
        ssu_card.setProperty("class", "card")
        ul = QVBoxLayout(ssu_card)
        ul.setContentsMargins(16, 14, 16, 14)
        ul.setSpacing(12)
        ul.addWidget(_card_header("SEQUENTIAL SHUNT UNIT"))
        r6, self.val_ssu_v = _stat_line("SSU Voltage", "V")
        r7, self.val_ssu_i = _stat_line("SSU Current", "A")
        r8, self.val_avail = _stat_line("Available Power", "kW")
        r9, self.val_panel_t = _stat_line("Panel Temp", "°C")
        for r in (r6, r7, r8, r9):
            ul.addLayout(r)
        ul.addStretch()
        solar.addWidget(ssu_card, 1)
        root.addLayout(solar)

        # --- BCDU channels ---
        root.addWidget(_section("BATTERY CHARGE / DISCHARGE UNITS · 12 CHANNELS"))
        self.card_bcdu_mode = MetricCard("BCDU Mode", "—", "", "NO DATA", "muted")
        self.card_bcdu_i = MetricCard("Bus Current", "—", "A", "NO DATA", "muted")
        self.card_batt_p = MetricCard("Battery Power", "—", "kW", "NO DATA", "muted")
        self.card_bcdu_health = MetricCard("Channels", "—", "", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_bcdu_mode, self.card_bcdu_i,
                                self.card_batt_p, self.card_bcdu_health]))
        ch_frame = QFrame()
        ch_frame.setProperty("class", "card")
        cg = QGridLayout(ch_frame)
        cg.setContentsMargins(12, 12, 12, 12)
        cg.setSpacing(8)
        self.channel_cells = []
        for c in range(N_CHANNELS):
            cell = _Cell(f"CH {c}")
            cg.addWidget(cell, c // 6, c % 6)
            self.channel_cells.append(cell)
        root.addWidget(ch_frame)

        # --- Battery ORUs ---
        root.addWidget(_section("LI-ION BATTERY ORUS · 24 UNITS"))
        self.card_energy = MetricCard("Stored Energy", "—", "kWh", "NO DATA", "muted")
        self.card_min_soc = MetricCard("Lowest ORU", "—", "%", "NO DATA", "muted")
        self.card_batt_t = MetricCard("Max Temp", "—", "°C", "NO DATA", "muted")
        self.card_failed = MetricCard("Failed ORUs", "—", "", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_energy, self.card_min_soc,
                                self.card_batt_t, self.card_failed]))
        oru_frame = QFrame()
        oru_frame.setProperty("class", "card")
        og = QGridLayout(oru_frame)
        og.setContentsMargins(12, 12, 12, 12)
        og.setSpacing(8)
        self.oru_cells = []
        for i in range(N_ORUS):
            cell = _Cell(f"ORU {i}")
            og.addWidget(cell, i // 6, i % 6)
            self.oru_cells.append(cell)
        root.addWidget(oru_frame)

        root.addStretch()

    # ----------------------------- ROS -----------------------------
    def _init_ros(self):
        try:
            if _HAVE_DIAG:
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/eps/diagnostics", self._on_diag, 10)
            if _HAVE_BCDU:
                self.node.create_subscription(
                    BCDUStatus, "/ssos/eps/bcdu/status", self._on_bcdu, 10)
                for c in range(N_CHANNELS):
                    self.node.create_subscription(
                        BCDUStatus, f"/ssos/eps/bcdu/channel_{c}/status",
                        lambda msg, idx=c: self._on_channel(idx, msg), 10)
            for i in range(N_ORUS):
                self.node.create_subscription(
                    BatteryState, f"/ssos/eps/battery/oru_{i}/state",
                    lambda msg, idx=i: self._on_oru(idx, msg), 10)
        except Exception as e:
            self.node.get_logger().warn(f"[eps] subscription setup failed: {e}")

    def _on_diag(self, msg):
        for status in msg.status:
            values = {kv.key: kv.value for kv in status.values}
            values["_message"] = status.message
            values["_level"] = status.level
            if status.name == "eps":
                self._eps = values
            elif status.name == "eps/solar":
                self._solar = values
            elif status.name == "eps/bcdu":
                self._bcdu_diag = values
            elif status.name == "eps/battery":
                self._battery = values

    def _on_bcdu(self, msg):
        self._bcdu = msg

    def _on_channel(self, idx, msg):
        self._channels[idx] = msg

    def _on_oru(self, idx, msg):
        self._orus[idx] = msg

    # ----------------------------- refresh -----------------------------
    def _refresh(self):
        self._plot_t += 1.0
        self._refresh_balance()
        self._refresh_solar()
        self._refresh_bcdu()
        self._refresh_batteries()

    def _refresh_balance(self):
        e = self._eps
        gen = _to_float(e.get("generation_w"))
        load = _to_float(e.get("load_demand_w"))
        served = _to_float(e.get("load_served_w"))
        unserved = _to_float(e.get("unserved_load_w"))
        balance = _to_float(e.get("power_balance_w"))
        soc = _to_float(e.get("battery_soc"))
        eclipse = e.get("in_eclipse") == "true"

        if gen is not None:
            self.card_gen.set_value(f"{gen / 1000.0:.1f}")
            self.card_gen.set_footer("ECLIPSE" if eclipse else "SUNLIT",
                                     "muted" if eclipse else "green")
            self.plot_gen.add_point(self._plot_t, gen / 1000.0)
        if load is not None:
            self.card_load.set_value(f"{load / 1000.0:.1f}")
            if unserved is not None and unserved > 1.0:
                self.card_load.set_footer(f"{unserved / 1000.0:.1f} KW UNSERVED", "red")
            elif served is not None:
                self.card_load.set_footer("FULLY SUPPLIED", "green")
        if balance is not None:
            self.card_balance.set_value(f"{balance / 1000.0:+.1f}")
            self.card_balance.set_footer("SURPLUS" if balance >= 0 else "DEFICIT",
                                         "green" if balance >= 0 else "amber")
        if soc is not None:
            pct = soc * 100.0
            self.card_soc.set_value(f"{pct:.1f}")
            self.card_soc.set_footer(
                "LOW" if pct < LOW_SOC_PCT else "NOMINAL",
                "red" if pct < LOW_SOC_PCT else "green")
            self.plot_soc.add_point(self._plot_t, pct)
        if self._bcdu is not None:
            v = self._bcdu.bus_voltage
            self.card_bus.set_value(f"{v:.1f}")
            if v <= 0.0:
                self.card_bus.set_footer("BUS DOWN", "red")
            elif v < 155.0:
                self.card_bus.set_footer("ON BATTERY", "amber")
            else:
                self.card_bus.set_footer("ON ARRAY", "green")

        charge = _to_float(e.get("battery_charge_w"))
        discharge = _to_float(e.get("battery_discharge_w"))
        shunt = _to_float(e.get("shunted_power_w"))
        if charge is not None:
            self.meter_charge.set_value(f"{charge / 1000.0:.1f}",
                                        min(charge / 1000.0 / ARRAY_PEAK_KW, 1.0))
        if discharge is not None:
            self.meter_discharge.set_value(f"{discharge / 1000.0:.1f}",
                                           min(discharge / 1000.0 / ARRAY_PEAK_KW, 1.0))
        if shunt is not None:
            self.meter_shunt.set_value(f"{shunt / 1000.0:.1f}",
                                       min(shunt / 1000.0 / ARRAY_PEAK_KW, 1.0))

    def _refresh_solar(self):
        s = self._solar
        if not s:
            return
        illum = _to_float(s.get("illumination"))
        beta = _to_float(s.get("beta_deg"))
        degr = _to_float(s.get("degradation_factor"))
        locked = s.get("gimbal_locked") == "true"
        msg = s.get("_message", "")
        self.val_sun.setText("ECLIPSE" if msg == "eclipse" else "SUN")
        if illum is not None:
            self.val_illum.setText(f"{illum * 100.0:.0f}")
        if beta is not None:
            self.val_beta.setText(f"{beta:+.1f}")
        self.val_gimbal.setText("GIMBAL LOCK" if locked else "TRACKING")
        self.val_gimbal.setStyleSheet(
            f"color: {theme.color('amber') if locked else theme.color('txt')};")
        if degr is not None:
            self.val_degr.setText(f"{degr * 100.0:.0f}")
            self.val_degr.setStyleSheet(
                f"color: {theme.color('amber') if degr < 1.0 else theme.color('txt')};")
        for key, label, fmt in (("ssu_voltage_v", self.val_ssu_v, "{:.1f}"),
                                ("ssu_current_a", self.val_ssu_i, "{:.0f}"),
                                ("panel_temperature_c", self.val_panel_t, "{:.1f}")):
            v = _to_float(s.get(key))
            if v is not None:
                label.setText(fmt.format(v))
        avail = _to_float(self._eps.get("available_power_w"))
        if avail is not None:
            self.val_avail.setText(f"{avail / 1000.0:.1f}")

    def _refresh_bcdu(self):
        if self._bcdu is not None:
            mode = self._bcdu.mode or "idle"
            self.card_bcdu_mode.set_value(mode.upper())
            if self._bcdu.fault:
                faulted = int(self._bcdu_diag.get("fault", "0") or 0)
                self.card_bcdu_mode.set_footer(
                    (self._bcdu.fault_message or "FAULT").upper(),
                    "red" if faulted else "amber")
            else:
                self.card_bcdu_mode.set_footer("NOMINAL", "green")
            self.card_bcdu_i.set_value(f"{self._bcdu.current_draw:+.0f}")
            self.card_bcdu_i.set_footer(
                "TO BUS" if self._bcdu.current_draw > 0 else
                "FROM BUS" if self._bcdu.current_draw < 0 else "—", "muted")

        batt = _to_float(self._eps.get("battery_power_w"))
        if batt is not None:
            self.card_batt_p.set_value(f"{batt / 1000.0:+.1f}")
            self.card_batt_p.set_footer(
                "DISCHARGING" if batt > 0 else "CHARGING" if batt < 0 else "IDLE",
                "amber" if batt > 0 else "green" if batt < 0 else "muted")

        d = self._bcdu_diag
        if d:
            faulted = int(d.get("fault", "0"))
            degraded = int(d.get("degraded", "0"))
            self.card_bcdu_health.set_value(
                f"{d.get('charging', '0')}C · {d.get('discharging', '0')}D · "
                f"{d.get('idle', '0')}I")
            if faulted:
                self.card_bcdu_health.set_footer(f"{faulted} FAULTED", "red")
            elif degraded:
                self.card_bcdu_health.set_footer(f"{degraded} DEGRADED", "amber")
            else:
                self.card_bcdu_health.set_footer("ALL NOMINAL", "green")

        for c, msg in enumerate(self._channels):
            cell = self.channel_cells[c]
            if msg is None:
                continue
            mode = msg.mode or "idle"
            tag, color = _MODE_TAGS.get(mode, (mode.upper(), "txt3"))
            if msg.fault and mode != "fault":
                color = "amber"
            cell.set_tag(tag, color)
            cell.set_readout(f"{msg.current_draw:+.1f} A · {msg.bus_voltage:.0f} V",
                             "red" if mode == "fault" else "amber" if msg.fault else "txt2")
            cell.setToolTip(msg.fault_message or mode)

    def _refresh_batteries(self):
        b = self._battery
        energy = _to_float(self._eps.get("stored_energy_kwh"))
        if energy is not None:
            self.card_energy.set_value(f"{energy:.1f}")
            self.card_energy.set_footer("AT OPEN CIRCUIT", "muted")
        min_soc = _to_float(self._eps.get("min_oru_soc"))
        if min_soc is not None:
            self.card_min_soc.set_value(f"{min_soc * 100.0:.1f}")
            self.card_min_soc.set_footer(
                "LOW" if min_soc * 100.0 < LOW_SOC_PCT else "NOMINAL",
                "red" if min_soc * 100.0 < LOW_SOC_PCT else "green")
        max_t = _to_float(b.get("max_temperature_c"))
        if max_t is not None:
            self.card_batt_t.set_value(f"{max_t:.1f}")
            self.card_batt_t.set_footer("NOMINAL" if max_t < 35.0 else "OVER TEMP",
                                        "green" if max_t < 35.0 else "red")
        failed = b.get("failed_orus")
        if failed is not None:
            self.card_failed.set_value(failed)
            self.card_failed.set_footer("NONE" if failed == "0" else "ISOLATED",
                                        "green" if failed == "0" else "amber")

        for i, msg in enumerate(self._orus):
            cell = self.oru_cells[i]
            if msg is None:
                continue
            pct = msg.percentage * 100.0
            dead = msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD
            hot = msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
            status = msg.power_supply_status
            if dead:
                cell.set_tag("FAIL", "red")
            elif status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                cell.set_tag("CHG", "green")
            elif status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
                cell.set_tag("DIS", "amber")
            elif status == BatteryState.POWER_SUPPLY_STATUS_FULL:
                cell.set_tag("FULL", "accent")
            else:
                cell.set_tag("IDLE", "txt3")
            color = "red" if dead or pct < LOW_SOC_PCT else ("amber" if hot else "txt2")
            cell.set_readout(f"{pct:5.1f}% · {msg.voltage:.1f} V", color)
            cell.setToolTip(
                f"Voltage: {msg.voltage:.2f} V\n"
                f"Current: {msg.current:+.2f} A (+charge)\n"
                f"Charge: {msg.charge:.1f} / {msg.capacity:.1f} Ah\n"
                f"Temperature: {msg.temperature:.1f} °C")
