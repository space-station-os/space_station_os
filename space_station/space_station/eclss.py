"""
EclssWidget — Air Revitalization (Four-Bed Molecular Sieve) console.

Rebuilt for the ssos_eclss high-fidelity physics package. Pure telemetry
display: subscribes to the real ssos topics, degrades gracefully (shows "—" /
"NO DATA") when a publisher is absent, and never crashes on missing data.

============================================================================
TOPIC CONTRACT  (matched against ssos_eclss as of feature/ssos-eclss)
============================================================================
Subscribed (all best-effort; panel works with any subset present):

  /ssos/cabin/co2_ppm          std_msgs/Float64
        Cabin CO2 concentration in ppm. Converted to a partial pressure in
        torr using the cabin total pressure (below) for the "CO₂ Partial"
        metric and the trend plot.

  /ssos/cabin/diagnostics      diagnostic_msgs/DiagnosticArray  (name="cabin")
        KeyValues consumed:
          o2_fraction         -> O₂ Level (%)
          total_pressure_kpa  -> Cabin Pressure (kPa) + torr conversion base
          (relative_humidity, co2_ppm also published; co2 used as fallback)

  /ssos/ars/co2_removal_kg_day std_msgs/Float64
        CO2 removal rate. Compared against the 4.16 kg/day ISS requirement.

  /ssos/ars/diagnostics        diagnostic_msgs/DiagnosticArray  (name="ars")
        KeyValues consumed:
          precooler_exit_k    -> Precooler exit temp
          system_dp_in_h2o    -> Blower / system pressure drop
          blower_flow_scfm    -> Air flow
          max_bed_temp_k      -> desorbing-bed temperature readout
          adsorbing_train     -> which train (0/1) is adsorbing vs desorbing
          co2_removal_kg_day, scrubbed_co2_torr (also published)

  /ssos/ars/bed_states         std_msgs/Float64MultiArray  (12 elements)
        Per-bed [ADS A1, ADS A2, DES D1, DES D2]:
          [0..3]  loading fraction (0-1)        -> bed fill bars
          [4..7]  solid temperature [K]         -> bed temp readouts
          [8..11] mode code (0=ADSORBING,1=DESORBING,2=AIR_SAVE,3=VACUUM)
                                                -> bed ADS/DES tags + hot colour

  /ssos/ars/cycle_phase        std_msgs/Float64MultiArray  (3 elements)
          [0] elapsed time in the half-cycle [s]
          [1] half-cycle duration [s]           -> 10-60-10 cycle-bar marker
          [2] adsorbing train index (0/1)

FALLBACK MODEL:
  * If /ssos/ars/bed_states or /ssos/ars/cycle_phase have no publisher, the
    bed bars and cycle marker fall back to a local 80-minute half-cycle clock
    estimate so the panel still animates; real telemetry overrides it.
============================================================================
"""

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QFrame
)
from PyQt5.QtCore import Qt, QTimer

from std_msgs.msg import Float64, Float64MultiArray
try:
    from diagnostic_msgs.msg import DiagnosticArray
    _HAVE_DIAG = True
except Exception:
    DiagnosticArray = None
    _HAVE_DIAG = False

from space_station import theme
from space_station.widgets import (
    MetricCard, SpecRow, TelemetryPlot, BedLoadBar, CyclePhaseBar, page_header
)

# --- Unit helpers / constants ---
_PA_PER_TORR = 133.322
_DEFAULT_TOTAL_PA = 101325.0
CO2_SETPOINT_TORR = 2.0
CO2_REMOVAL_REQ = 4.16  # kg/day (ISS 4-crew requirement)
HALF_CYCLE_MIN = 80.0   # 10 air-save + 60 adsorb/desorb + 10 vacuum
O2_REQUIRED_KG_DAY = 2.3   # OGS O2 production requirement
POTABLE_LIMIT_US = 100.0   # WRS potable-water conductivity limit [µS/cm]


def k_to_c(k):
    return k - 273.15


def k_to_f(k):
    return (k - 273.15) * 9.0 / 5.0 + 32.0


def _stat_line(label, unit):
    """A label/value/unit line for the precooler/blower stat block."""
    row = QHBoxLayout()
    row.setContentsMargins(0, 0, 0, 0)
    lab = QLabel(label.upper())
    lab.setProperty("class", "label")
    lab.setFont(theme.label_font(8, tracking=1.5))
    row.addWidget(lab)
    row.addStretch()
    val = QLabel("—")
    val.setProperty("class", "value")
    val.setFont(theme.mono_font(13))
    row.addWidget(val)
    u = QLabel(unit)
    u.setProperty("class", "label")
    u.setFont(theme.label_font(8))
    row.addWidget(u)
    return row, val


class _Bed(QFrame):
    """One sieve bed cell: name + mode tag, a BedLoadBar, load% + temp."""

    def __init__(self, name, mode="ADS", parent=None):
        super().__init__(parent)
        self.setStyleSheet("QFrame { border: none; background: transparent; }")
        v = QVBoxLayout(self)
        v.setContentsMargins(16, 12, 16, 12)
        v.setSpacing(8)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        self.name = QLabel(name)
        self.name.setFont(theme.label_font(9, tracking=2.0, bold=True))
        self.name.setStyleSheet(f"color: {theme.color('txt')};")
        top.addWidget(self.name)
        top.addStretch()
        self.tag = QLabel(mode)
        self.tag.setFont(theme.mono_font(8))
        top.addWidget(self.tag)
        v.addLayout(top)

        self.bar = BedLoadBar(0.0, mode)
        v.addWidget(self.bar)

        readout = QHBoxLayout()
        readout.setContentsMargins(0, 0, 0, 0)
        self.load = QLabel("—")
        self.load.setFont(theme.mono_font(11))
        self.load.setStyleSheet(f"color: {theme.color('txt2')};")
        readout.addWidget(self.load)
        readout.addStretch()
        self.temp = QLabel("—")
        self.temp.setFont(theme.mono_font(11))
        self.temp.setStyleSheet(f"color: {theme.color('txt3')};")
        readout.addWidget(self.temp)
        v.addLayout(readout)

        self.set_mode(mode)

    def set_mode(self, mode):
        mode = (mode or "ADS").upper()
        self._mode = mode
        self.tag.setText(mode)
        is_des = mode == "DES"
        color = theme.color("amber") if is_des else theme.color("txt3")
        self.tag.setStyleSheet(
            f"color: {color}; border: 1px solid {theme.color('line')};"
            f" border-radius: 3px; padding: 1px 5px;"
        )
        self.bar.set_mode(mode)

    def set_loading(self, fraction):
        self.bar.set_loading(fraction)
        self.load.setText(f"{fraction * 100:.0f}%")

    def set_temp_f(self, temp_f, hot=False):
        if temp_f is None:
            self.temp.setText("—")
            return
        self.temp.setText(f"{temp_f:.0f}°F")
        self.temp.setStyleSheet(
            f"color: {theme.color('amber') if hot else theme.color('txt3')};")


class EclssWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node

        # latched telemetry (None == no data yet)
        self._co2_ppm = None
        self._o2_frac = None
        self._cabin_pa = _DEFAULT_TOTAL_PA
        self._removal = None
        self._precooler_k = None
        self._system_dp = None
        self._flow_scfm = None
        self._max_bed_k = None
        self._adsorbing_train = 0
        self._have_cabin_press = False

        # Real ARS bed states + cycle phase (from ssos_eclss). None == fall back
        # to the local GUI estimate so the panel still animates without them.
        self._bed_states = None     # 12 floats: 4 load, 4 tempK, 4 mode codes
        self._cycle_real = None     # (elapsed_s, half_cycle_s, adsorbing_train)

        # OGS (oxygen generation) telemetry
        self._o2_prod = None
        self._stack_v = None
        self._stack_p = None
        self._stack_t_k = None
        # WRS (water recovery) telemetry
        self._potable = None
        self._conduct_us = None
        self._recovery = None
        self._voc_conv = None

        # GUI-side cycle model (fallback when /ssos/ars/cycle_phase is absent)
        self._cycle_min = 0.0
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
            "Environmental Control & Life Support",
            "Air · Oxygen · Water Recovery", "ECLSS"))

        # --- Air Revitalization (ARS / 4BMS) section ---
        ars_hdr = QLabel("AIR REVITALIZATION · ARS · 4BMS")
        ars_hdr.setProperty("class", "label")
        ars_hdr.setFont(theme.label_font(10, tracking=2.0))
        root.addWidget(ars_hdr)

        # --- Spec row ---
        self.card_co2 = MetricCard("CO₂ Partial", "—", "torr", "NO DATA", "muted")
        self.card_o2 = MetricCard("O₂ Level", "—", "%", "NO DATA", "muted")
        self.card_removal = MetricCard("CO₂ Removal", "—", "kg/day", "NO DATA", "muted")
        self.card_press = MetricCard("Cabin Pressure", "—", "kPa", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_co2, self.card_o2,
                                self.card_removal, self.card_press]))

        # --- Two-column row: plot (wide) + precooler/blower stats ---
        cols = QHBoxLayout()
        cols.setSpacing(16)

        plot_card = QFrame()
        plot_card.setProperty("class", "card")
        pcl = QVBoxLayout(plot_card)
        pcl.setContentsMargins(16, 14, 16, 14)
        plot_hdr = QLabel("CO₂ PARTIAL PRESSURE · 80 MIN")
        plot_hdr.setProperty("class", "label")
        plot_hdr.setFont(theme.label_font(8, tracking=2.0))
        pcl.addWidget(plot_hdr)
        self.plot = TelemetryPlot(
            max_points=240, y_range=(0, 4.0), setpoint=CO2_SETPOINT_TORR,
            y_label="torr")
        pcl.addWidget(self.plot)
        cols.addWidget(plot_card, 3)

        stat_card = QFrame()
        stat_card.setProperty("class", "card")
        scl = QVBoxLayout(stat_card)
        scl.setContentsMargins(16, 14, 16, 14)
        scl.setSpacing(12)
        stat_hdr = QLabel("PRECOOLER / BLOWER")
        stat_hdr.setProperty("class", "label")
        stat_hdr.setFont(theme.label_font(8, tracking=2.0))
        scl.addWidget(stat_hdr)
        r1, self.val_precooler = _stat_line("Precooler Exit", "°C")
        r2, self.val_dp = _stat_line("System ΔP", "in·H₂O")
        r3, self.val_flow = _stat_line("Air Flow", "SCFM")
        r4, self.val_ltl = _stat_line("LTL Temp", "°C")
        for r in (r1, r2, r3, r4):
            scl.addLayout(r)
        scl.addStretch()
        cols.addWidget(stat_card, 2)
        root.addLayout(cols)

        # --- Cycle phase block ---
        cycle_card = QFrame()
        cycle_card.setProperty("class", "card")
        ccl = QVBoxLayout(cycle_card)
        ccl.setContentsMargins(16, 14, 16, 14)
        ccl.setSpacing(8)
        cyc_hdr = QLabel("HALF-CYCLE PHASE")
        cyc_hdr.setProperty("class", "label")
        cyc_hdr.setFont(theme.label_font(8, tracking=2.0))
        ccl.addWidget(cyc_hdr)
        self.cycle_bar = CyclePhaseBar()
        ccl.addWidget(self.cycle_bar)
        marks = QHBoxLayout()
        marks.setContentsMargins(0, 0, 0, 0)
        for i, m in enumerate(["0", "10", "70", "80 MIN"]):
            lab = QLabel(m)
            lab.setFont(theme.mono_font(8))
            lab.setStyleSheet(f"color: {theme.color('txt3')};")
            if i == 0:
                lab.setAlignment(Qt.AlignLeft)
            elif i == 3:
                lab.setAlignment(Qt.AlignRight)
            else:
                lab.setAlignment(Qt.AlignCenter)
            marks.addWidget(lab, 1)
        ccl.addLayout(marks)
        root.addWidget(cycle_card)

        # --- Bed row ---
        bed_card = QFrame()
        bed_card.setProperty("class", "flush")
        bed_row = QHBoxLayout(bed_card)
        bed_row.setContentsMargins(0, 0, 0, 0)
        bed_row.setSpacing(0)
        self.beds = [
            _Bed("BED A1", "ADS"),
            _Bed("BED A2", "ADS"),
            _Bed("BED D1", "DES"),
            _Bed("BED D2", "DES"),
        ]
        for i, bed in enumerate(self.beds):
            bed_row.addWidget(bed, 1)
            if i < len(self.beds) - 1:
                d = QFrame()
                d.setProperty("class", "vline")
                d.setFixedWidth(1)
                bed_row.addWidget(d)
        root.addWidget(bed_card)

        # --- Oxygen Generation (OGS) section ---
        ogs_hdr = QLabel("OXYGEN GENERATION · OGS · PEM ELECTROLYSIS")
        ogs_hdr.setProperty("class", "label")
        ogs_hdr.setFont(theme.label_font(10, tracking=2.0))
        root.addWidget(ogs_hdr)
        self.card_o2_prod = MetricCard("O₂ Production", "—", "kg/day", "NO DATA", "muted")
        self.card_stack_v = MetricCard("Stack Voltage", "—", "V", "NO DATA", "muted")
        self.card_stack_p = MetricCard("Stack Power", "—", "W", "NO DATA", "muted")
        self.card_stack_t = MetricCard("Stack Temp", "—", "°C", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_o2_prod, self.card_stack_v,
                                self.card_stack_p, self.card_stack_t]))

        # --- Water Recovery (WRS) section ---
        wrs_hdr = QLabel("WATER RECOVERY · WRS · UPA + WPA")
        wrs_hdr.setProperty("class", "label")
        wrs_hdr.setFont(theme.label_font(10, tracking=2.0))
        root.addWidget(wrs_hdr)
        self.card_potable = MetricCard("Potable Water", "—", "kg/day", "NO DATA", "muted")
        self.card_conduct = MetricCard("Conductivity", "—", "µS/cm", "NO DATA", "muted")
        self.card_recovery = MetricCard("Water Recovery", "—", "%", "NO DATA", "muted")
        self.card_voc = MetricCard("VOC Conversion", "—", "%", "NO DATA", "muted")
        root.addWidget(SpecRow([self.card_potable, self.card_conduct,
                                self.card_recovery, self.card_voc]))

        root.addStretch()

    # ----------------------------- ROS -----------------------------
    def _init_ros(self):
        try:
            self.node.create_subscription(
                Float64, "/ssos/cabin/co2_ppm", self._on_co2_ppm, 10)
            self.node.create_subscription(
                Float64, "/ssos/ars/co2_removal_kg_day", self._on_removal, 10)
            if _HAVE_DIAG:
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/cabin/diagnostics",
                    self._on_cabin_diag, 10)
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/ars/diagnostics",
                    self._on_ars_diag, 10)
            # High-fidelity per-bed + cycle telemetry (ssos_eclss ArsNode).
            self.node.create_subscription(
                Float64MultiArray, "/ssos/ars/bed_states", self._on_bed_states, 10)
            self.node.create_subscription(
                Float64MultiArray, "/ssos/ars/cycle_phase", self._on_cycle_phase, 10)
            # OGS + WRS telemetry (ssos_eclss OxygenGeneratorSystem / WaterRecovery).
            self.node.create_subscription(
                Float64, "/ssos/ogs/o2_kg_day", self._on_ogs_o2, 10)
            self.node.create_subscription(
                Float64, "/ssos/wrs/potable_kg_day", self._on_wrs_water, 10)
            if _HAVE_DIAG:
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/ogs/diagnostics", self._on_ogs_diag, 10)
                self.node.create_subscription(
                    DiagnosticArray, "/ssos/wrs/diagnostics", self._on_wrs_diag, 10)
        except Exception as e:
            self.node.get_logger().warn(f"[eclss] subscription setup failed: {e}")

    def _on_ogs_o2(self, msg):
        self._o2_prod = msg.data

    def _on_wrs_water(self, msg):
        self._potable = msg.data

    def _on_ogs_diag(self, msg):
        for status in msg.status:
            for kv in status.values:
                try:
                    val = float(kv.value)
                except (TypeError, ValueError):
                    continue
                if kv.key == "o2_kg_day" and self._o2_prod is None:
                    self._o2_prod = val
                elif kv.key == "stack_voltage":
                    self._stack_v = val
                elif kv.key == "stack_power_w":
                    self._stack_p = val
                elif kv.key == "stack_temp_k":
                    self._stack_t_k = val

    def _on_wrs_diag(self, msg):
        for status in msg.status:
            for kv in status.values:
                try:
                    val = float(kv.value)
                except (TypeError, ValueError):
                    continue
                if kv.key == "potable_kg_day" and self._potable is None:
                    self._potable = val
                elif kv.key == "conductivity_us":
                    self._conduct_us = val
                elif kv.key == "overall_recovery":
                    self._recovery = val
                elif kv.key == "voc_conversion":
                    self._voc_conv = val

    def _on_bed_states(self, msg):
        if len(msg.data) >= 12:
            self._bed_states = list(msg.data)

    def _on_cycle_phase(self, msg):
        if len(msg.data) >= 2:
            self._cycle_real = (msg.data[0], msg.data[1],
                                int(round(msg.data[2])) if len(msg.data) >= 3 else 0)

    def _on_co2_ppm(self, msg):
        self._co2_ppm = msg.data

    def _on_removal(self, msg):
        self._removal = msg.data

    def _on_cabin_diag(self, msg):
        for status in msg.status:
            for kv in status.values:
                try:
                    val = float(kv.value)
                except (TypeError, ValueError):
                    continue
                if kv.key == "o2_fraction":
                    self._o2_frac = val
                elif kv.key == "total_pressure_kpa":
                    self._cabin_pa = val * 1000.0
                    self._have_cabin_press = True
                elif kv.key == "co2_ppm" and self._co2_ppm is None:
                    self._co2_ppm = val

    def _on_ars_diag(self, msg):
        for status in msg.status:
            for kv in status.values:
                try:
                    val = float(kv.value)
                except (TypeError, ValueError):
                    continue
                if kv.key == "precooler_exit_k":
                    self._precooler_k = val
                elif kv.key == "system_dp_in_h2o":
                    self._system_dp = val
                elif kv.key == "blower_flow_scfm":
                    self._flow_scfm = val
                elif kv.key == "max_bed_temp_k":
                    self._max_bed_k = val
                elif kv.key == "adsorbing_train":
                    self._adsorbing_train = int(round(val))
                elif kv.key == "co2_removal_kg_day" and self._removal is None:
                    self._removal = val

    # --------------------------- helpers ---------------------------
    def _co2_torr(self):
        if self._co2_ppm is None:
            return None
        return (self._co2_ppm * 1e-6) * self._cabin_pa / _PA_PER_TORR

    # --------------------------- refresh ---------------------------
    def _refresh(self):
        # advance GUI cycle clock (1 sim-min per second, wraps at 80)
        self._cycle_min = (self._cycle_min + 1.0) % HALF_CYCLE_MIN
        self._plot_t += 1.0

        # ---- Spec cards ----
        co2 = self._co2_torr()
        if co2 is not None:
            ok = co2 <= CO2_SETPOINT_TORR + 0.5
            self.card_co2.set_value(f"{co2:.2f}")
            self.card_co2.set_footer(
                f"SETPOINT {CO2_SETPOINT_TORR:.1f}", "green" if ok else "amber")
            self.plot.add_point(self._plot_t, co2)

        if self._o2_frac is not None:
            pct = self._o2_frac * 100.0
            ok = 19.5 <= pct <= 23.5
            self.card_o2.set_value(f"{pct:.1f}")
            self.card_o2.set_footer(
                "NOMINAL" if ok else "OUT OF RANGE", "green" if ok else "amber")

        if self._removal is not None:
            ok = self._removal >= CO2_REMOVAL_REQ
            self.card_removal.set_value(f"{self._removal:.2f}")
            self.card_removal.set_footer(
                f"REQ {CO2_REMOVAL_REQ:.2f} KG/DAY", "green" if ok else "amber")

        if self._have_cabin_press:
            kpa = self._cabin_pa / 1000.0
            ok = 95.0 <= kpa <= 104.0
            self.card_press.set_value(f"{kpa:.1f}")
            self.card_press.set_footer(
                "NOMINAL" if ok else "CHECK", "green" if ok else "amber")

        # ---- Precooler / blower stats ----
        if self._precooler_k is not None:
            self.val_precooler.setText(f"{k_to_c(self._precooler_k):.1f}")
        if self._system_dp is not None:
            self.val_dp.setText(f"{self._system_dp:.2f}")
        if self._flow_scfm is not None:
            self.val_flow.setText(f"{self._flow_scfm:.0f}")
        # LTL (low-temp loop) not published by ssos_eclss -> stays "—"

        # ---- Cycle phase ---- (real telemetry if present, else local clock)
        if self._cycle_real is not None:
            elapsed_min = self._cycle_real[0] / 60.0
            total_min = self._cycle_real[1] / 60.0 if self._cycle_real[1] > 0 else HALF_CYCLE_MIN
            self.cycle_bar.set_phase(elapsed_min, total_min)
        else:
            self.cycle_bar.set_phase(self._cycle_min, HALF_CYCLE_MIN)

        # ---- Beds ----
        if self._bed_states is not None:
            self._update_beds_real()
        else:
            self._update_beds()

        # ---- OGS ----
        if self._o2_prod is not None:
            ok = self._o2_prod >= O2_REQUIRED_KG_DAY
            self.card_o2_prod.set_value(f"{self._o2_prod:.2f}")
            self.card_o2_prod.set_footer(
                f"REQ {O2_REQUIRED_KG_DAY:.1f} KG/DAY", "green" if ok else "amber")
        if self._stack_v is not None:
            self.card_stack_v.set_value(f"{self._stack_v:.1f}")
            self.card_stack_v.set_footer("STACK", "green")
        if self._stack_p is not None:
            self.card_stack_p.set_value(f"{self._stack_p:.0f}")
            self.card_stack_p.set_footer("ELECTRICAL", "green")
        if self._stack_t_k is not None:
            self.card_stack_t.set_value(f"{k_to_c(self._stack_t_k):.1f}")
            self.card_stack_t.set_footer("CELL STACK", "green")

        # ---- WRS ----
        if self._potable is not None:
            self.card_potable.set_value(f"{self._potable:.2f}")
            self.card_potable.set_footer("TO POTABLE BUS", "green")
        if self._conduct_us is not None:
            ok = self._conduct_us <= POTABLE_LIMIT_US
            self.card_conduct.set_value(f"{self._conduct_us:.0f}")
            self.card_conduct.set_footer(
                f"LIMIT {POTABLE_LIMIT_US:.0f} µS/CM", "green" if ok else "red")
        if self._recovery is not None:
            self.card_recovery.set_value(f"{self._recovery * 100.0:.1f}")
            self.card_recovery.set_footer("OVERALL", "green")
        if self._voc_conv is not None:
            self.card_voc.set_value(f"{self._voc_conv * 100.0:.1f}")
            self.card_voc.set_footer("CATALYTIC", "green")

    def _update_beds(self):
        """Trains: A (beds 0,1), D (beds 2,3). adsorbing_train picks which
        train is adsorbing; the other desorbs. Loading is a GUI-side estimate
        from the half-cycle clock; desorbing-bed temp uses max_bed_temp_k."""
        # adsorb-phase progress within the 10..70 min window (0..1)
        if self._cycle_min <= 10.0:
            adsorb_frac = 0.0
        elif self._cycle_min >= 70.0:
            adsorb_frac = 1.0
        else:
            adsorb_frac = (self._cycle_min - 10.0) / 60.0

        a_adsorbing = (self._adsorbing_train == 0)
        des_temp_f = k_to_f(self._max_bed_k) if self._max_bed_k is not None else None
        cabin_temp_f = 70.0  # nominal cabin air temp for adsorbing beds

        for idx in (0, 1):  # train A beds
            if a_adsorbing:
                self.beds[idx].set_mode("ADS")
                self.beds[idx].set_loading(adsorb_frac)
                self.beds[idx].set_temp_f(cabin_temp_f, hot=False)
            else:
                self.beds[idx].set_mode("DES")
                self.beds[idx].set_loading(1.0 - adsorb_frac)
                self.beds[idx].set_temp_f(des_temp_f, hot=True)
        for idx in (2, 3):  # train D beds
            if a_adsorbing:
                self.beds[idx].set_mode("DES")
                self.beds[idx].set_loading(1.0 - adsorb_frac)
                self.beds[idx].set_temp_f(des_temp_f, hot=True)
            else:
                self.beds[idx].set_mode("ADS")
                self.beds[idx].set_loading(adsorb_frac)
                self.beds[idx].set_temp_f(cabin_temp_f, hot=False)

    def _update_beds_real(self):
        """Drive the four bed cells directly from /ssos/ars/bed_states.
        Layout (12 floats): [load x4][tempK x4][mode x4], bed order
        [ADS A1, ADS A2, DES D1, DES D2]. mode 0=ADSORBING else regenerating."""
        d = self._bed_states
        for i in range(4):
            loading = d[i]
            temp_k = d[4 + i]
            mode_code = int(round(d[8 + i]))
            adsorbing = (mode_code == 0)
            self.beds[i].set_mode("ADS" if adsorbing else "DES")
            self.beds[i].set_loading(max(0.0, min(1.0, loading)))
            self.beds[i].set_temp_f(k_to_f(temp_k), hot=not adsorbing)
