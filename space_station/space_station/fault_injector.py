#!/usr/bin/env python3
"""Fault-injection (drill) dialog.

Every drill goes through the simulation controller's /sim/inject_fault
service. The controller allows one drill at a time and announces it on
/ssos/fault_event; the target subsystem applies it to its own physics, and
system_manager and the event feed see it there. "Restore Nominal" sends
clear_faults to the subsystem running the drill.
"""
import json
import time

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QPushButton
)
from PyQt5.QtCore import Qt

try:
    from space_station_interfaces.srv import InjectFault
    _HAVE_INJECT = True
except Exception:
    InjectFault = None
    _HAVE_INJECT = False

from space_station import theme


def _drill(label, target, fault_type, desc, **params):
    return {"label": label, "target": target, "type": fault_type,
            "params": params, "desc": desc}


# Each drill: (list label, target subsystem, fault type, description, params).
# See the fault catalogs in ssos_eclss, ssos_eps and ssos_thermal.
DRILLS = [
    ("ECLSS", [
        _drill("Crew metabolic surge", "crew", "crew_metabolic_surge",
               "Crew CO2, O2 and humidity loads double (EVA prep, heavy work). "
               "Cabin CO2 climbs until the ARS catches up.", scale=2.0),
        _drill("Cabin leak", "cabin", "cabin_leak",
               "A 2 mm hole: about 38 kg/day of air lost. Pressure control "
               "draws N2 from the tanks and the OGS raises O2 production.",
               area_m2=3.0e-6),
        _drill("O2 sensor stuck high", "cabin", "o2_sensor_stuck",
               "The cabin O2 sensor reads 23% regardless of the air. The OGS "
               "throttles back and the analyser flags the disagreement.",
               value_pct=23.0),
        _drill("CHX failure", "cabin", "chx_failure",
               "The condensing heat exchanger stops removing humidity. Cabin RH "
               "rises past 75% in about 6 hours."),
        _drill("CO2 scrubber degraded (ARS)", "ars", "ars_scrubber_degraded",
               "Sorbent capture efficiency drops to 0.2. CO2 removal collapses "
               "and cabin CO2 climbs toward the alarm.", capture_efficiency=0.2),
        _drill("ARS blower failure", "ars", "ars_blower_failure",
               "No process air through the beds: CO2 removal stops."),
        _drill("ARS bed heater failure", "ars", "ars_heater_failure",
               "Beds cannot regenerate. Capture fades as the sorbent saturates; "
               "the ARS flags low removal after about 2 hours."),
        _drill("O2 generation failure (OGS)", "ogs", "ogs_generation_failure",
               "The OGS stack stops. Cabin O2 falls about 0.7 kPa/day until "
               "O2 makeup from the tanks takes over."),
        _drill("UPA failure (WRS)", "wrs", "upa_assembly_failure",
               "The urine processor stops. The wastewater tank fills and potable "
               "water recovery drops."),
        _drill("Multifiltration breakthrough (WRS)", "wrs",
               "wpa_multifiltration_breakthrough",
               "The water processor bed is exhausted: product water goes out of "
               "spec and is recycled. Clearing the drill swaps in a new bed."),
        _drill("Sabatier reactor failure", "sabatier", "sabatier_reactor_failure",
               "No CO2 reduction: all CO2 from the ARS is vented and Sabatier "
               "water to the WRS stops."),
    ]),
    ("EPS", [
        _drill("Solar array degradation", "eps", "solar_array_degraded",
               "Array output drops to 30% of beginning-of-life; the batteries "
               "drain orbit by orbit.", factor=0.3),
        _drill("Solar array gimbal lock", "eps", "array_gimbal_lock",
               "The arrays stop tracking and output falls to 60%."),
        _drill("Battery degradation", "eps", "battery_degraded",
               "Battery state of health drops to 50%; eclipse discharges go twice "
               "as deep.", state_of_health=0.5),
        _drill("Low battery", "eps", "battery_low_soc",
               "The bank is drained to 15%. In daylight the arrays recharge it.",
               soc=0.15),
        _drill("Battery ORU failure", "eps", "battery_oru_failed",
               "Battery ORU 0 fails and is isolated; its channel partner carries "
               "the channel.", oru=0),
        _drill("Battery ORU overheat", "eps", "battery_oru_overheat",
               "Extra heat in ORU 0 takes it past its temperature limit in about "
               "an hour.", oru=0, heat_w=800.0),
        _drill("BCDU fault", "eps", "bcdu_fault",
               "The BCDU on channel 0 isolates its batteries; MBSU 0 reroutes.",
               channel=0),
        _drill("Channel feeder open", "eps", "mbsu_channel_loss",
               "The feeder from channel 0 to MBSU 0 opens; the MBSU reroutes "
               "onto channels 1 and 2.", channel=0),
        _drill("MBSU fault", "eps", "mbsu_fault",
               "MBSU 0 is isolated and its DDCUs lose power.", mbsu=0),
        _drill("DDCU fault", "eps", "ddcu_fault",
               "DDCU 0 trips and its loads are shed.", ddcu=0),
        _drill("DDCU overheat", "eps", "ddcu_overheat",
               "Extra heat in DDCU 0 trips it on over-temperature after about "
               "40 minutes; its loads are shed until it cools.", ddcu=0,
               heat_w=400.0),
        _drill("Load spike", "eps", "load_spike",
               "Every station load doubles; DDCUs over their rating shed their "
               "least critical loads.", demand_scale=2.0),
    ]),
    ("Thermal", [
        _drill("ITCS loop degraded", "thermal", "itcs_loop_degraded",
               "The internal coolant loop runs at 26 C: the EPS cold plate passes "
               "its 28 C limit within about a minute and the cabin warms toward "
               "31 C.", sink_temp_c=26.0),
        _drill("Cold-plate conductance loss", "thermal",
               "coldplate_conductance_loss",
               "The EPS cold plate keeps 35% of its coupling to the coolant and "
               "passes its 28 C limit within a few minutes.", factor=0.35),
    ]),
]

FAULTS = [d for _, drills in DRILLS for d in drills]
TARGETS = sorted({d["target"] for d in FAULTS})


class FaultInjectorDialog(QDialog):
    def __init__(self, node, executor, parent=None):
        super().__init__(parent)
        self.node = node
        self.executor = executor
        self._active_target = None
        self.setWindowTitle("Fault Injection")
        self.resize(620, 560)

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        title = QLabel("RUN A DRILL  ·  one at a time")
        title.setFont(theme.label_font(13, tracking=2.0))
        root.addWidget(title)

        self.list = QListWidget()
        self._rows = []
        for group, drills in DRILLS:
            header = QListWidgetItem(group.upper(), self.list)
            header.setFlags(Qt.NoItemFlags)
            self._rows.append(None)
            for d in drills:
                QListWidgetItem("    " + d["label"], self.list)
                self._rows.append(d)
        self.list.currentRowChanged.connect(self._on_select)
        root.addWidget(self.list, 1)

        self.desc = QLabel("")
        self.desc.setWordWrap(True)
        self.desc.setStyleSheet(f"color: {theme.color('txt3')};")
        self.desc.setMinimumHeight(60)
        root.addWidget(self.desc)

        self.status = QLabel("")
        self.status.setWordWrap(True)
        self.status.setStyleSheet(f"color: {theme.color('txt3')};")
        root.addWidget(self.status)

        btns = QHBoxLayout()
        self.btn_restore = QPushButton("Restore Nominal")
        self.btn_restore.clicked.connect(self._on_restore)
        btns.addWidget(self.btn_restore)
        btns.addStretch()
        self.btn_close = QPushButton("Close")
        self.btn_close.clicked.connect(self.accept)
        btns.addWidget(self.btn_close)
        self.btn_inject = QPushButton("Start Drill")
        self.btn_inject.setDefault(True)
        self.btn_inject.setStyleSheet(
            f"color: {theme.color('red')}; font-weight: bold;")
        self.btn_inject.clicked.connect(self._on_inject)
        btns.addWidget(self.btn_inject)
        root.addLayout(btns)

        self.list.setCurrentRow(1)

    # ---------------- ROS helpers ----------------
    def _call_sync(self, client, req, timeout=2.0):
        if not client.service_is_ready() and \
                not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            try:
                self.executor.spin_once(timeout_sec=0.02)
            except Exception:
                break
        return fut.result() if fut.done() else None

    def _inject(self, target, fault_type, params):
        if not _HAVE_INJECT:
            return None
        cli = self.node.create_client(InjectFault, "/sim/inject_fault")
        req = InjectFault.Request()
        req.target_subsystem = target
        req.fault_type = fault_type
        req.parameters_json = json.dumps(params)
        req.duration_s = 0.0          # until Restore Nominal
        resp = self._call_sync(cli, req)
        self.node.destroy_client(cli)
        return resp

    # ---------------- UI actions ----------------
    def _selected(self):
        row = self.list.currentRow()
        return self._rows[row] if 0 <= row < len(self._rows) else None

    def _on_select(self, _row):
        drill = self._selected()
        self.btn_inject.setEnabled(drill is not None)
        self.desc.setText(drill["desc"] if drill else "")

    def _on_inject(self):
        drill = self._selected()
        if drill is None:
            return
        resp = self._inject(drill["target"], drill["type"], drill["params"])
        if resp is None:
            self.status.setText("Simulation controller not available.")
        elif resp.success:
            self._active_target = drill["target"]
            self.status.setText(f"Drill running: {drill['label']}.")
        else:
            self.status.setText(f"Not started: {resp.message}")

    def _on_restore(self):
        # The running drill's subsystem if we started it; otherwise every
        # subsystem, since the drill may have come from a scenario.
        targets = [self._active_target] if self._active_target else TARGETS
        ok = all(self._inject(t, "clear_faults", {}) is not None for t in targets)
        self._active_target = None
        self.status.setText("Drill cleared; systems returning to nominal." if ok else
                            "Simulation controller not available.")
