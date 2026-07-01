#!/usr/bin/env python3
"""Fault-injection dialog.

A small operator tool to trigger a chosen ECLSS fault. Each fault maps to a
recipe of (a) an announcement via the simulation controller's /sim/inject_fault
service (published as a FaultEvent the system manager + event feed pick up) and
(b) live parameter changes on the target node that produce the actual physical
effect. "Restore Nominal" reverts every recipe's parameters.
"""
import time

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QPushButton
)
from PyQt5.QtCore import Qt

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType

try:
    from space_station_interfaces.srv import InjectFault
    _HAVE_INJECT = True
except Exception:
    InjectFault = None
    _HAVE_INJECT = False

from space_station import theme

# Each fault: apply/restore are (node, param, value) triples applied live via
# SetParameters; the /sim/inject_fault call announces it on /ssos/fault_event.
FAULTS = [
    {
        "label": "O₂ Generation Failure (OGS)",
        "desc": "OGS electrolysis stack current → 0 A. O₂ production stops, the "
                "cabin O₂ balance turns negative, and the OGS reports a fault.",
        "target": "ogs", "type": "ogs_generation_failure", "params_json": "{}",
        "apply": [("ogs_node", "stack_current_a", 0.0),
                  ("ogs_node", "enable_auto_faults", True)],
        "restore": [("ogs_node", "stack_current_a", 27.0),
                    ("ogs_node", "enable_auto_faults", False)],
    },
    {
        "label": "UPA Assembly Failure (WRS)",
        "desc": "Urine Processor stops (throughput → 0 kg/day). The wastewater "
                "tank fills and potable water steadily declines.",
        "target": "wrs", "type": "upa_assembly_failure", "params_json": "{}",
        "apply": [("wrs_node", "max_urine_process_kg_day", 0.0)],
        "restore": [("wrs_node", "max_urine_process_kg_day", 13.6)],
    },
    {
        "label": "CO₂ Scrubber Degraded (ARS)",
        "desc": "ARS capture efficiency → 0.2. CO₂ removal collapses and cabin "
                "CO₂ climbs toward the alarm; ARS and cabin report faults.",
        "target": "ars", "type": "ars_scrubber_degraded", "params_json": "{}",
        "apply": [("ars_node", "ars.efficiency.capture_efficiency", 0.2),
                  ("ars_node", "enable_auto_faults", True),
                  ("cabin_node", "enable_auto_faults", True)],
        "restore": [("ars_node", "ars.efficiency.capture_efficiency", 0.84),
                    ("ars_node", "enable_auto_faults", False),
                    ("cabin_node", "enable_auto_faults", False)],
    },
    {
        "label": "O₂ Sensor Stuck (inject)",
        "desc": "Injects a stuck cabin O₂-sensor fault via /sim/inject_fault, "
                "announced to the system manager and event feed.",
        "target": "eclss", "type": "sensor_stuck_at",
        "params_json": '{"stuck_value": 15.0}',
        "apply": [], "restore": [],
    },
]


class FaultInjectorDialog(QDialog):
    def __init__(self, node, executor, parent=None):
        super().__init__(parent)
        self.node = node
        self.executor = executor
        self.setWindowTitle("Fault Injection")
        self.resize(560, 460)

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        title = QLabel("INJECT FAULT")
        title.setFont(theme.label_font(13, tracking=2.0))
        root.addWidget(title)

        self.list = QListWidget()
        for f in FAULTS:
            QListWidgetItem(f["label"], self.list)
        self.list.setCurrentRow(0)
        self.list.currentRowChanged.connect(self._on_select)
        root.addWidget(self.list, 1)

        self.desc = QLabel("")
        self.desc.setWordWrap(True)
        self.desc.setStyleSheet(f"color: {theme.color('txt3')};")
        self.desc.setMinimumHeight(60)
        root.addWidget(self.desc)

        self.status = QLabel("")
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
        self.btn_inject = QPushButton("Inject Fault")
        self.btn_inject.setDefault(True)
        self.btn_inject.setStyleSheet(
            f"color: {theme.color('red')}; font-weight: bold;")
        self.btn_inject.clicked.connect(self._on_inject)
        btns.addWidget(self.btn_inject)
        root.addLayout(btns)

        self._on_select(0)

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

    @staticmethod
    def _param_value(v):
        if isinstance(v, bool):
            return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
        if isinstance(v, int):
            return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
        if isinstance(v, float):
            return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(v))

    def _set_params(self, triples):
        by_node = {}
        for node_name, name, val in triples:
            by_node.setdefault(node_name, []).append(
                RclParameter(name=name, value=self._param_value(val)))
        ok = True
        for node_name, params in by_node.items():
            cli = self.node.create_client(
                SetParameters, f"/{node_name}/set_parameters")
            req = SetParameters.Request()
            req.parameters = params
            resp = self._call_sync(cli, req)
            self.node.destroy_client(cli)
            if resp is None or not all(r.successful for r in resp.results):
                ok = False
        return ok

    def _inject(self, fault):
        if not _HAVE_INJECT:
            return None
        cli = self.node.create_client(InjectFault, "/sim/inject_fault")
        req = InjectFault.Request()
        req.target_subsystem = fault["target"]
        req.fault_type = fault["type"]
        req.parameters_json = fault.get("params_json", "{}")
        req.duration_s = -1.0
        resp = self._call_sync(cli, req)
        self.node.destroy_client(cli)
        return resp

    # ---------------- UI actions ----------------
    def _on_select(self, row):
        if 0 <= row < len(FAULTS):
            self.desc.setText(FAULTS[row]["desc"])

    def _on_inject(self):
        row = self.list.currentRow()
        if not (0 <= row < len(FAULTS)):
            return
        fault = FAULTS[row]
        resp = self._inject(fault)
        params_ok = self._set_params(fault["apply"]) if fault["apply"] else True
        announced = "announced" if resp is not None else "no sim service"
        if params_ok:
            self.status.setText(f"Injected '{fault['label']}' ({announced}).")
        else:
            self.status.setText(
                f"'{fault['label']}' {announced}, but some parameters were "
                f"rejected (is the node running?).")

    def _on_restore(self):
        triples = []
        for f in FAULTS:
            triples.extend(f["restore"])
        ok = self._set_params(triples) if triples else True
        self.status.setText(
            "Restored nominal parameters." if ok else
            "Restore incomplete — some nodes may be unavailable.")
