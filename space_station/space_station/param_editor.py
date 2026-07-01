#!/usr/bin/env python3
"""Live parameter editor dialog.

Discovers parameters from the running sim + ECLSS nodes via the ROS 2 parameter
services (list/get/set_parameters) and pushes edits live. Service futures are
resolved by pumping the GUI's executor with a bounded timeout; safe because the
dialog runs in Qt event context, not inside a ROS spin.
"""
import time

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTreeWidget, QTreeWidgetItem,
    QPushButton, QLabel, QHeaderView, QMessageBox
)
from PyQt5.QtCore import Qt

from rcl_interfaces.srv import ListParameters, GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType

from space_station import theme

# Nodes whose parameters we expose. Order = display order.
_TARGET_NODES = [
    "simulation_controller",
    "cabin_node",
    "ars_node",
    "ogs_node",
    "wrs_node",
    "sabatier_node",
]

_TYPE_NAMES = {
    ParameterType.PARAMETER_BOOL: "bool",
    ParameterType.PARAMETER_INTEGER: "int",
    ParameterType.PARAMETER_DOUBLE: "double",
    ParameterType.PARAMETER_STRING: "string",
}


class ParameterEditorDialog(QDialog):
    def __init__(self, node, executor, parent=None):
        super().__init__(parent)
        self.node = node
        self.executor = executor
        self.setWindowTitle("Simulation Parameters")
        self.resize(640, 720)
        # (node, name) -> (param_type, original_text)
        self._original = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        title = QLabel("LIVE PARAMETERS")
        title.setFont(theme.label_font(13, tracking=2.0))
        root.addWidget(title)
        hint = QLabel("Edit a value and press Apply to push it to the running "
                      "node. Changed rows are highlighted.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {theme.color('txt3')};")
        root.addWidget(hint)

        self.tree = QTreeWidget()
        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(["Parameter", "Type", "Value"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.tree.itemChanged.connect(self._on_item_changed)
        root.addWidget(self.tree, 1)

        self.status = QLabel("")
        self.status.setStyleSheet(f"color: {theme.color('txt3')};")
        root.addWidget(self.status)

        btns = QHBoxLayout()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self.populate)
        btns.addWidget(self.btn_refresh)
        btns.addStretch()
        self.btn_apply = QPushButton("Apply Changes")
        self.btn_apply.setDefault(True)
        self.btn_apply.clicked.connect(self._apply)
        btns.addWidget(self.btn_apply)
        self.btn_close = QPushButton("Close")
        self.btn_close.clicked.connect(self.accept)
        btns.addWidget(self.btn_close)
        root.addLayout(btns)

        self.populate()

    # ---------------- ROS sync helper ----------------
    def _call_sync(self, client, req, timeout=2.0):
        """Send a request and pump the executor until the future resolves."""
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

    def _client(self, node_name, srv_type, srv_name):
        return self.node.create_client(srv_type, f"/{node_name}/{srv_name}")

    # ---------------- populate ----------------
    def populate(self):
        self.tree.blockSignals(True)
        self.tree.clear()
        self._original.clear()
        found_any = False

        for node_name in _TARGET_NODES:
            list_cli = self._client(node_name, ListParameters, "list_parameters")
            resp = self._call_sync(list_cli, ListParameters.Request(), timeout=1.0)
            self.node.destroy_client(list_cli)
            if resp is None:
                continue
            names = sorted(resp.result.names)
            # Skip the implicit/uninteresting ones.
            names = [n for n in names
                     if not n.startswith("qos_overrides")
                     and n != "use_sim_time"]
            if not names:
                continue

            get_cli = self._client(node_name, GetParameters, "get_parameters")
            greq = GetParameters.Request()
            greq.names = names
            gresp = self._call_sync(get_cli, greq, timeout=1.0)
            self.node.destroy_client(get_cli)
            if gresp is None:
                continue

            found_any = True
            parent = QTreeWidgetItem(self.tree, [node_name, "", ""])
            parent.setFirstColumnSpanned(True)
            f = parent.font(0)
            f.setBold(True)
            parent.setFont(0, f)
            parent.setExpanded(True)

            for name, pval in zip(names, gresp.values):
                text, editable = self._value_to_text(pval)
                tname = _TYPE_NAMES.get(pval.type, "array")
                item = QTreeWidgetItem(parent, [name, tname, text])
                item.setData(0, Qt.UserRole, (node_name, name, pval.type))
                if editable:
                    item.setFlags(item.flags() | Qt.ItemIsEditable)
                else:
                    item.setForeground(2, Qt.gray)
                self._original[(node_name, name)] = text

        self.tree.blockSignals(False)
        if not found_any:
            self.status.setText("No nodes reachable — is the simulation running?")
        else:
            self.status.setText("Ready.")

    @staticmethod
    def _value_to_text(pval):
        """(text, editable) for a ParameterValue."""
        t = pval.type
        if t == ParameterType.PARAMETER_BOOL:
            return ("true" if pval.bool_value else "false", True)
        if t == ParameterType.PARAMETER_INTEGER:
            return (str(pval.integer_value), True)
        if t == ParameterType.PARAMETER_DOUBLE:
            return (repr(pval.double_value), True)
        if t == ParameterType.PARAMETER_STRING:
            return (pval.string_value, True)
        # Arrays / unknown: show but keep read-only.
        for attr in ("integer_array_value", "double_array_value",
                     "bool_array_value", "string_array_value",
                     "byte_array_value"):
            v = getattr(pval, attr, None)
            if v:
                return (", ".join(str(x) for x in v), False)
        return ("", False)

    def _on_item_changed(self, item, col):
        if col != 2:
            return
        meta = item.data(0, Qt.UserRole)
        if meta is None:
            return
        node_name, name = meta[0], meta[1]
        changed = item.text(2) != self._original.get((node_name, name))
        # Highlight changed rows.
        item.setForeground(2, Qt.yellow if changed else Qt.white)

    # ---------------- apply ----------------
    def _apply(self):
        # Collect changed values grouped by node.
        by_node = {}
        errors = []
        it = self.tree.invisibleRootItem()
        for i in range(it.childCount()):
            parent = it.child(i)
            for j in range(parent.childCount()):
                item = parent.child(j)
                meta = item.data(0, Qt.UserRole)
                if meta is None:
                    continue
                node_name, name, ptype = meta
                new_text = item.text(2)
                if new_text == self._original.get((node_name, name)):
                    continue
                try:
                    pv = self._text_to_value(ptype, new_text)
                except ValueError as e:
                    errors.append(f"{node_name}/{name}: {e}")
                    continue
                by_node.setdefault(node_name, []).append(
                    RclParameter(name=name, value=pv))

        if errors:
            QMessageBox.warning(self, "Invalid values", "\n".join(errors))
            return
        if not by_node:
            self.status.setText("No changes to apply.")
            return

        applied, failed = 0, 0
        for node_name, params in by_node.items():
            cli = self._client(node_name, SetParameters, "set_parameters")
            req = SetParameters.Request()
            req.parameters = params
            resp = self._call_sync(cli, req, timeout=2.0)
            self.node.destroy_client(cli)
            if resp is None:
                failed += len(params)
                continue
            for p, r in zip(params, resp.results):
                if r.successful:
                    applied += 1
                    self._original[(node_name, p.name)] = \
                        self._value_text_for(p.value)
                else:
                    failed += 1
                    errors.append(f"{node_name}/{p.name}: "
                                  f"{r.reason or 'rejected'}")

        msg = f"Applied {applied} parameter(s)."
        if failed:
            msg += f" {failed} failed."
        self.status.setText(msg)
        if errors:
            QMessageBox.warning(self, "Some updates rejected", "\n".join(errors))
        # Re-colour rows to reflect the new baseline.
        self.populate()

    @staticmethod
    def _text_to_value(ptype, text):
        text = text.strip()
        if ptype == ParameterType.PARAMETER_BOOL:
            low = text.lower()
            if low in ("true", "1", "yes", "on"):
                return ParameterValue(type=ptype, bool_value=True)
            if low in ("false", "0", "no", "off"):
                return ParameterValue(type=ptype, bool_value=False)
            raise ValueError("expected true/false")
        if ptype == ParameterType.PARAMETER_INTEGER:
            return ParameterValue(type=ptype, integer_value=int(text))
        if ptype == ParameterType.PARAMETER_DOUBLE:
            return ParameterValue(type=ptype, double_value=float(text))
        if ptype == ParameterType.PARAMETER_STRING:
            return ParameterValue(type=ptype, string_value=text)
        raise ValueError("read-only type")

    @staticmethod
    def _value_text_for(pv):
        return ParameterEditorDialog._value_to_text(pv)[0]
