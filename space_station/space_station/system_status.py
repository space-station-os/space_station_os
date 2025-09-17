# space_station/system_status.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QTextEdit, QDialog
)
from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import Qt, QTimer
from diagnostic_msgs.msg import DiagnosticStatus
from rcl_interfaces.msg import Log


class LogViewerDialog(QDialog):
    def __init__(self, log_buffer, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ROS Logs - /rosout")
        self.setMinimumSize(700, 400)
        self.log_buffer = log_buffer

        layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        self.setLayout(layout)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.update_logs)
        self.refresh_timer.start(5000)

        self.update_logs()

    def update_logs(self):
        self.log_text.clear()
        last_logs = self.log_buffer[-100:]
        for log in last_logs:
            self.log_text.append(f"[{log['level']}] [{log['name']}] {log['msg']}")


class SystemStatusWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self.subsystem_map = {}
        self.log_buffer = []
        self.init_ui()
        self.init_ros_interfaces()

    def init_ui(self):
        layout = QVBoxLayout()

        title = QLabel("System Diagnostics & Status")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)
        layout.addSpacing(10)

        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Subsystem", "Status"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #2a2a2a;
                color: white;
                gridline-color: #444;
            }
            QHeaderView::section {
                background-color: #333;
                color: white;
            }
        """)

        subsystems = [
            ("ARS", "OK"),
            ("OGS", "OK"),
            ("WRS", "OK"),
            ("Thermal Solver", "OK"),
            ("Coolant Manager", "OK"),
            ("GNC", "OK"),
            ("Comms Bridge", "OK")
        ]
        self.diagnostic_aliases = {
            "WRS": "WRS",
            "ARS": "ARS",
            "OGS": "OGS",
            "WasteCollector": "WRS",
            "Sabatier Reactor": "ARS",
            "OxygenGenUnit": "OGS",
            "CO2Sensor": "ARS",
            "O2Tank": "OGS",
            "ProductWaterTank": "WRS",
            "CoolantManager": "Coolant Manager",
            "ThermalSolver": "Thermal Solver",
            "GNC": "GNC",
            "Comms Bridge": "Comms Bridge",
            "CommsBridge": "Comms Bridge",
        }

        self.table.setRowCount(len(subsystems))
        for i, (name, status) in enumerate(subsystems):
            name_item = QTableWidgetItem(name)
            status_item = QTableWidgetItem(status)
            status_item.setForeground(QColor("lightgreen"))
            self.table.setItem(i, 0, name_item)
            self.table.setItem(i, 1, status_item)
            self.subsystem_map[name.upper()] = i

        layout.addWidget(self.table)

        btn_layout = QHBoxLayout()
        self.check_btn = QPushButton("Run Self-Check")
        self.logs_btn = QPushButton("View Logs")
        self.logs_btn.clicked.connect(self.open_log_viewer)
        btn_layout.addStretch()
        btn_layout.addWidget(self.check_btn)
        btn_layout.addWidget(self.logs_btn)

        layout.addLayout(btn_layout)
        layout.addStretch()
        self.setLayout(layout)

    def init_ros_interfaces(self):
        self.node.create_subscription(DiagnosticStatus, '/ars/diagnostics', self.ars_callback, 10)
        self.node.create_subscription(DiagnosticStatus, '/ogs/diagnostics', self.ogs_callback, 10)
        self.node.create_subscription(DiagnosticStatus, '/wrs/diagnostics', self.wrs_callback, 10)
        self.node.create_subscription(Log, '/rosout', self.rosout_callback, 50)

    def ars_callback(self, msg: DiagnosticStatus):
        self.handle_diagnostic(msg)

    def ogs_callback(self, msg: DiagnosticStatus):
        self.handle_diagnostic(msg)

    def wrs_callback(self, msg: DiagnosticStatus):
        self.handle_diagnostic(msg)

    def handle_diagnostic(self, msg: DiagnosticStatus):
        # self.node.get_logger().info(f"[SYSTEM STATUS] Received diagnostic for {msg.name}: {msg.message}")
        alias = self.diagnostic_aliases.get(msg.name, None)
        if alias is None or alias.upper() not in self.subsystem_map:
            # self.node.get_logger().warn(f"[SYSTEM STATUS] Unknown subsystem: {msg.name}")
            return
        subsystem_name = alias.upper()

        try:
            level = int(msg.level)
        except Exception:
            level = DiagnosticStatus.STALE

        if level == DiagnosticStatus.OK:
            status_text = "OK"
            color = QColor("lightgreen")
        elif level == DiagnosticStatus.WARN:
            status_text = "WARNING"
            color = QColor("orange")
        elif level == DiagnosticStatus.ERROR:
            status_text = "FAILURE"
            color = QColor("red")
        else:
            status_text = "UNKNOWN"
            color = QColor("gray")

        for name_key in self.subsystem_map:
            if name_key in subsystem_name:
                row = self.subsystem_map[name_key]
                QTimer.singleShot(0, lambda r=row, text=status_text, col=color: self.update_status_row(r, text, col))
                break

    def update_status_row(self, row: int, text: str, color: QColor):
        status_item = self.table.item(row, 1)
        if status_item:
            status_item.setText(text)
            status_item.setForeground(color)

    def rosout_callback(self, msg: Log):
        log_entry = {
            "level": msg.level,
            "name": msg.name,
            "msg": msg.msg
        }
        self.log_buffer.append(log_entry)
        if len(self.log_buffer) > 500:
            self.log_buffer = self.log_buffer[-500:]

    def open_log_viewer(self):
        self.log_viewer = LogViewerDialog(self.log_buffer, self)
        self.log_viewer.show()
