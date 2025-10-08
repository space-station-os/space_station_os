import subprocess
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton,
    QTableWidget, QTableWidgetItem, QTextEdit, QDialog
)
from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import QTimer
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
        self.log_viewer = None
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
        self.LOG_LEVEL_MAP = {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }
        subsystems = [
            ("ARS", "OK"),
            ("OGS", "OK"),
            ("WRS", "OK"),
            ("Thermal Solver", "OK"),
            ("Coolant Manager", "OK"),
            ("Thermal System", "OK"),
            ("EPS", "OK"),
            ("GNC", "OK"),
            ("Comms Bridge", "OK"),
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
            "ThermalNetwork": "Thermal System",
            "Thermal": "Thermal System",
            "EPS": "EPS",
            "EpsPowerController": "EPS",
            "BCDU": "EPS",
            "BatteryManager": "EPS",
            "MBSU": "EPS",
            "DDCU": "EPS",
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
        self.reconfig_btn = QPushButton("Open rqt_reconfigure")

        self.logs_btn.clicked.connect(self.open_log_viewer)
        self.reconfig_btn.clicked.connect(self.launch_rqt_reconfigure)

        btn_layout.addStretch()
        btn_layout.addWidget(self.check_btn)
        btn_layout.addWidget(self.logs_btn)
        btn_layout.addWidget(self.reconfig_btn)

        layout.addLayout(btn_layout)
        layout.addStretch()
        self.setLayout(layout)

    def init_ros_interfaces(self):
        self.node.create_subscription(DiagnosticStatus, '/ars/diagnostics', self.handle_diagnostic, 10)
        self.node.create_subscription(DiagnosticStatus, '/ogs/diagnostics', self.handle_diagnostic, 10)
        self.node.create_subscription(DiagnosticStatus, '/wrs/diagnostics', self.handle_diagnostic, 10)
        self.node.create_subscription(DiagnosticStatus, '/thermals/diagnostics', self.handle_diagnostic, 10)
        self.node.create_subscription(DiagnosticStatus, '/eps/diagnostics', self.handle_diagnostic, 10)
        self.node.create_subscription(Log, '/rosout', self.rosout_callback, 50)

    def handle_diagnostic(self, msg: DiagnosticStatus):
        alias = self.diagnostic_aliases.get(msg.name, None)
        if alias is None or alias.upper() not in self.subsystem_map:
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
            "level": self.LOG_LEVEL_MAP.get(msg.level, str(msg.level)),
            "name": msg.name,
            "msg": msg.msg
        }
        self.log_buffer.append(log_entry)
        if len(self.log_buffer) > 500:
            self.log_buffer = self.log_buffer[-500:]

    def open_log_viewer(self):
        self.log_viewer = LogViewerDialog(self.log_buffer, self)
        self.log_viewer.show()

    def launch_rqt_reconfigure(self):
        """Launch rqt_reconfigure as a subprocess."""
        try:
            subprocess.Popen(["rqt"])

        except FileNotFoundError:
            self.node.get_logger().error("rqt not found. Please install with: sudo apt install ros-humble-rqt-reconfigure")
        except Exception as e:
            self.node.get_logger().error(f"Failed to launch rqt_reconfigure: {e}")
