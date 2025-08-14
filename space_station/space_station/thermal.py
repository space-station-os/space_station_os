from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from space_station_thermal_control.msg import InternalLoopStatus, ExternalLoopStatus
from PyQt5.QtCore import QMetaObject, Q_ARG


class ThermalWidget(QWidget):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self.init_ui()
        self.setup_subscriptions()

    def init_ui(self):
        self.setStyleSheet("background-color: #2d2d2d; color: white;")
        layout = QHBoxLayout()

        # Left pane (placeholder for graph visualization)
        self.graph_area = QLabel("Thermal Graph Visualization")
        self.graph_area.setAlignment(Qt.AlignCenter)
        self.graph_area.setStyleSheet("border: 1px solid gray; background-color: #1c1c1c;")
        layout.addWidget(self.graph_area, 2)

        # Right panel for info and controls
        right_panel = QVBoxLayout()

        # Status
        self.status_label = QLabel("Thermal system active - Data received")
        self.status_label.setStyleSheet("color: lime;")
        right_panel.addWidget(self.status_label)

        right_panel.addSpacing(10)
        right_panel.addWidget(self._make_divider("Coolant Information"))

        # Data Labels (pass layout explicitly)
        self.ammonia_temp = self._make_data_label("Ammonia Temp", "-- °C", right_panel)
        self.ammonia_pressure = self._make_data_label("Ammonia Pressure", "-- Pa", right_panel)
        self.heater_status = self._make_data_label("Heater Status", "--", right_panel)
        self.loop_a_temp = self._make_data_label("Loop A Temp", "-- °C", right_panel)
        self.loop_b_temp = self._make_data_label("Loop B Temp", "-- °C", right_panel)

        right_panel.addSpacing(10)
        right_panel.addWidget(self._make_divider("Controls"))

        # Buttons
        control_layout = QHBoxLayout()
        vent_btn = QPushButton("Vent Heat")
        refresh_btn = QPushButton("Refresh Water")
        control_layout.addWidget(vent_btn)
        control_layout.addWidget(refresh_btn)
        right_panel.addLayout(control_layout)

        right_panel.addStretch()
        layout.addLayout(right_panel, 1)
        self.setLayout(layout)

    def _make_data_label(self, name, value, parent_layout):
        label = QLabel(f"{name}: {value}")
        label.setFont(QFont("Arial", 10))
        label.setStyleSheet("color: lightgray;")
        parent_layout.addWidget(label)
        return label

    def _make_divider(self, title):
        divider = QLabel(title)
        divider.setFont(QFont("Arial", 11, QFont.Bold))
        divider.setStyleSheet("color: white; margin-top: 10px;")
        return divider

    def setup_subscriptions(self):
        self.node.create_subscription(
            InternalLoopStatus,
            "/tcs/internal_loop_heat",
            self.update_internal,
            10
        )
        self.node.create_subscription(
            ExternalLoopStatus,
            "/tcs/external_loop_a/status",
            self.update_external,
            10
        )

    def update_internal(self, msg: InternalLoopStatus):
        self.node.get_logger().info("[ThermalWidget] Received internal loop data")
        QMetaObject.invokeMethod(self.loop_a_temp, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Loop A Temp: {msg.loop_a.temperature:.1f} °C"))
        QMetaObject.invokeMethod(self.loop_b_temp, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Loop B Temp: {msg.loop_b.temperature:.1f} °C"))

    def update_external(self, msg: ExternalLoopStatus):
        self.node.get_logger().info("[ThermalWidget] Received external loop data")
        QMetaObject.invokeMethod(self.ammonia_temp, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Ammonia Temp: {msg.ammonia_inlet_temp:.1f} °C"))
        QMetaObject.invokeMethod(self.ammonia_pressure, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Ammonia Outlet Temp: {msg.ammonia_outlet_temp:.1f} °C"))
        QMetaObject.invokeMethod(self.heater_status, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Loop Inlet Temp: {msg.loop_inlet_temp:.1f} °C"))
