from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QSplitter, QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import Qt, QTimer
from collections import deque
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from space_station_interfaces.msg import ThermalNodeDataArray, ThermalLinkFlowsArray
from space_station_interfaces.action import Coolant

from space_station import theme
from space_station.widgets import page_header, MetricCard, SpecRow, TelemetryPlot


class ThermalWidget(QWidget):
    def __init__(self, gui_node: Node, parent=None):
        super().__init__(parent)
        self.node = gui_node
        self.node.get_logger().info("[ThermalWidget] Quadrant UI Initializing (PyQtGraph)")

        # --- Cached state ---
        self._lock = threading.Lock()
        self.thermal_nodes = {}   # {name: temperature °C}
        self.thermal_links = []   # [{"a":, "b":, "flow":}, ...]
        self.coolant_status = {
            "internal_temp_c": None,
            "ammonia_temp_c": None,
            "vented_heat_kj": None
        }

        # histories for avg temp plot
        self.temp_time = deque(maxlen=300)
        self.avg_temp_history = deque(maxlen=300)
        self._time_counter = 0

        # --- Build UI ---
        self._build_ui()

        # --- ROS Interfaces ---
        self._init_ros_interfaces()

        # --- Timer for GUI updates ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_gui)
        self.timer.start(1000)  # update at 1 Hz

    # ------------------- UI -------------------
    def _build_ui(self):
        layout = QVBoxLayout(self)

        # Page header (Vast Space design system)
        layout.addWidget(page_header(
            "Thermal Control",
            "Internal/External Loops · Node Network",
            "TCS",
        ))

        # Coolant status as a SpecRow of MetricCards
        self.card_internal = MetricCard("Internal Temp", "—", "°C", "NO DATA", "muted")
        self.card_ammonia = MetricCard("Ammonia Temp", "—", "°C", "NO DATA", "muted")
        self.card_vented = MetricCard("Vented Heat", "—", "kJ", "NO DATA", "muted")
        layout.addWidget(SpecRow([self.card_internal, self.card_ammonia, self.card_vented]))

        root_splitter = QSplitter(Qt.Vertical)
        layout.addWidget(root_splitter)

        # --- Top half: Nodes + Links ---
        top_splitter = QSplitter(Qt.Horizontal)
        root_splitter.addWidget(top_splitter)

        # Quadrant 1: Thermal Nodes
        nodes_group = QGroupBox("Thermal Nodes")
        self.nodes_table = QTableWidget(0, 2)
        self.nodes_table.setHorizontalHeaderLabels(["Node", "Temp (°C)"])
        nodes_layout = QVBoxLayout()
        nodes_layout.addWidget(self.nodes_table)
        nodes_group.setLayout(nodes_layout)
        top_splitter.addWidget(nodes_group)

        # Quadrant 2: Thermal Links
        links_group = QGroupBox("Thermal Links")
        self.links_table = QTableWidget(0, 3)
        self.links_table.setHorizontalHeaderLabels(["Node A", "Node B", "Heat Flow (W)"])
        links_layout = QVBoxLayout()
        links_layout.addWidget(self.links_table)
        links_group.setLayout(links_layout)
        top_splitter.addWidget(links_group)

        # --- Bottom half: Avg Temp Plot ---
        plot_group = QGroupBox("Avg Node Temperature (°C)")
        plot_layout = QVBoxLayout()

        self.temp_plot = TelemetryPlot(max_points=300, y_label="°C")
        plot_layout.addWidget(self.temp_plot)
        plot_group.setLayout(plot_layout)
        root_splitter.addWidget(plot_group)

    # ------------------- ROS -------------------
    def _init_ros_interfaces(self):
        # Nodes
        self.node.create_subscription(
            ThermalNodeDataArray, "/thermal/nodes/state", self._node_cb, 10
        )
        # Links
        self.node.create_subscription(
            ThermalLinkFlowsArray, "/thermal/links/flux", self._link_cb, 10
        )

        # Coolant Action Client (for feedback)
        self.coolant_client = ActionClient(self.node, Coolant, "coolant_heat_transfer")

        goal_msg = Coolant.Goal()
        goal_msg.component_id = "thermal_gui"
        goal_msg.input_temperature_c = 30.0  # arbitrary test value

        # timeout_sec keeps the GUI from blocking forever when no server is up
        if self.coolant_client.wait_for_server(timeout_sec=2.0):
            self.coolant_client.send_goal_async(
                goal_msg,
                feedback_callback=self._coolant_feedback_cb
            )
        else:
            self.node.get_logger().warn(
                "[ThermalWidget] Coolant action server unavailable; skipping goal"
            )

    # ------------------- Callbacks -------------------
    def _node_cb(self, msg: ThermalNodeDataArray):
        with self._lock:
            self.thermal_nodes = {n.name: n.temperature for n in msg.nodes}

    def _link_cb(self, msg: ThermalLinkFlowsArray):
        with self._lock:
            self.thermal_links = [
                {"a": l.node_a, "b": l.node_b, "flow": l.heat_flow} for l in msg.links
            ]

    def _coolant_feedback_cb(self, msg: Coolant.Feedback):
        with self._lock:
            self.coolant_status["internal_temp_c"] = msg.internal_temp_c
            self.coolant_status["ammonia_temp_c"] = msg.ammonia_temp_c
            self.coolant_status["vented_heat_kj"] = msg.vented_heat_kj

    # ------------------- GUI Update -------------------
    def _update_gui(self):
        with self._lock:
            nodes = dict(self.thermal_nodes)
            links = list(self.thermal_links)
            coolant = dict(self.coolant_status)

        # Update nodes table
        self.nodes_table.setRowCount(len(nodes))
        for i, (name, temp_c) in enumerate(nodes.items()):
            self.nodes_table.setItem(i, 0, QTableWidgetItem(name))
            self.nodes_table.setItem(i, 1, QTableWidgetItem(f"{temp_c:.1f}"))

        # Update links table
        self.links_table.setRowCount(len(links))
        for i, l in enumerate(links):
            self.links_table.setItem(i, 0, QTableWidgetItem(l["a"]))
            self.links_table.setItem(i, 1, QTableWidgetItem(l["b"]))
            self.links_table.setItem(i, 2, QTableWidgetItem(f"{l['flow']:.2f}"))

        # Update coolant info
        if coolant["internal_temp_c"] is not None:
            self.card_internal.set_value(f"{coolant['internal_temp_c']:.1f}")
            self.card_internal.set_footer("LIVE", "green")
        if coolant["ammonia_temp_c"] is not None:
            self.card_ammonia.set_value(f"{coolant['ammonia_temp_c']:.1f}")
            self.card_ammonia.set_footer("LIVE", "green")
        if coolant["vented_heat_kj"] is not None:
            self.card_vented.set_value(f"{coolant['vented_heat_kj']:.1f}")
            self.card_vented.set_footer("LIVE", "green")

        # Update avg temp plot
        if nodes:
            avg_temp_c = sum(nodes.values()) / len(nodes)
            self._time_counter += 1
            self.temp_time.append(self._time_counter)
            self.avg_temp_history.append(avg_temp_c)

            self.temp_plot.add_point(self._time_counter, avg_temp_c)
