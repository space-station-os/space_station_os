
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout,
    QSplitter, QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import Qt, QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from collections import deque
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from space_station_thermal_control.msg import ThermalNodeDataArray, ThermalLinkFlowsArray
from space_station_thermal_control.action import Coolant


class ThermalWidget(QWidget):
    def __init__(self, gui_node: Node, parent=None):
        super().__init__(parent)
        self.node = gui_node
        self.node.get_logger().info("[ThermalWidget] Quadrant UI Initializing (PyQt5)")

        # --- Cached state ---
        self._lock = threading.Lock()
        self.thermal_nodes = {}   # {name: temperature °C}
        self.thermal_links = []   # [{"a":, "b":, "flow":}, ...]
        self.coolant_status = {
            "internal_temp_c": None,
            "ammonia_temp_c": None,
            "vented_heat_kj": None
        }
        self._time_counter = 0

        # histories for avg temp plot
        self.temp_time = deque(maxlen=300)
        self.avg_temp_history = deque(maxlen=300)

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
        root_splitter = QSplitter(Qt.Vertical)
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(root_splitter)

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

        # --- Bottom half: Coolant + Plot ---
        bottom_splitter = QSplitter(Qt.Horizontal)
        root_splitter.addWidget(bottom_splitter)

        # Quadrant 3: Coolant Status (from action feedback)
        coolant_group = QGroupBox("Coolant Status (Feedback)")
        coolant_layout = QFormLayout()
        self.coolant_internal_temp = QLabel("--")
        self.coolant_ammonia_temp = QLabel("--")
        self.coolant_vented_heat = QLabel("--")
        coolant_layout.addRow("Internal Temp (°C):", self.coolant_internal_temp)
        coolant_layout.addRow("Ammonia Temp (°C):", self.coolant_ammonia_temp)
        coolant_layout.addRow("Vented Heat (kJ):", self.coolant_vented_heat)
        coolant_group.setLayout(coolant_layout)
        bottom_splitter.addWidget(coolant_group)

        # Quadrant 4: Avg Temp Plot
        plot_group = QGroupBox("Avg Node Temp (°C)")
        plot_layout = QVBoxLayout()
        fig, self.ax = plt.subplots(figsize=(4, 3), dpi=100)
        self.canvas = FigureCanvas(fig)
        (self.temp_line,) = self.ax.plot([], [], 'r-')
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temperature (°C)")
        self.ax.grid(True)
        plot_layout.addWidget(self.canvas)
        plot_group.setLayout(plot_layout)
        bottom_splitter.addWidget(plot_group)

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

        self.coolant_client.wait_for_server()
        self.coolant_client.send_goal_async(
            goal_msg,
            feedback_callback=self._coolant_feedback_cb
        )

    # ------------------- Callbacks -------------------
    def _node_cb(self, msg: ThermalNodeDataArray):
        with self._lock:
            # Node temps are assumed to be in °C already
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
            self.coolant_internal_temp.setText(f"{coolant['internal_temp_c']:.1f} °C")
        if coolant["ammonia_temp_c"] is not None:
            self.coolant_ammonia_temp.setText(f"{coolant['ammonia_temp_c']:.1f} °C")
        if coolant["vented_heat_kj"] is not None:
            self.coolant_vented_heat.setText(f"{coolant['vented_heat_kj']:.1f} kJ")

        # Update avg temp plot
        if nodes:
            avg_temp_c = sum(nodes.values()) / len(nodes)
            self._time_counter += 1
            self.temp_time.append(self._time_counter)
            self.avg_temp_history.append(avg_temp_c)

            self.ax.cla()
            self.ax.plot(self.temp_time, self.avg_temp_history, 'r-')
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("Temperature (°C)")
            self.ax.set_title("Avg Node Temp (°C)")
            self.ax.grid(True)

            # auto-scale y axis around the data
            ymin = min(self.avg_temp_history) - 2
            ymax = max(self.avg_temp_history) + 2
            self.ax.set_ylim(ymin, ymax)

            self.canvas.draw()
