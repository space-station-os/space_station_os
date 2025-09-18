from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QPushButton, QGroupBox, QGridLayout, QProgressBar, QHBoxLayout
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer, QMetaObject, Q_ARG
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64
from space_station_eclss.action import AirRevitalisation, WaterRecovery
from space_station_eclss.srv import O2Request, RequestProductWater
import random

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
# Constants
MAX_O2_STORAGE = 60000.0  # grams
MAX_WATER_STORAGE = 2000.0  # liters
MAX_CO2_STORAGE = 7000.0  # mmHg
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,   
    durability=DurabilityPolicy.TRANSIENT_LOCAL 
)
class EclssWidget(QWidget):
    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self.node = node
        self.init_ui()
        self.init_ros_interfaces()

        # Simulation state
        self.co2_level = 400.0
        self.urine_volume = 0.0
        self.tick = 0
        self.day = 1
        self.crew_size = 4
        self.total_days = 180
        self.exercise_event = 3

        # Disable manual buttons
        self.ars_button.setEnabled(False)
        self.ogs_button.setEnabled(False)
        self.wrs_button.setEnabled(False)

        self.timer = QTimer()
        self.timer.timeout.connect(self.simulate_event)
        self.timer.start(2000)

    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("Environmental Control & Life Support System (ECLSS)")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)

        grid = QGridLayout()

        # ARS Group
        ars_group = QGroupBox("Air Revitalization (ARS)")
        ars_group.setStyleSheet("color: white;")
        ars_layout = QVBoxLayout()
        self.co2_status = QLabel("CO₂: --- mmHg")
        self.co2_status.setStyleSheet("color: lightgray;")
        self.ars_button = QPushButton("Vent CO₂")
        self.ars_button.setStyleSheet("background-color: #444; color: white;")
        self.ars_button.clicked.connect(self.send_ars_goal)
        self.co2_bar = QProgressBar()
        self.co2_bar.setOrientation(Qt.Vertical)
        self.co2_bar.setRange(0, 100)
        self.co2_bar.setFixedWidth(40)
        self.co2_bar.setStyleSheet("QProgressBar::chunk { background-color: #2ecc71; }")
        ars_layout.addWidget(self.co2_status)
        ars_layout.addWidget(self.ars_button)
        ars_layout.addWidget(self.co2_bar, alignment=Qt.AlignHCenter)
        ars_group.setLayout(ars_layout)
        grid.addWidget(ars_group, 0, 0)

        # OGS Group
        ogs_group = QGroupBox("Oxygen Generation (OGS)")
        ogs_group.setStyleSheet("color: white;")
        ogs_layout = QVBoxLayout()
        self.o2_status = QLabel("O₂ Reserve: --- %")
        self.o2_status.setStyleSheet("color: lightgray;")
        self.ogs_button = QPushButton("Request O₂")
        self.ogs_button.setStyleSheet("background-color: #444; color: white;")
        self.ogs_button.clicked.connect(self.send_ogs_service)
        self.o2_bar = QProgressBar()
        self.o2_bar.setOrientation(Qt.Vertical)
        self.o2_bar.setRange(0, 100)
        self.o2_bar.setFixedWidth(40)
        self.o2_bar.setStyleSheet("QProgressBar::chunk { background-color: #3498db; }")
        ogs_layout.addWidget(self.o2_status)
        ogs_layout.addWidget(self.ogs_button)
        ogs_layout.addWidget(self.o2_bar, alignment=Qt.AlignHCenter)
        ogs_group.setLayout(ogs_layout)
        grid.addWidget(ogs_group, 0, 1)

        # WRS Group
        wrs_group = QGroupBox("Water Recovery (WRS)")
        wrs_group.setStyleSheet("color: white;")
        wrs_layout = QVBoxLayout()
        self.water_status = QLabel("Water Tank: --- % Full")
        self.water_status.setStyleSheet("color: lightgray;")
        self.wrs_button = QPushButton("Start Recycling")
        self.wrs_button.setStyleSheet("background-color: #444; color: white;")
        self.wrs_button.clicked.connect(self.send_wrs_goal)
        self.water_bar = QProgressBar()
        self.water_bar.setOrientation(Qt.Vertical)
        self.water_bar.setRange(0, 100)
        self.water_bar.setFixedWidth(40)
        self.water_bar.setStyleSheet("QProgressBar::chunk { background-color: #9b59b6; }")
        wrs_layout.addWidget(self.water_status)
        wrs_layout.addWidget(self.wrs_button)
        wrs_layout.addWidget(self.water_bar, alignment=Qt.AlignHCenter)
        wrs_group.setLayout(wrs_layout)
        grid.addWidget(wrs_group, 0, 2)

        layout.addLayout(grid)
        layout.addStretch()
        self.setLayout(layout)

    def init_ros_interfaces(self):
        self.ars_client = ActionClient(self.node, AirRevitalisation, '/air_revitalisation')
        self.wrs_client = ActionClient(self.node, WaterRecovery, '/water_recovery_systems')
        self.o2_client = self.node.create_client(O2Request, '/ogs/request_o2')
        self.water_client = self.node.create_client(RequestProductWater, '/wrs/product_water_request')

        self.node.create_subscription(Float64, '/co2_storage', self.update_co2_status, 10)
        self.node.create_subscription(Float64, '/o2_storage', self.update_o2_status, 10)
        self.node.create_subscription(Float64, '/wrs/product_water_reserve', self.update_water_status, 10)

    def simulate_event(self):
        if self.day > self.total_days:
            self.timer.stop()
            self.node.get_logger().info("[Sim] Mission Complete")
            return

        self.tick += 1
        self.node.get_logger().info(f"[Sim] Day {self.day} - Event {self.tick}")
        self.node.get_logger().info("======================================")
        high_metabolism = random.random() < 0.25

        # CO2 generation
        base_co2 = 100.0 if not high_metabolism else 160.0
        exercise_boost = 80.0 if self.tick == self.exercise_event else 0.0
        co2_generated = (base_co2 + exercise_boost) * self.crew_size
        self.co2_level += co2_generated

        if self.co2_level >= 2000.0:
            self.send_ars_goal(self.co2_level)
            self.co2_level = 400.0

        # Water
        daily_liters = (5.0 if not high_metabolism else 15.0) * 3.78541
        water_per_event = daily_liters / 7.0
        if self.water_client.service_is_ready():
            req = RequestProductWater.Request()
            req.amount = water_per_event
            self.water_client.call_async(req)

        self.urine_volume += water_per_event * (0.9 if high_metabolism else 0.85)

        # WRS
        hygiene = (1.59 if not high_metabolism else 2.0) * self.crew_size
        excess = (0.2 if not high_metabolism else 0.5) * self.crew_size
        total_waste = self.urine_volume + hygiene + excess
        self.send_wrs_goal(total_waste)
        self.urine_volume = 0.0

        # O2
        if self.o2_client.service_is_ready():
            req = O2Request.Request()
            req.o2_req = (800.0 if not high_metabolism else 1000.0) * self.crew_size
            self.o2_client.call_async(req)

        # Day end
        if self.tick >= 7:
            self.day += 1
            self.tick = 0
            self.node.get_logger().info(f"[Sim] Completed Day {self.day - 1}")
        self.node.get_logger().info("+======================================+")

    def update_co2_status(self, msg):
        # self.node.get_logger().info(f"[GUI][CO2] Received: {msg.data} mmHg")
        percent = round((msg.data / MAX_CO2_STORAGE) * 100.0, 1)
        QMetaObject.invokeMethod(self.co2_status, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"CO₂: {round(msg.data, 2)} mmHg"))
        QMetaObject.invokeMethod(self.co2_bar, "setValue", Qt.QueuedConnection,
            Q_ARG(int, int(percent)))

    def update_o2_status(self, msg):
        # self.node.get_logger().info(f"[GUI][O2] Received: {msg.data} g")
        percent = round((msg.data / MAX_O2_STORAGE) * 100.0, 1)
        QMetaObject.invokeMethod(self.o2_status, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"O₂ Reserve: {percent} %"))
        QMetaObject.invokeMethod(self.o2_bar, "setValue", Qt.QueuedConnection,
            Q_ARG(int, int(percent)))

    def update_water_status(self, msg):
        # self.node.get_logger().info(f"[GUI][Water] Received: {msg.data} L")
        percent = round((msg.data / MAX_WATER_STORAGE) * 100.0, 1)
        QMetaObject.invokeMethod(self.water_status, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Water Tank: {percent} % Full"))
        QMetaObject.invokeMethod(self.water_bar, "setValue", Qt.QueuedConnection,
            Q_ARG(int, int(percent)))

    def send_ars_goal(self, co2_mass=1800.0):
        if not self.ars_client.wait_for_server(timeout_sec=2.0):
            self.co2_status.setText("ARS Unavailable")
            return
        goal = AirRevitalisation.Goal()
        goal.initial_co2_mass = co2_mass
        goal.initial_moisture_content = 0.8 * 2.5 * self.crew_size
        goal.initial_contaminants = 5.0
        self.node.get_logger().info("Sending ARS goal...")
        self.ars_client.send_goal_async(goal)

    def send_wrs_goal(self, urine_volume=8.0):
        if not self.wrs_client.wait_for_server(timeout_sec=2.0):
            self.water_status.setText("WRS Unavailable")
            return
        goal = WaterRecovery.Goal()
        goal.urine_volume = urine_volume
        self.node.get_logger().info("Sending WRS goal...")
        self.wrs_client.send_goal_async(goal)

    def send_ogs_service(self):
        if not self.o2_client.wait_for_service(timeout_sec=2.0):
            self.o2_status.setText("OGS Service Not Found")
            return
        req = O2Request.Request()
        req.o2_req = 10.0
        self.o2_client.call_async(req)