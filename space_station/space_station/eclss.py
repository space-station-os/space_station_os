# eclss_widget.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QGroupBox, QGridLayout, QProgressBar,
    QComboBox, QHBoxLayout
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer, QMetaObject, Q_ARG
import pyqtgraph as pg
import random

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64
from space_station_eclss.action import AirRevitalisation, WaterRecovery
from space_station_eclss.srv import O2Request, RequestProductWater
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ---------------- Constants ---------------- #
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
        self.sim_minutes = 0   # 1 sec real = 1 minute simulated
        self.co2_level = 400.0
        self.urine_volume = 0.0
        self.crew_size = 4
        self.total_days = 180

        # Graph data (independent timelines)
        self.co2_x, self.co2_data = [], []
        self.o2_x, self.o2_data = [], []
        self.h2o_x, self.h2o_data = [], []

        # Timers
        self.timer = QTimer()
        self.timer.timeout.connect(self.simulate_event)
        self.timer.start(1000)   # 1 Hz (1 sec = 1 min sim)

        self.health_timer = QTimer()
        self.health_timer.timeout.connect(self.update_mock_health)
        self.health_timer.start(3000)

    # ---------------- UI ---------------- #
    def init_ui(self):
        layout = QGridLayout()
        layout.setSpacing(10)

        # Quadrant 1: Tanks (horizontal)
        tanks_group = QGroupBox("Storage Tanks")
        tanks_layout = QHBoxLayout()

        self.co2_bar = self._create_horizontal_tank("CO₂", "#19da69")
        self.o2_bar = self._create_horizontal_tank("O₂", "#55c3df")
        self.water_bar = self._create_horizontal_tank("H₂O", "#ae3bd1")

        tanks_layout.addWidget(self.co2_bar["box"])
        tanks_layout.addWidget(self.o2_bar["box"])
        tanks_layout.addWidget(self.water_bar["box"])
        tanks_group.setLayout(tanks_layout)
        layout.addWidget(tanks_group, 0, 0)

        # Quadrant 2: Production Graphs
        prod_group = QGroupBox("Production (CO₂, O₂, H₂O)")
        prod_layout = QVBoxLayout()
        self.prod_plot = pg.PlotWidget(title="Production Over Time")
        self.prod_plot.addLegend()
        self.prod_plot.showGrid(x=True, y=True)
        self.co2_curve = self.prod_plot.plot(pen='g', name="CO₂")
        self.o2_curve = self.prod_plot.plot(pen='b', name="O₂")
        self.h2o_curve = self.prod_plot.plot(pen='m', name="H₂O")
        prod_layout.addWidget(self.prod_plot)
        prod_group.setLayout(prod_layout)
        layout.addWidget(prod_group, 0, 1)

        # Quadrant 3: Consumption Graphs (placeholder for now)
        cons_group = QGroupBox("Consumption vs Reserves")
        cons_layout = QVBoxLayout()
        self.cons_plot = pg.PlotWidget(title="Consumption Over Time")
        self.cons_plot.addLegend()
        self.cons_plot.showGrid(x=True, y=True)
        self.co2_cons_curve = self.cons_plot.plot(pen='r', name="CO₂ Consumed")
        self.o2_cons_curve = self.cons_plot.plot(pen='c', name="O₂ Consumed")
        self.h2o_cons_curve = self.cons_plot.plot(pen='y', name="H₂O Consumed")
        cons_layout.addWidget(self.cons_plot)
        cons_group.setLayout(cons_layout)
        layout.addWidget(cons_group, 1, 0)

        # Quadrant 4: Crew Health Monitor
        health_group = QGroupBox("Crew Health Monitor")
        health_layout = QVBoxLayout()

        self.crew_dropdown = QComboBox()
        self.crew_dropdown.addItems([f"Astronaut {i+1}" for i in range(4)])
        health_layout.addWidget(self.crew_dropdown)

        self.hr_label = QLabel("Heart Rate: --- bpm")
        self.spo2_label = QLabel("SpO₂: --- %")
        self.bp_label = QLabel("Blood Pressure: ---/--- mmHg")
        self.temp_label = QLabel("Temp: --- °C")

        for lbl in [self.hr_label, self.spo2_label, self.bp_label, self.temp_label]:
            lbl.setStyleSheet("color: lightgray;")
            health_layout.addWidget(lbl)

        health_group.setLayout(health_layout)
        layout.addWidget(health_group, 1, 1)

        self.setLayout(layout)

    def _create_horizontal_tank(self, name, color):
        box = QGroupBox(name)
        vbox = QVBoxLayout()
        label = QLabel(f"{name}: ---")
        bar = QProgressBar()
        bar.setOrientation(Qt.Horizontal)
        bar.setRange(0, 100)
        bar.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; }}")
        vbox.addWidget(label)
        vbox.addWidget(bar)
        box.setLayout(vbox)
        return {"box": box, "label": label, "bar": bar}

    # ---------------- ROS Interfaces ---------------- #
    def init_ros_interfaces(self):
        self.ars_client = ActionClient(self.node, AirRevitalisation, '/air_revitalisation')
        self.wrs_client = ActionClient(self.node, WaterRecovery, '/water_recovery_systems')
        self.o2_client = self.node.create_client(O2Request, '/ogs/request_o2')
        self.water_client = self.node.create_client(RequestProductWater, '/wrs/product_water_request')

        self.node.create_subscription(Float64, '/co2_storage', self.update_co2_status, 10)
        self.node.create_subscription(Float64, '/o2_storage', self.update_o2_status, 10)
        self.node.create_subscription(Float64, '/wrs/product_water_reserve', self.update_water_status, 10)

    # ---------------- Simulation ---------------- #
    def simulate_event(self):
        # advance simulation
        self.sim_minutes += 1

        # --- Send INITIAL goals at startup ---
        if self.sim_minutes == 1:
            self.node.get_logger().info("[Sim] Sending initial ARS/WRS/OGS/Water goals...")

            self.send_ars_goal(self.co2_level)

            total_waste = self.urine_volume + (1.5 * self.crew_size)
            self.send_wrs_goal(total_waste)
            self.urine_volume = 0.0

            if self.o2_client.service_is_ready():
                req = O2Request.Request()
                req.o2_req = 800.0 * self.crew_size
                self.o2_client.call_async(req)

            if self.water_client.service_is_ready():
                req = RequestProductWater.Request()
                req.amount = 10.0 * self.crew_size
                self.water_client.call_async(req)

        # --- Then every 30 simulated minutes ---
        elif self.sim_minutes % 30 == 0:
            self.node.get_logger().info("[Sim] Sending scheduled ARS/WRS/OGS/Water goals...")

            self.send_ars_goal(self.co2_level)

            total_waste = self.urine_volume + (1.5 * self.crew_size)
            self.send_wrs_goal(total_waste)
            self.urine_volume = 0.0

            if self.o2_client.service_is_ready():
                req = O2Request.Request()
                req.o2_req = 800.0 * self.crew_size
                self.o2_client.call_async(req)

            if self.water_client.service_is_ready():
                req = RequestProductWater.Request()
                req.amount = 10.0 * self.crew_size
                self.water_client.call_async(req)

    # ---------------- Health ---------------- #
    def update_mock_health(self):
        hr = random.randint(60, 100)
        spo2 = random.randint(94, 99)
        bp_sys = random.randint(110, 130)
        bp_dia = random.randint(70, 85)
        temp = round(random.uniform(36.5, 37.5), 1)

        QMetaObject.invokeMethod(self.hr_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Heart Rate: {hr} bpm"))
        QMetaObject.invokeMethod(self.spo2_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"SpO₂: {spo2} %"))
        QMetaObject.invokeMethod(self.bp_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Blood Pressure: {bp_sys}/{bp_dia} mmHg"))
        QMetaObject.invokeMethod(self.temp_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Temp: {temp} °C"))

    # ---------------- Status Updates ---------------- #
    def update_co2_status(self, msg):
        percent = round((msg.data / MAX_CO2_STORAGE) * 100.0, 1)
        self.co2_bar["label"].setText(f"CO₂: {round(msg.data, 2)} mmHg")
        self.co2_bar["bar"].setValue(int(percent))

        self.co2_x.append(self.sim_minutes)
        self.co2_data.append(msg.data)
        self.co2_curve.setData(self.co2_x, self.co2_data)

    def update_o2_status(self, msg):
        percent = round((msg.data / MAX_O2_STORAGE) * 100.0, 1)
        self.o2_bar["label"].setText(f"O₂: {round(msg.data, 2)} g")
        self.o2_bar["bar"].setValue(int(percent))

        self.o2_x.append(self.sim_minutes)
        self.o2_data.append(msg.data)
        self.o2_curve.setData(self.o2_x, self.o2_data)

    def update_water_status(self, msg):
        percent = round((msg.data / MAX_WATER_STORAGE) * 100.0, 1)
        self.water_bar["label"].setText(f"H₂O: {round(msg.data, 2)} L")
        self.water_bar["bar"].setValue(int(percent))

        self.h2o_x.append(self.sim_minutes)
        self.h2o_data.append(msg.data)
        self.h2o_curve.setData(self.h2o_x, self.h2o_data)

    # ---------------- Action/Service Senders ---------------- #
    def send_ars_goal(self, co2_mass=2800.0):
        if not self.ars_client.wait_for_server(timeout_sec=2.0):
            self.co2_bar["label"].setText("ARS Unavailable")
            return
        goal = AirRevitalisation.Goal()
        goal.initial_co2_mass = co2_mass
        goal.initial_moisture_content = 0.8 * 2.5 * self.crew_size
        goal.initial_contaminants = 25.0
        self.node.get_logger().info("Sending ARS goal...")
        self.ars_client.send_goal_async(goal)

    def send_wrs_goal(self, urine_volume=28.0):
        if not self.wrs_client.wait_for_server(timeout_sec=2.0):
            self.water_bar["label"].setText("WRS Unavailable")
            return
        goal = WaterRecovery.Goal()
        goal.urine_volume = urine_volume
        self.node.get_logger().info("Sending WRS goal...")
        self.wrs_client.send_goal_async(goal)
