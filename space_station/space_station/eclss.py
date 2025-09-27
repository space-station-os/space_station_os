# eclss_widget.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QGroupBox, QGridLayout, QProgressBar,
    QComboBox, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer, QMetaObject, Q_ARG
import pyqtgraph as pg
import random

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64
from space_station_eclss.action import AirRevitalisation, WaterRecovery
from space_station_eclss.srv import O2Request, RequestProductWater
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Import custom astronaut vitals msg
from space_station_eclss.msg import AstronautHealth

# ---------------- Constants ---------------- #
MAX_O2_STORAGE = 60.0    # grams (more realistic than 60000 g)
MAX_WATER_STORAGE = 2000.0  # liters
MAX_CO2_STORAGE = 7000.0    # mmHg

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
        
        self.hydration_next_minute = 1
        self.wrs_next_minute = 1

        # Urine accumulation model (fraction of intake that becomes urine)
        self.urine_accum = 0.0
        self.URINE_FRACTION_OF_INTAKE = 0.7  # simple physiology model

        # NASA-like daily physiology (per astronaut)
        self.O2_CONS_G_PER_DAY  = 840.0    # g/day
        self.CO2_GEN_G_PER_DAY  = 1000.0   # g/day
        self.H2O_CONS_L_PER_DAY = 2.4      # L/day (set 3.0 for upper-bound)
        self.MIN_PER_DAY = 1440.0
        # Simulation state
        self.sim_minutes = 0   # 1 sec real = 1 minute simulated
        self.co2_level = 400.0
        self.urine_volume = 0.0
        self.crew_size = 4
        self.total_days = 180

        # Graph data
        self.co2_x, self.co2_data = [], []
        self.o2_x, self.o2_data = [], []
        self.h2o_x, self.h2o_data = [], []
        self.co2_cons_x, self.co2_cons_data = [], []
        self.o2_cons_x, self.o2_cons_data = [], []
        self.h2o_cons_x, self.h2o_cons_data = [], []

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

        # Quadrant 1: Tanks
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

        # Quadrant 3: Consumption Graphs
        cons_group = QGroupBox("Consumption vs Reserves")
        cons_layout = QVBoxLayout()
        self.cons_plot = pg.PlotWidget(title="Consumption Over Time")
        self.cons_plot.addLegend()
        self.cons_plot.showGrid(x=True, y=True)
        self.co2_cons_curve = self.cons_plot.plot(pen='r', name="CO₂ Exhaled")
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

        # Astronaut BMS publisher
        self.bms_pub = self.node.create_publisher(AstronautHealth, "/crew/vitals", 10)

    def simulate_event(self):
        self.sim_minutes += 1

        # Per-person metabolic rates per simulated minute (derived from daily)
        o2_cons = self.O2_CONS_G_PER_DAY  / self.MIN_PER_DAY   # ≈0.583 g/min
        co2_gen = self.CO2_GEN_G_PER_DAY  / self.MIN_PER_DAY   # ≈0.694 g/min
        h2o_cons = self.H2O_CONS_L_PER_DAY / self.MIN_PER_DAY  # ≈0.001667 L/min

        # Update totals / “sensed” environment
        self.co2_level += co2_gen * self.crew_size
        o2_used = o2_cons * self.crew_size
        h2o_used = h2o_cons * self.crew_size

        # Track consumption curves
        self.co2_cons_x.append(self.sim_minutes)
        self.o2_cons_x.append(self.sim_minutes)
        self.h2o_cons_x.append(self.sim_minutes)
        self.co2_cons_data.append(self.co2_level)
        self.o2_cons_data.append(o2_used)
        self.h2o_cons_data.append(h2o_used)
        self.co2_cons_curve.setData(self.co2_cons_x, self.co2_cons_data)
        self.o2_cons_curve.setData(self.o2_cons_x, self.o2_cons_data)
        self.h2o_cons_curve.setData(self.h2o_cons_x, self.h2o_cons_data)

        # Accumulate urine (fraction of consumed water becomes urine)
        self.urine_accum += h2o_used * self.URINE_FRACTION_OF_INTAKE

        # --- CO2 vent trigger (ARS action) when threshold exceeded ---
        if self.co2_level >= 500.0:
            self.node.get_logger().info("[Sim] CO₂ exceeded threshold, sending ARS goal…")
            self.send_ars_goal(self.co2_level)
            self.co2_level = 200.0  # reset baseline after venting

        # --- Hydration: first at minute 1, then every 120 minutes ---
        if self.sim_minutes >= self.hydration_next_minute:
            if self.water_client.service_is_ready():
                per_astronaut_interval = self.H2O_CONS_L_PER_DAY / 12.0   # 2 h = 1/12 day
                req = RequestProductWater.Request()
                req.amount = per_astronaut_interval * self.crew_size
                self.node.get_logger().info(f"[Sim] Hydration request: {req.amount:.2f} L")
                self.water_client.call_async(req)
            self.hydration_next_minute += 120  # schedule next after 120 min

        # --- WRS urine flush: first at minute 1, then every 120 minutes ---
        if self.sim_minutes >= self.wrs_next_minute:
            if self.wrs_client.wait_for_server(timeout_sec=0.5) and self.urine_accum > 0.0:
                self.node.get_logger().info(f"[Sim] Sending WRS goal (urine={self.urine_accum:.2f} L)")
                self.send_wrs_goal(self.urine_accum)
                self.urine_accum = 0.0
            self.wrs_next_minute += 120  # schedule next after 120 min

    # ---------------- Health ---------------- #
    def update_mock_health(self):
        hr = random.randint(60, 100)
        spo2 = random.randint(94, 99)
        bp_sys = random.randint(110, 130)
        bp_dia = random.randint(70, 85)
        temp = round(random.uniform(36.5, 37.5), 1)

        # Update GUI
        QMetaObject.invokeMethod(self.hr_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Heart Rate: {hr} bpm"))
        QMetaObject.invokeMethod(self.spo2_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"SpO₂: {spo2} %"))
        QMetaObject.invokeMethod(self.bp_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Blood Pressure: {bp_sys}/{bp_dia} mmHg"))
        QMetaObject.invokeMethod(self.temp_label, "setText", Qt.QueuedConnection,
            Q_ARG(str, f"Temp: {temp} °C"))

        # Publish BMS data
        msg = AstronautHealth()
        msg.astronautname = self.crew_dropdown.currentText()
        msg.heartrate = float(hr)
        msg.spo2 = float(spo2)
        msg.bp = float(bp_sys + bp_dia/100.0)  # simple encoding
        self.bms_pub.publish(msg)

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
