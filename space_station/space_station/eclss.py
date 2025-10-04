"""
EclssWidget: Crew Environmental Simulation & Visualization

- Simulates ISS crew quarters environment with 1 sec = 1 simulated minute.
- Tracks CO₂, O₂, and H₂O storage using ROS 2 topics (/co2_storage, /o2_storage, /wrs/product_water_reserve).
- Models astronaut physiology:
    * CO₂ rises during day, reduced at night (21:30–06:00 GMT).
    * Daily exercise (19:00–20:00 GMT) increases CO₂/O₂ usage.
    * Automatic ARS vent triggered when CO₂ exceeds threshold.
- Periodically requests O₂ (3360 g/day for 4 crew) and H₂O (3.8 L/day for 4 crew) via ROS services.
- Accumulates waste water (urine, greywater, etc.) and sends WRS goals for recovery.
- Provides GUI:
    * Tank bars for CO₂, O₂, H₂O.
    * Production/consumption graphs over time.
    * Crew health monitor (HR, SpO₂, BP, Temp).
    * Manual emergency buttons (Vent CO₂, Request O₂, Request H₂O).
- Designed to mimic ISS schedule (GMT) with sleep and exercise cycles.
"""


from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QGroupBox, QGridLayout, QProgressBar,
    QComboBox, QHBoxLayout, QPushButton, QInputDialog
)
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
import random
import datetime

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64
from space_station_eclss.action import AirRevitalisation, WaterRecovery
from space_station_eclss.srv import O2Request, RequestProductWater
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from space_station_eclss.msg import AstronautHealth

# ---------------- Constants ---------------- #
MAX_O2_STORAGE = 60000.0    # grams (large tank)
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

        # Simulation state
        self.sim_minutes = 0
        self.crew_size = 4
        self.total_days = 180

        # Metabolic constants
        self.O2_CONS_G_PER_DAY = 840.0
        self.CO2_GEN_G_PER_DAY = 1000.0
        self.H2O_CONS_L_PER_DAY = 3.8  # updated as per requirement
        self.MIN_PER_DAY = 1440.0

        # Buffers
        self.latest_co2 = 400.0
        self.latest_o2 = 0.0
        self.latest_h2o = 0.0
        self.o2_used = 0.0
        self.h2o_used = 0.0
        # Urine accumulation
        self.urine_accum = 0.0
        self.URINE_FRACTION_OF_INTAKE = 0.7

        # Graph data
        self.co2_x, self.co2_data = [], []
        self.o2_x, self.o2_data = [], []
        self.h2o_x, self.h2o_data = [], []
        self.co2_cons_x, self.co2_cons_data = [], []
        self.o2_cons_x, self.o2_cons_data = [], []
        self.h2o_cons_x, self.h2o_cons_data = [], []

        # Simulation tick (1 Hz = 1 min sim)
        self.sim_timer = QTimer()
        self.sim_timer.timeout.connect(self.simulate_event)
        self.sim_timer.start(1000)
        self.vent_rate = 0.15   # 15% per simulated minute until target is reached
        self.vent_target = 350.0
        # GUI refresh (5 Hz)
        self.gui_update_timer = QTimer()
        self.gui_update_timer.timeout.connect(self.refresh_gui)
        self.gui_update_timer.start(200)
        self.venting = False 
        self.send_initial_goals()

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

        # --- Production toggles ---
        self.cb_co2_prod = QCheckBox("Show CO₂")
        self.cb_co2_prod.setChecked(True)
        self.cb_o2_prod = QCheckBox("Show O₂")
        self.cb_o2_prod.setChecked(True)
        self.cb_h2o_prod = QCheckBox("Show H₂O")
        self.cb_h2o_prod.setChecked(True)

        self.cb_co2_prod.toggled.connect(lambda state: self.co2_curve.setVisible(state))
        self.cb_o2_prod.toggled.connect(lambda state: self.o2_curve.setVisible(state))
        self.cb_h2o_prod.toggled.connect(lambda state: self.h2o_curve.setVisible(state))

        prod_layout.addWidget(self.cb_co2_prod)
        prod_layout.addWidget(self.cb_o2_prod)
        prod_layout.addWidget(self.cb_h2o_prod)

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

        # --- Consumption toggles ---
        self.cb_co2_cons = QCheckBox("Show CO₂ Exhaled")
        self.cb_co2_cons.setChecked(True)
        self.cb_o2_cons = QCheckBox("Show O₂ Consumed")
        self.cb_o2_cons.setChecked(True)
        self.cb_h2o_cons = QCheckBox("Show H₂O Consumed")
        self.cb_h2o_cons.setChecked(True)

        self.cb_co2_cons.toggled.connect(lambda state: self.co2_cons_curve.setVisible(state))
        self.cb_o2_cons.toggled.connect(lambda state: self.o2_cons_curve.setVisible(state))
        self.cb_h2o_cons.toggled.connect(lambda state: self.h2o_cons_curve.setVisible(state))

        cons_layout.addWidget(self.cb_co2_cons)
        cons_layout.addWidget(self.cb_o2_cons)
        cons_layout.addWidget(self.cb_h2o_cons)

        cons_group.setLayout(cons_layout)
        layout.addWidget(cons_group, 1, 0)

        # Quadrant 4: Crew Health Monitor + Buttons
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

        # Emergency buttons
        btn_layout = QHBoxLayout()
        self.btn_vent = QPushButton("Vent CO₂")
        self.btn_o2 = QPushButton("Request O₂")
        self.btn_h2o = QPushButton("Request H₂O")
        self.btn_vent.clicked.connect(self.manual_vent)
        self.btn_o2.clicked.connect(self.manual_o2)
        self.btn_h2o.clicked.connect(self.manual_h2o)
        btn_layout.addWidget(self.btn_vent)
        btn_layout.addWidget(self.btn_o2)
        btn_layout.addWidget(self.btn_h2o)
        health_layout.addLayout(btn_layout)

        health_group.setLayout(health_layout)
        layout.addWidget(health_group, 1, 1)

        # Final layout
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

    # ---------------- ROS ---------------- #
    def init_ros_interfaces(self):
        self.ars_client = ActionClient(self.node, AirRevitalisation, '/air_revitalisation')
        self.wrs_client = ActionClient(self.node, WaterRecovery, '/water_recovery_systems')
        self.o2_client = self.node.create_client(O2Request, '/ogs/request_o2')
        self.water_client = self.node.create_client(RequestProductWater, '/wrs/product_water_request')

        # Subscriptions
        self.node.create_subscription(Float64, '/co2_storage', lambda m: setattr(self, "latest_co2", m.data), 10)
        self.node.create_subscription(Float64, '/o2_storage', lambda m: setattr(self, "latest_o2", m.data), 10)
        self.node.create_subscription(Float64, '/wrs/product_water_reserve', lambda m: setattr(self, "latest_h2o", m.data), 10)

        self.bms_pub = self.node.create_publisher(AstronautHealth, "/crew/vitals", 10)

    # ---------------- Simulation ---------------- #
    def simulate_event(self):
        self.sim_minutes += 1
        sim_hour = (self.sim_minutes / 60.0) % 24.0

        # Normal metabolism
        o2_cons = self.O2_CONS_G_PER_DAY / self.MIN_PER_DAY
        co2_gen = self.CO2_GEN_G_PER_DAY / self.MIN_PER_DAY
        h2o_cons = self.H2O_CONS_L_PER_DAY / self.MIN_PER_DAY

        # Sleep reduces metabolism
        if 21.5 <= sim_hour or sim_hour < 6.0:
            co2_gen *= 0.5
            o2_cons *= 0.5
            h2o_cons *= 0.7

        # Exercise spike
        if 19.0 <= sim_hour < 20.0:
            co2_gen *= 1.5
            o2_cons *= 1.5

        # Apply generation
        self.latest_co2 += co2_gen * self.crew_size
        self.o2_used = o2_cons * self.crew_size
        self.h2o_used = h2o_cons * self.crew_size

        # ---------------- CO₂ Venting ----------------
        if not self.venting and self.latest_co2 >= 1500.0 or self.sim_minutes % 100 == 0:
            self.node.get_logger().info(
                f"[Sim] Vent triggered at {self.latest_co2:.2f} mmHg"
            )
            self.send_ars_goal(self.latest_co2)   
            self.venting = True

        if self.venting:
            # Reduce CO₂ gradually (smooth drop)
            
            drop = self.latest_co2 * self.vent_rate
            self.latest_co2 = max(self.latest_co2 - drop, 0.0)
           
            self.node.get_logger().debug(
                f"[Sim] Venting... CO₂ reduced by {drop:.2f}, now {self.latest_co2:.2f} mmHg"
            )

            # Stop venting once we are back in safe zone
            if self.latest_co2 <= self.vent_target:
                self.venting = False
                self.node.get_logger().debug(
                    f"[Sim] Venting complete, CO₂ stabilized at {self.latest_co2:.2f} mmHg"
                )

        # O2 request hourly
        if self.sim_minutes % 60 == 0:
            if self.o2_client.service_is_ready():
                req = O2Request.Request()
                req.o2_req = (self.O2_CONS_G_PER_DAY * self.crew_size) / 24.0
                self.node.get_logger().info(f"[Sim] Requesting O₂: {req.o2_req:.2f} g")
                self.o2_client.call_async(req)

        # Water request hourly
        if self.sim_minutes % 60 == 0:
            if self.water_client.service_is_ready():
                req = RequestProductWater.Request()
                req.amount = (self.H2O_CONS_L_PER_DAY * self.crew_size) / 24.0
                self.node.get_logger().info(f"[Sim] Requesting H₂O: {req.amount:.2f} L")
                self.water_client.call_async(req)
                
        self.latest_co2 += co2_gen * self.crew_size
        self.o2_used = o2_cons * self.crew_size
        self.h2o_used = h2o_cons * self.crew_size

        # ---------------- Consumption tracking ----------------
        self.co2_cons_x.append(self.sim_minutes)
        self.co2_cons_data.append(co2_gen * self.crew_size)

        self.o2_cons_x.append(self.sim_minutes)
        self.o2_cons_data.append(self.o2_used)

        self.h2o_cons_x.append(self.sim_minutes)
        self.h2o_cons_data.append(self.h2o_used)

    # ---------------- GUI Refresh ---------------- #
    def refresh_gui(self):
        # Update CO₂
        percent = round((self.latest_co2 / MAX_CO2_STORAGE) * 100.0, 1)
        self.co2_bar["label"].setText(f"CO₂: {round(self.latest_co2, 2)} mmHg")
        self.co2_bar["bar"].setValue(int(percent))
        self.co2_x.append(self.sim_minutes)
        self.co2_data.append(self.latest_co2)
        self.co2_curve.setData(self.co2_x, self.co2_data)

        # Update O₂
        percent = round((self.latest_o2 / MAX_O2_STORAGE) * 100.0, 1)
        self.o2_bar["label"].setText(f"O₂: {round(self.latest_o2, 2)} g")
        self.o2_bar["bar"].setValue(int(percent))
        self.o2_x.append(self.sim_minutes)
        self.o2_data.append(self.latest_o2)
        self.o2_curve.setData(self.o2_x, self.o2_data)

        # Update H₂O
        percent = round((self.latest_h2o / MAX_WATER_STORAGE) * 100.0, 1)
        self.water_bar["label"].setText(f"H₂O: {round(self.latest_h2o, 2)} L")
        self.water_bar["bar"].setValue(int(percent))
        self.h2o_x.append(self.sim_minutes)
        self.h2o_data.append(self.latest_h2o)
        self.h2o_curve.setData(self.h2o_x, self.h2o_data)
        

        # Update consumption plots  
        self.co2_cons_curve.setData(self.co2_cons_x, self.co2_cons_data)
        self.o2_cons_curve.setData(self.o2_cons_x, self.o2_cons_data)
        self.h2o_cons_curve.setData(self.h2o_cons_x, self.h2o_cons_data)
        
        

        # Update astronaut health
        self.update_health_from_environment(self.o2_used, self.latest_co2)

    # ---------------- Health ---------------- #
    def update_health_from_environment(self, o2_used, co2_level):
        o2_ratio = max(0.0, 1.0 - (o2_used / MAX_O2_STORAGE))
        co2_ratio = min(1.0, co2_level / MAX_CO2_STORAGE)

        hr = 70 + int(40 * co2_ratio)
        spo2 = 99 - int(10 * (1 - o2_ratio))
        bp_sys = 120 - int(15 * (1 - o2_ratio))
        bp_dia = 80 - int(10 * (1 - o2_ratio))
        temp = 37.0 + round(0.3 * co2_ratio, 1)

        self.hr_label.setText(f"Heart Rate: {hr} bpm")
        self.spo2_label.setText(f"SpO₂: {spo2} %")
        self.bp_label.setText(f"Blood Pressure: {bp_sys}/{bp_dia} mmHg")
        self.temp_label.setText(f"Temp: {temp} °C")

        msg = AstronautHealth()
        msg.astronautname = self.crew_dropdown.currentText()
        msg.heartrate = float(hr)
        msg.spo2 = float(spo2)
        msg.bp = float(bp_sys + bp_dia/100.0)
        self.bms_pub.publish(msg)

    # ---------------- Actions ---------------- #
    def send_ars_goal(self, co2_mass=2800.0):
        if not self.ars_client.wait_for_server(timeout_sec=2.0):
            return
        goal = AirRevitalisation.Goal()
        goal.initial_co2_mass = co2_mass
        goal.initial_moisture_content = 0.8 * 2.5 * self.crew_size
        goal.initial_contaminants = 25.0
        self.node.get_logger().info("Sending ARS goal...")
        self.ars_client.send_goal_async(goal)

    def send_wrs_goal(self, urine_volume=28.0):
        if not self.wrs_client.wait_for_server(timeout_sec=2.0):
            return
        goal = WaterRecovery.Goal()
        goal.urine_volume = urine_volume
        self.node.get_logger().info("Sending WRS goal...")
        self.wrs_client.send_goal_async(goal)

    # ---------------- Manual Buttons ---------------- #
    def manual_vent(self):
        amount, ok = QInputDialog.getDouble(self, "Vent CO₂", "Enter CO₂ to vent (mmHg):", 500.0, 0.0, 10000.0, 2)
        if ok:
            self.send_ars_goal(amount)

    def manual_o2(self):
        amount, ok = QInputDialog.getDouble(self, "Request O₂", "Enter O₂ amount (grams):", 1000.0, 0.0, 60000.0, 2)
        if ok and self.o2_client.service_is_ready():
            req = O2Request.Request()
            req.o2_req = amount
            self.o2_client.call_async(req)

    def manual_h2o(self):
        amount, ok = QInputDialog.getDouble(self, "Request H₂O", "Enter H₂O amount (liters):", 1.0, 0.0, 2000.0, 2)
        if ok and self.water_client.service_is_ready():
            req = RequestProductWater.Request()
            req.amount = amount
            self.water_client.call_async(req)

    # ---------------- Startup ---------------- #
    def send_initial_goals(self):
        self.node.get_logger().info("[Startup] Initializing ECLSS...")

        # --- Initial CO₂ vent (ARS) ---
        if self.ars_client.wait_for_server(timeout_sec=2.0):
            goal = AirRevitalisation.Goal()
            goal.initial_co2_mass = self.latest_co2
            goal.initial_moisture_content = 0.8 * 2.5 * self.crew_size
            goal.initial_contaminants = 25.0
            self.node.get_logger().info("[Startup] Sending initial ARS goal...")
            self.ars_client.send_goal_async(goal)

        # --- Initial O₂ request ---
        if self.o2_client.service_is_ready():
            req = O2Request.Request()
            req.o2_req = (self.O2_CONS_G_PER_DAY * self.crew_size) / 24.0  # one hour’s worth
            self.node.get_logger().info(f"[Startup] Initial O₂ request: {req.o2_req:.2f} g")
            self.o2_client.call_async(req)

        # --- Initial hydration request (WRS water service) ---
        if self.water_client.service_is_ready():
            req = RequestProductWater.Request()
            req.amount = (self.H2O_CONS_L_PER_DAY * self.crew_size) / 24.0  # one hour’s worth
            self.node.get_logger().info(f"[Startup] Initial H₂O request: {req.amount:.2f} L")
            self.water_client.call_async(req)

        # --- Initial urine flush (WRS action) ---
        if self.wrs_client.wait_for_server(timeout_sec=2.0):
            goal = WaterRecovery.Goal()
            goal.urine_volume = 5.0
            self.node.get_logger().info("[Startup] Sending initial WRS goal...")
            self.wrs_client.send_goal_async(goal)

