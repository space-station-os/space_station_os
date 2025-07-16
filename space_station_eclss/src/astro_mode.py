import random
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QDialog, QSpinBox,
    QDoubleSpinBox, QComboBox, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Bool
from space_station_eclss.action import AirRevitalisation, OxygenGeneration, WaterRecovery
from rclpy.action import ActionClient

class FailurePopup(QDialog):
    def __init__(self, image_path, node, title, subsystem):
        super().__init__()
        self.setWindowTitle(title)
        self.setFixedSize(420, 480)
        self.node = node
        self.subsystem = subsystem

        layout = QVBoxLayout()

        image_label = QLabel()
        pixmap = QPixmap(image_path).scaled(400, 400, Qt.KeepAspectRatio)
        image_label.setPixmap(pixmap)
        image_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(image_label)

        btn_layout = QHBoxLayout()
        diag_btn = QPushButton("Self Diagnose")
        warn_btn = QPushButton("Issue Warning")

        diag_btn.clicked.connect(self.publish_self_diagnose)
        warn_btn.clicked.connect(self.issue_warning)

        btn_layout.addWidget(diag_btn)
        btn_layout.addWidget(warn_btn)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

    def publish_self_diagnose(self):
        pub = self.node.create_publisher(Bool, "/self_diagnosis", 10)
        msg = Bool(); msg.data = True
        pub.publish(msg)
        self.accept()

    def issue_warning(self):
        self.node.get_logger().warn(f"Astronaut confirmed failure in {self.subsystem}")
        self.accept()

class AstronautSimGui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Astronaut Mode")
        self.inputs = {}
        self.co2_level = 400.0
        self.urine_volume = 0.0
        self.tick = 0
        self.day = 1

        self.subsystems = ["ARS", "OGS", "WRS"]
        self.failure_images = {
            "ARS": "/home/siddarth/ssos_ws/src/space_station_os/space_station_eclss/assets/Co2_failure.png",
            "OGS": "/home/siddarth/ssos_ws/src/space_station_os/space_station_eclss/assets/Oxygen_failure.png",
            "WRS": "/home/siddarth/ssos_ws/src/space_station_os/space_station_eclss/assets/wrs_failure.png"
        }

        layout = QVBoxLayout()
        for name, default in [
            ("crew_size", 4), ("events_per_day", 7), ("number_of_days", 1),
            ("mode", "rest"), ("calorie_intake", 2000.0), ("potable_water_intake", 2.5)
        ]:
            layout.addWidget(QLabel(name))
            if isinstance(default, int):
                box = QSpinBox(); box.setValue(default); box.setMaximum(10000)
            elif isinstance(default, float):
                box = QDoubleSpinBox(); box.setValue(default); box.setDecimals(2); box.setMaximum(10000.0)
            else:
                box = QComboBox(); box.addItems(["rest", "exercise"])
            self.inputs[name] = box
            layout.addWidget(box)

        self.sim_button = QPushButton("Start Simulation")
        self.sim_button.clicked.connect(self.start_simulation)
        layout.addWidget(self.sim_button)

        self.status = QLabel("Status: Ready")
        layout.addWidget(self.status)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.simulate_event)

        self.ars_client = ActionClient(self.node, AirRevitalisation, "air_revitalisation")
        self.ogs_client = ActionClient(self.node, OxygenGeneration, "oxygen_generation")
        self.wrs_client = ActionClient(self.node, WaterRecovery, "water_recovery_systems")

        self.node.create_subscription(DiagnosticStatus, "/ars/heartbeat", self.diagnostic_callback, 10)
        self.node.create_subscription(DiagnosticStatus, "/ogs/diagnostics", self.diagnostic_callback, 10)
        self.node.create_subscription(DiagnosticStatus, "/wrs/diagnostics", self.diagnostic_callback, 10)

    def diagnostic_callback(self, msg: DiagnosticStatus):
        if msg.level >= DiagnosticStatus.ERROR:
            for sub in self.subsystems:
                if sub in msg.name.upper():
                    popup = FailurePopup(self.failure_images[sub], self.node, f"{sub} FAILURE Detected", sub)
                    popup.exec_()

    def start_simulation(self):
        # Set parameters on node
        param_map = {
            "crew_size": Parameter.Type.INTEGER,
            "events_per_day": Parameter.Type.INTEGER,
            "number_of_days": Parameter.Type.INTEGER,
            "mode": Parameter.Type.STRING,
            "calorie_intake": Parameter.Type.DOUBLE,
            "potable_water_intake": Parameter.Type.DOUBLE,
        }
        params = []
        for name, widget in self.inputs.items():
            value = widget.value() if hasattr(widget, 'value') else widget.currentText()
            params.append(Parameter(name, param_map[name], value))
        self.node.set_parameters(params)

        self.tick = 0
        self.day = 1
        self.status.setText("Status: Running simulation...")
        self.timer.start(2000)

    def simulate_event(self):
        total_days = int(self.inputs["number_of_days"].value())
        events_per_day = int(self.inputs["events_per_day"].value())
        if self.day > total_days:
            self.timer.stop()
            self.status.setText("Status: Simulation Complete")
            self.node.get_logger().info("[Sim] All days complete.")
            return

        self.tick += 1
        self.co2_level += random.uniform(200.0, 600.0)
        self.urine_volume += 0.2
        self.node.get_logger().info(f"[Sim] Day {self.day}, Event {self.tick}: CO2={self.co2_level:.2f}, Urine={self.urine_volume:.2f}")

        if self.co2_level >= 2000.0:
            self.call_ars()
            self.co2_level = 400.0

        if self.urine_volume >= 1.0:
            self.call_wrs()
            self.urine_volume = 0.0

        self.call_ogs()

        # Simulate rotating failure
        if random.random() < 0.2:
            fail_sub = self.subsystems[self.tick % 3]
            popup = FailurePopup(self.failure_images[fail_sub], self.node, f"{fail_sub} FAILURE Simulated", fail_sub)
            popup.exec_()

        if self.tick >= events_per_day:
            self.day += 1
            self.tick = 0
            self.node.get_logger().info(f"[Sim] Day {self.day - 1} completed.")

    def call_ars(self):
        goal = AirRevitalisation.Goal()
        goal.initial_co2_mass = float(self.co2_level)
        goal.initial_moisture_content = 0.8 * float(self.inputs["potable_water_intake"].value()) * int(self.inputs["crew_size"].value())
        goal.initial_contaminants = 5.0
        self.ars_client.wait_for_server()
        self.ars_client.send_goal_async(goal).add_done_callback(self.handle_ars_result)

    def handle_ars_result(self, future):
        result = future.result().get_result()
        msg = f"[ARS RESULT] Success={result.success}, CO2 Vented={result.total_co2_vented:.2f}, Msg={result.summary_message}"
        self.node.get_logger().info(msg)

    def call_wrs(self):
        goal = WaterRecovery.Goal()
        goal.urine_volume = float(self.urine_volume)
        self.wrs_client.wait_for_server()
        self.wrs_client.send_goal_async(goal).add_done_callback(self.handle_wrs_result)

    def handle_wrs_result(self, future):
        result = future.result().get_result()
        msg = f"[WRS RESULT] Success={result.success}, Purified={result.total_purified_water:.2f}L, Msg={result.summary_message}"
        self.node.get_logger().info(msg)

    def call_ogs(self):
        goal = OxygenGeneration.Goal()
        goal.input_water_mass = float(self.inputs["potable_water_intake"].value())
        goal.iodine_concentration = 0.3
        self.ogs_client.wait_for_server()
        self.ogs_client.send_goal_async(goal).add_done_callback(self.handle_ogs_result)

    def handle_ogs_result(self, future):
        result = future.result().get_result()
        msg = f"[OGS RESULT] Success={result.success}, O2={result.total_o2_generated:.2f}g, CH4={result.total_ch4_vented:.2f}g, Msg={result.summary_message}"
        self.node.get_logger().info(msg)
