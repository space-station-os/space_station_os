#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QSpinBox, QComboBox,
    QDoubleSpinBox, QTextEdit, QHBoxLayout, QGroupBox, QFormLayout, QScrollArea
)
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rclpy.action import ActionClient
from std_msgs.msg import Float64
from space_station_eclss.action import AirRevitalisation, WaterRecovery
from space_station_eclss.srv import O2Request, RequestProductWater

from Subsystem import SubsystemParamDialog

class HumanSimGuiNode(Node):
    def __init__(self):
        super().__init__('crew_simulation_node')

        self.declare_parameters('', [
            ('crew_size', 4),
            ('events_per_day', 7),
            ('number_of_days', 1),
            ('mode', 'rest'),
            ('calorie_intake', 2000.0),
            ('potable_water_intake', 2.5),
            ('ars_failure_enabled', False),
            ('ogs_enable_failure', False),
            ('ogs_o2_efficiency', 0.95),
            ('ogs_sabatier_efficiency', 0.75),
            ('wrs_max_capacity', 100.0),
        ])

        self.ars_client = ActionClient(self, AirRevitalisation, 'air_revitalisation')
        self.wrs_client = ActionClient(self, WaterRecovery, 'water_recovery_systems')
        self.o2_client = self.create_client(O2Request, 'ogs/request_o2')
        self.water_client = self.create_client(RequestProductWater, 'wrs/product_water_request')

        self.o2_sub = self.create_subscription(Float64, 'o2_storage', self.o2_cb, 10)
        self.water_sub = self.create_subscription(Float64, 'wrs/product_water_reserve', self.water_cb, 10)

        self.o2_reserve = 0.0
        self.water_reserve = 0.0
        self.current_day = 0
        self.events_done = 0
        self.ars_sent = False
        self.wrs_sent = False
        self.ogs_sent = False

    def o2_cb(self, msg): self.o2_reserve = msg.data
    def water_cb(self, msg): self.water_reserve = msg.data

    def run_simulation_step(self, gui):
        if self.current_day >= self.get_parameter('number_of_days').value:
            gui.log_box.append("Simulation complete.")
            return

        crew_size = self.get_parameter('crew_size').value
        mode = self.get_parameter('mode').value
        calorie = self.get_parameter('calorie_intake').value
        water_intake = self.get_parameter('potable_water_intake').value
        o2_needed = 0.771 * crew_size + (0.376 * crew_size if mode == 'exercise' else 0.0)

        if not self.ars_sent:
            self.send_ars_goal(gui)
            self.ars_sent = True

        if not self.wrs_sent and self.water_reserve >= water_intake * crew_size:
            self.send_wrs_goal(gui)
            self.send_water_request(water_intake * crew_size, gui)
            self.wrs_sent = True

        if not self.ogs_sent and self.o2_reserve >= o2_needed:
            self.send_o2_request(o2_needed, gui)
            self.ogs_sent = True

        if self.ars_sent and self.wrs_sent and self.ogs_sent:
            self.events_done += 1
            self.ars_sent = self.wrs_sent = self.ogs_sent = False
            if self.events_done >= self.get_parameter('events_per_day').value:
                self.current_day += 1
                self.events_done = 0
                gui.log_box.append(f"Day {self.current_day} completed")

    def send_ars_goal(self, gui):
        goal = AirRevitalisation.Goal()
        crew_size = self.get_parameter('crew_size').value
        calorie = self.get_parameter('calorie_intake').value
        water = self.get_parameter('potable_water_intake').value
        co2_ppm = 350.0 if calorie >= 2000 else 262.0 if calorie >= 1500 else 175.0
        goal.initial_co2_mass = co2_ppm * crew_size
        goal.initial_moisture_content = 0.8 * water * crew_size
        goal.initial_contaminants = 5.0
        self.ars_client.wait_for_server()
        self.ars_client.send_goal_async(goal).add_done_callback(lambda fut: gui.log_box.append("ARS goal sent."))

    def send_wrs_goal(self, gui):
        goal = WaterRecovery.Goal()
        crew_size = self.get_parameter('crew_size').value
        events = self.get_parameter('events_per_day').value
        goal.urine_volume = 1.0 * crew_size * events
        self.wrs_client.wait_for_server()
        self.wrs_client.send_goal_async(goal).add_done_callback(lambda fut: gui.log_box.append("WRS goal sent."))

    def send_o2_request(self, amount, gui):
        req = O2Request.Request(); req.o2_req = amount
        self.o2_client.wait_for_service()
        fut = self.o2_client.call_async(req)
        fut.add_done_callback(lambda f: gui.log_box.append(f"O2 granted: {f.result().o2_resp}" if f.result().success else "O2 request failed"))

    def send_water_request(self, amount, gui):
        req = RequestProductWater.Request(); req.amount = amount
        self.water_client.wait_for_service()
        fut = self.water_client.call_async(req)
        fut.add_done_callback(lambda f: gui.log_box.append(f"Water granted: {f.result().water_granted}" if f.result().success else "Water request failed"))

### -----------------------------
### User Mode
### -----------------------------
class UserSimGui(QWidget):
    def __init__(self, node): super().__init__(); self.node = node; self.init_ui()
    def init_ui(self):
        self.setWindowTitle("User Mode")
        layout = QVBoxLayout()
        self.inputs = {}
        for label, default in [("Crew Size", 4), ("Events/Day", 7), ("Days", 1),
                               ("Mode", "rest"), ("Calorie Intake", 2000.0), ("Water Intake", 2.5)]:
            layout.addWidget(QLabel(label))
            if label == "Mode":
                box = QComboBox(); box.addItems(["rest", "exercise"])
            elif isinstance(default, int):
                box = QSpinBox(); box.setValue(default)
            else:
                box = QDoubleSpinBox(); box.setValue(default)
            layout.addWidget(box); self.inputs[label] = box
        self.log_box = QTextEdit(); self.log_box.setReadOnly(True)
        self.button = QPushButton("Start Simulation"); self.button.clicked.connect(self.start_sim)
        layout.addWidget(self.button); layout.addWidget(self.log_box); self.setLayout(layout)
        self.timer = QTimer(); self.timer.timeout.connect(lambda: self.node.run_simulation_step(self))
    def log(self, msg): self.log_box.append(msg)
    def start_sim(self):
        param_map = {
            "Crew Size": "crew_size",
            "Events/Day": "events_per_day",
            "Days": "number_of_days",
            "Mode": "mode",
            "Calorie Intake": "calorie_intake",
            "Water Intake": "potable_water_intake",
        }
        params = []
        for label, widget in self.inputs.items():
            value = widget.value() if hasattr(widget, 'value') else widget.currentText()
            t = Parameter.Type.INTEGER if isinstance(value, int) else Parameter.Type.DOUBLE if isinstance(value, float) else Parameter.Type.STRING
            params.append(Parameter(param_map[label], t, value))
        self.node.set_parameters(params)
        self.log("Simulation started."); self.timer.start(100)

### -----------------------------
### Developer Mode
### -----------------------------
class DeveloperSimGui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Developer Mode")
        self.inputs = {}
        self.subsystem_parameters = {
            "ARS": {
                "sim_time": 10,
                "enable_failure": True,
                "max_co2_storage": 3947.0,
                "contaminant_limit": 100.0,
                "des1_capacity": 100.0,
                "des1_removal": 1.5,
                "des1_temp_limit": 120.0,
                "des2_capacity": 100.0,
                "des2_removal": 1.5,
                "des2_temp_limit": 120.0,
                "ads1_capacity": 100.0,
                "ads1_removal": 2.5,
                "ads1_temp_limit": 404.0,
                "ads2_capacity": 100.0,
                "ads2_removal": 2.5,
                "ads2_temp_limit": 404.0,
            },
            "OGS": {
                "enable_failure": True,
                "electrolysis_temp": 100.0,
                "o2_efficiency": 0.95,
                "sabatier_efficiency": 0.75,
                "sabatier_temp": 300.0,
                "sabatier_pressure": 1.0,
                "min_o2_capacity": 100.0,
                "max_o2_capacity": 10000.0,
            },
            "WRS": {
                "enable_failure": True,
                "product_max_capacity": 2000.0,
                "waste_max_capacity": 500.0,
                "upa_valve_pressure": 100.0,
                "upa_max_temperature": 170.0,
                "ionization_valve_pressure": 90.0,
                "ionization_max_temperature": 165.0,
                "filter_valve_pressure": 85.0,
                "filter_max_temperature": 120.0,
                "catalytic_valve_pressure": 95.0,
                "catalytic_max_temperature": 180.0,
                "product_valve_pressure": 110.0,
                "waste_valve_pressure": 90.0,
            }
        }
        self.subsystem_choice = QComboBox()
        self.subsystem_choice.addItems(["ARS", "OGS", "WRS"])
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        base_params = [
            ("crew_size", 4),
            ("events_per_day", 7),
            ("number_of_days", 1),
            ("mode", "rest"),
            ("calorie_intake", 2000.0),
            ("potable_water_intake", 2.5),
        ]

        for name, default in base_params:
            layout.addWidget(QLabel(f"Base Parameters\n{name}"))
            if isinstance(default, int):
                box = QSpinBox()
                box.setMaximum(int(1e6))
                box.setValue(default)
            elif isinstance(default, float):
                box = QDoubleSpinBox()
                box.setDecimals(4)
                box.setMaximum(1e6)
                box.setValue(default)
            else:
                box = QComboBox()
                box.addItems(["rest", "exercise"])
            self.inputs[name] = box
            layout.addWidget(box)

        layout.addWidget(QLabel("Select Subsystem"))
        layout.addWidget(self.subsystem_choice)

        self.sub_button = QPushButton("Open Subsystem Config")
        self.sub_button.clicked.connect(self.open_subsystem_dialog)
        layout.addWidget(self.sub_button)

        self.apply_button = QPushButton("Apply + Start Simulation")
        self.apply_button.clicked.connect(self.apply_and_start)
        layout.addWidget(self.apply_button)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(self.log_box)

        self.setLayout(layout)
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: self.node.run_simulation_step(self))

    def open_subsystem_dialog(self):
        subsystem = self.subsystem_choice.currentText()
        dialog = SubsystemParamDialog(subsystem, self.subsystem_parameters[subsystem], self)
        if dialog.exec_():
            updated_params = dialog.get_parameters()
            for p in updated_params:
                self.subsystem_parameters[subsystem][p.name] = p.value

    def apply_and_start(self):
        params = []

        # Base params
        base_params = [
            ("crew_size", 4),
            ("events_per_day", 7),
            ("number_of_days", 1),
            ("mode", "rest"),
            ("calorie_intake", 2000.0),
            ("potable_water_intake", 2.5),
        ]

        for name, widget in self.inputs.items():
            if isinstance(widget, QSpinBox):
                value = widget.value()
                t = Parameter.Type.INTEGER
            elif isinstance(widget, QDoubleSpinBox):
                value = widget.value()
                t = Parameter.Type.DOUBLE
            elif isinstance(widget, QComboBox):
                value = widget.currentText()
                t = Parameter.Type.STRING
            else:
                continue
            # Declare before setting
            try:
                self.node.declare_parameter(name, value)
            except Exception:
                pass  # Already declared
            params.append(Parameter(name, t, value))

        # Subsystem params
        subsystem = self.subsystem_choice.currentText()
        subsystem_dict = self.subsystem_parameters[subsystem]

        for name, value in subsystem_dict.items():
            if isinstance(value, bool):
                t = Parameter.Type.BOOL
            elif isinstance(value, int):
                t = Parameter.Type.INTEGER
            elif isinstance(value, float):
                t = Parameter.Type.DOUBLE
            else:
                t = Parameter.Type.STRING
            try:
                self.node.declare_parameter(name, value)
            except Exception:
                pass  # Already declared
            params.append(Parameter(name, t, value))

        # Apply and log
        for param in params:
            try:
                self.node.set_parameters([param])
                self.log_box.append(f"[OK] Set: {param.name} = {param.value}")
            except Exception as e:
                self.log_box.append(f"[FAIL] Could not set '{param.name}': {str(e)}")

        self.log_box.append("Parameters applied. Starting simulation.")
        self.timer.start(100)




### -----------------------------
### Astronaut Mode (Stub)
### -----------------------------
class AstronautSimGui(QWidget):
    def __init__(self): super().__init__(); self.setWindowTitle("Astronaut Mode"); layout = QVBoxLayout(); layout.addWidget(QLabel("Coming soon.")); self.setLayout(layout)

### -----------------------------
### Mode Selector
### -----------------------------
class ModeSelectionWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Simulation Mode Selector")
        layout = QVBoxLayout()
        user = QPushButton("User Mode"); dev = QPushButton("Developer Mode"); astro = QPushButton("Astronaut Mode")
        user.clicked.connect(self.launch_user); dev.clicked.connect(self.launch_dev); astro.clicked.connect(self.launch_astro)
        layout.addWidget(QLabel("Choose a simulation mode:")); layout.addWidget(user); layout.addWidget(dev); layout.addWidget(astro)
        self.setLayout(layout)
    def launch_user(self): self.hide(); self.ui = UserSimGui(self.node); self.ui.show()
    def launch_dev(self): self.hide(); self.ui = DeveloperSimGui(self.node); self.ui.show()
    def launch_astro(self): self.hide(); self.ui = AstronautSimGui(); self.ui.show()

### -----------------------------
### main()
### -----------------------------
def main():
    rclpy.init()
    node = HumanSimGuiNode()
    app = QApplication(sys.argv)
    selector = ModeSelectionWindow(node); selector.show()
    from threading import Thread; Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
