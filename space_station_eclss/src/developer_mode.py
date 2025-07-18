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

import random
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from space_station_eclss.action import AirRevitalisation, WaterRecovery
from space_station_eclss.srv import O2Request, RequestProductWater
from astro_mode import AstronautSimGui
from Subsystem import SubsystemParamDialog

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
