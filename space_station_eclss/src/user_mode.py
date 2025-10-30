
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
from space_station_interfaces.action import AirRevitalisation, WaterRecovery
from space_station_interfaces.srv import O2Request, RequestProductWater


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
