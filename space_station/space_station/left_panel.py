# space_station/left_panel.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt


class LeftPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout()
        layout.setSpacing(15)

        # Header
        header = QLabel("System Overview")
        header.setFont(QFont("Arial", 12, QFont.Bold))
        header.setStyleSheet("color: white;")
        layout.addWidget(header)

        # Placeholder data
        self.co2_label = QLabel("CO₂ Level: 3.8 mmHg")
        self.o2_label = QLabel("O₂ Reserve: 85%")
        self.temp_label = QLabel("Cabin Temp: 22.1°C")

        for label in [self.co2_label, self.o2_label, self.temp_label]:
            label.setStyleSheet("color: lightgray;")
            layout.addWidget(label)

        layout.addWidget(self.separator())

        # Failures
        failure_header = QLabel("Failure Status")
        failure_header.setFont(QFont("Arial", 12, QFont.Bold))
        failure_header.setStyleSheet("color: white;")
        layout.addWidget(failure_header)

        self.failure_label = QLabel("All systems nominal")
        self.failure_label.setStyleSheet("color: lightgreen;")
        layout.addWidget(self.failure_label)

        layout.addWidget(self.separator())

        # Active Goals
        goals_header = QLabel("Active Goals")
        goals_header.setFont(QFont("Arial", 12, QFont.Bold))
        goals_header.setStyleSheet("color: white;")
        layout.addWidget(goals_header)

        self.goals_label = QLabel("None currently")
        self.goals_label.setStyleSheet("color: lightgray;")
        layout.addWidget(self.goals_label)

        layout.addStretch()
        self.setLayout(layout)

    def separator(self):
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("color: gray;")
        return line

    # Future methods to update labels from ROS 2 callbacks
    def update_co2(self, value):
        self.co2_label.setText(f"CO₂ Level: {value:.2f} mmHg")

    def update_o2(self, percent):
        self.o2_label.setText(f"O₂ Reserve: {percent:.1f}%")

    def update_temp(self, temp):
        self.temp_label.setText(f"Cabin Temp: {temp:.1f}°C")

    def update_failure(self, msg, is_critical=False):
        color = "red" if is_critical else "yellow"
        self.failure_label.setText(msg)
        self.failure_label.setStyleSheet(f"color: {color};")

    def update_goal_summary(self, summary):
        self.goals_label.setText(summary)
