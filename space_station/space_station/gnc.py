# space_station/gnc.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt


class GncWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Title
        title = QLabel("Guidance, Navigation & Control (GNC)")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)
        layout.addSpacing(10)

        # Orbital parameters
        orbit_box = QGroupBox("Orbital Parameters")
        orbit_box.setStyleSheet("color: white;")
        orbit_layout = QVBoxLayout()
        self.altitude_label = QLabel("Altitude: 405 km")
        self.inclination_label = QLabel("Inclination: 51.6°")
        self.velocity_label = QLabel("Velocity: 7.66 km/s")
        for lbl in [self.altitude_label, self.inclination_label, self.velocity_label]:
            lbl.setStyleSheet("color: lightgray;")
            orbit_layout.addWidget(lbl)
        orbit_box.setLayout(orbit_layout)

        # Attitude
        attitude_box = QGroupBox("Attitude")
        attitude_box.setStyleSheet("color: white;")
        attitude_layout = QVBoxLayout()
        self.attitude_label = QLabel("Quaternion: [x: 0.0, y: 0.0, z: 0.0, w: 1.0]")
        self.attitude_label.setStyleSheet("color: lightgray;")
        attitude_layout.addWidget(self.attitude_label)
        attitude_box.setLayout(attitude_layout)

        # Control buttons
        control_layout = QHBoxLayout()
        thrust_button = QPushButton("Thrust")
        reset_button = QPushButton("Reset Attitude")
        control_layout.addWidget(thrust_button)
        control_layout.addWidget(reset_button)

        # Placeholder for Orbit/RViz/Canvas area
        canvas_frame = QFrame()
        canvas_frame.setFixedHeight(300)
        canvas_frame.setStyleSheet("background-color: #1e1e1e; border: 1px solid #444;")
        canvas_label = QLabel("3D Orbit / Attitude Visualization (placeholder)", canvas_frame)
        canvas_label.setStyleSheet("color: gray;")
        canvas_label.setAlignment(Qt.AlignCenter)

        # Assemble layout
        layout.addWidget(orbit_box)
        layout.addWidget(attitude_box)
        layout.addLayout(control_layout)
        layout.addWidget(canvas_frame)
        layout.addStretch()
        self.setLayout(layout)
