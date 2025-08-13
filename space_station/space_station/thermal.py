# space_station/thermal.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt


class ThermalWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Title
        title = QLabel("Thermal Control System")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)
        layout.addSpacing(10)

        # Internal Loop
        internal_box = QGroupBox("Internal Cooling Loop")
        internal_box.setStyleSheet("color: white;")
        internal_layout = QVBoxLayout()
        self.temp_label = QLabel("Internal Temp: 35.2°C")
        self.temp_label.setStyleSheet("color: lightgray;")
        self.water_level_label = QLabel("Water Level: 90%")
        self.water_level_label.setStyleSheet("color: lightgray;")
        internal_layout.addWidget(self.temp_label)
        internal_layout.addWidget(self.water_level_label)
        internal_box.setLayout(internal_layout)

        # External Loop
        external_box = QGroupBox("External Radiator Loop")
        external_box.setStyleSheet("color: white;")
        external_layout = QVBoxLayout()
        self.ext_temp_label = QLabel("Radiator Temp: 5.6°C")
        self.ext_temp_label.setStyleSheet("color: lightgray;")
        self.state_label = QLabel("Radiator State: ACTIVE")
        self.state_label.setStyleSheet("color: lightgreen;")
        self.heat_flux_label = QLabel("Heat Flux: 480 W")
        self.heat_flux_label.setStyleSheet("color: lightgray;")
        external_layout.addWidget(self.ext_temp_label)
        external_layout.addWidget(self.state_label)
        external_layout.addWidget(self.heat_flux_label)
        external_box.setLayout(external_layout)

        # Controls
        controls = QHBoxLayout()
        vent_btn = QPushButton("Vent Heat")
        refresh_btn = QPushButton("Refresh Water")
        controls.addWidget(vent_btn)
        controls.addWidget(refresh_btn)

        # Assemble
        layout.addWidget(internal_box)
        layout.addWidget(external_box)
        layout.addLayout(controls)
        layout.addStretch()
        self.setLayout(layout)
