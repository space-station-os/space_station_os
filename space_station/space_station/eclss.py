# space_station/eclss.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QGridLayout
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt


class EclssWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Title
        title = QLabel("Environmental Control & Life Support System (ECLSS)")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)
        layout.addSpacing(10)

        # Section layout
        grid = QGridLayout()
        grid.setSpacing(20)

        # ARS - Air Revitalization
        ars_group = QGroupBox("Air Revitalization (ARS)")
        ars_group.setStyleSheet("color: white;")
        ars_layout = QVBoxLayout()
        self.co2_status = QLabel("CO₂: 3.8 mmHg")
        self.co2_status.setStyleSheet("color: lightgray;")
        ars_layout.addWidget(self.co2_status)
        ars_vent_button = QPushButton("Vent CO₂")
        ars_layout.addWidget(ars_vent_button)
        ars_group.setLayout(ars_layout)
        grid.addWidget(ars_group, 0, 0)

        # OGS - Oxygen Generation
        ogs_group = QGroupBox("Oxygen Generation (OGS)")
        ogs_group.setStyleSheet("color: white;")
        ogs_layout = QVBoxLayout()
        self.o2_status = QLabel("O₂ Reserve: 85%")
        self.o2_status.setStyleSheet("color: lightgray;")
        ogs_layout.addWidget(self.o2_status)
        ogs_button = QPushButton("Request O₂")
        ogs_layout.addWidget(ogs_button)
        ogs_group.setLayout(ogs_layout)
        grid.addWidget(ogs_group, 0, 1)

        # WRS - Water Recovery
        wrs_group = QGroupBox("Water Recovery (WRS)")
        wrs_group.setStyleSheet("color: white;")
        wrs_layout = QVBoxLayout()
        self.water_status = QLabel("Waste Tank: 35% Full")
        self.water_status.setStyleSheet("color: lightgray;")
        wrs_layout.addWidget(self.water_status)
        wrs_button = QPushButton("Start Recycling")
        wrs_layout.addWidget(wrs_button)
        wrs_group.setLayout(wrs_layout)
        grid.addWidget(wrs_group, 0, 2)

        layout.addLayout(grid)
        layout.addStretch()
        self.setLayout(layout)
