# space_station/comms.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QListWidget
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt


class CommsWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Title
        title = QLabel("Communication System")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)
        layout.addSpacing(10)

        # Status group
        status_box = QGroupBox("Relay & Link Status")
        status_box.setStyleSheet("color: white;")
        status_layout = QVBoxLayout()

        self.uplink_label = QLabel("Uplink Status: CONNECTED")
        self.downlink_label = QLabel("Last Packet Received: 3 sec ago")
        self.relay_path_label = QLabel("WebSocket Path: ws://localhost:9090")

        for label in [self.uplink_label, self.downlink_label, self.relay_path_label]:
            label.setStyleSheet("color: lightgray;")
            status_layout.addWidget(label)

        status_box.setLayout(status_layout)

        # Telemetry list
        tm_box = QGroupBox("Streaming Telemetry Topics")
        tm_box.setStyleSheet("color: white;")
        tm_layout = QVBoxLayout()

        self.tm_list = QListWidget()
        self.tm_list.setStyleSheet("color: white; background-color: #222;")
        self.tm_list.addItems([
            "/co2_storage",
            "/wrs/product_water_reserve",
            "/o2_storage"
        ])
        tm_layout.addWidget(self.tm_list)
        tm_box.setLayout(tm_layout)

        # Control buttons
        controls = QHBoxLayout()
        test_btn = QPushButton("Send Test TM")
        check_btn = QPushButton("Check Link")
        controls.addWidget(test_btn)
        controls.addWidget(check_btn)

        # Assemble layout
        layout.addWidget(status_box)
        layout.addWidget(tm_box)
        layout.addLayout(controls)
        layout.addStretch()
        self.setLayout(layout)
