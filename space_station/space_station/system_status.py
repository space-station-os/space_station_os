# space_station/system_status.py

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QGroupBox
)
from PyQt5.QtGui import QFont, QColor
from PyQt5.QtCore import Qt


class SystemStatusWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Title
        title = QLabel("System Diagnostics & Status")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        layout.addWidget(title)
        layout.addSpacing(10)

        # System Table
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Subsystem", "Status"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #2a2a2a;
                color: white;
                gridline-color: #444;
            }
            QHeaderView::section {
                background-color: #333;
                color: white;
            }
        """)

        subsystems = [
            ("ARS", "OK"),
            ("OGS", "OK"),
            ("WRS", "OK"),
            ("Thermal Solver", "OK"),
            ("Coolant Manager", "OK"),
            ("GNC", "OK"),
            ("Comms Bridge", "OK")
        ]

        self.table.setRowCount(len(subsystems))
        for i, (name, status) in enumerate(subsystems):
            name_item = QTableWidgetItem(name)
            status_item = QTableWidgetItem(status)
            status_item.setForeground(QColor("lightgreen"))
            self.table.setItem(i, 0, name_item)
            self.table.setItem(i, 1, status_item)

        layout.addWidget(self.table)

        # Buttons
        btn_layout = QHBoxLayout()
        self.check_btn = QPushButton("Run Self-Check")
        self.logs_btn = QPushButton("View Logs")
        btn_layout.addStretch()
        btn_layout.addWidget(self.check_btn)
        btn_layout.addWidget(self.logs_btn)

        layout.addLayout(btn_layout)
        layout.addStretch()
        self.setLayout(layout)
