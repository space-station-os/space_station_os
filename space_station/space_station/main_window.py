import os
import sys
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QTabWidget, QLabel, QFrame, QApplication
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

from space_station.video_player import VideoPlayer
from space_station.theme import load_dark_theme, load_light_theme

# Import placeholders
from space_station.eclss import EclssWidget
from space_station.thermal import ThermalWidget
from space_station.gnc import GncWidget
from space_station.comms import CommsWidget
from space_station.system_status import SystemStatusWidget
from space_station.left_panel import LeftPanel

# NEW: importlib.resources for robust, package-relative paths
try:
    # Py3.9+ recommended API
    from importlib.resources import files, as_file
    _USE_NEW_RESOURCES_API = True
except ImportError:
    # Fallback for older Python (still fine if you’re on 3.8)
    import importlib.resources as pkg_resources
    _USE_NEW_RESOURCES_API = False

class MainWindow(QMainWindow):
    def __init__(self,node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Space Station Operations Dashboard")
        self.resize(1200, 800)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.init_ui()
        self.dark_mode = True
        load_dark_theme(QApplication.instance())

    def init_ui(self):
        main_layout = QVBoxLayout()

        # Header with title and theme toggle
        header = QHBoxLayout()
        title = QLabel(" Space Station Operations Dashboard")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setStyleSheet("color: white;")

        self.toggle_button = QPushButton("☀ Light Mode")
        self.toggle_button.clicked.connect(self.toggle_theme)

        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.toggle_button)

        # Tab bar
        self.tabs = QTabWidget()
        self.tabs.addTab(EclssWidget(self.node), "ECLSS")
        self.tabs.addTab(ThermalWidget(self.node), "THERMAL")
        self.tabs.addTab(GncWidget(), "GNC")
        self.tabs.addTab(CommsWidget(), "COMMS")
        self.tabs.addTab(SystemStatusWidget(self.node), "SYSTEM STATUS")

        # Horizontal split: Left panel + Tab content
        split_layout = QHBoxLayout()
        self.left_panel = LeftPanel()
        self.left_panel.setFixedWidth(250)

        split_layout.addWidget(self.left_panel)
        split_layout.addWidget(self.tabs)

        # Bottom status + controls
        footer = QHBoxLayout()
        self.crew_label = QLabel("👨‍🚀 Crew: 4")
        self.day_label = QLabel("📅 Mission Day: 125")
        self.shutdown_button = QPushButton("Shutdown GUI")
        self.diagnose_button = QPushButton("🛠 Run Diagnose")

        self.shutdown_button.clicked.connect(self.play_shutdown_video)

        footer.addWidget(self.crew_label)
        footer.addWidget(self.day_label)
        footer.addStretch()
        footer.addWidget(self.diagnose_button)
        footer.addWidget(self.shutdown_button)

        # Assemble
        main_layout.addLayout(header)
        main_layout.addLayout(split_layout)
        main_layout.addLayout(footer)

        self.central_widget.setLayout(main_layout)

    def toggle_theme(self):
        if self.dark_mode:
            load_light_theme(QApplication.instance())
            self.toggle_button.setText("🌙 Dark Mode")
        else:
            load_dark_theme(QApplication.instance())
            self.toggle_button.setText("☀ Light Mode")
        self.dark_mode = not self.dark_mode

    def play_shutdown_video(self):
        """
        Load exit_vid.mp4 from the package data.
        """
        def after_video():
            QApplication.quit()

        if _USE_NEW_RESOURCES_API:
            video_resource = files("space_station.assets") / "exit_vid.mp4"
            # as_file() yields a real filesystem path even if packaged in a zip
            with as_file(video_resource) as path:
                player = VideoPlayer(str(path), on_finished_callback=after_video)
                player.play()  # Blocking until finished
        else:
            # Legacy API
            with pkg_resources.path("space_station.assets", "exit_vid.mp4") as path:
                player = VideoPlayer(str(path), on_finished_callback=after_video)
                player.play()  # Blocking until finished

