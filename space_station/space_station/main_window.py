# space_station/main_window.py

import os
import sys
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QTabWidget, QLabel, QApplication, QFormLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy.node import Node
from space_station.video_player import VideoPlayer
from space_station.theme import load_dark_theme, load_light_theme

# Subsystem tabs
from space_station.eclss import EclssWidget
from space_station.thermal import ThermalWidget
from space_station.gnc import GncWidget
from space_station.comms import CommsWidget
from space_station.system_status import SystemStatusWidget
from space_station.eps import EPSWidget   


from space_station.left_panel import LeftPanel

from space_station.agent import SsosAIAgent

# --- Resources API (Py 3.9+ files/as_file) ---
try:
    from importlib.resources import files, as_file
    _USE_NEW_RESOURCES_API = True
except ImportError:
    import importlib.resources as pkg_resources
    _USE_NEW_RESOURCES_API = False


class MainWindow(QMainWindow):
    """
    QMainWindow that owns its ROS 2 context, node and executor.
    ROS callbacks are pumped via a GUI-thread QTimer (no Python threads).
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Space Station Operations Dashboard")
        self.resize(1200, 800)

        # ---- ROS init ----
        self._ros_ctx = rclpy.context.Context()
        rclpy.init(args=None, context=self._ros_ctx)

        self.node: Node = rclpy.create_node('space_station_gui_node', context=self._ros_ctx)
        self.executor = MultiThreadedExecutor(context=self._ros_ctx, num_threads=4)

        self.executor.add_node(self.node)

        # Pump ROS callbacks
        self._ros_timer = QTimer(self)
        self._ros_timer.setInterval(20)
        self._ros_timer.timeout.connect(self._spin_ros_once)
        self._ros_timer.start()

        # ---- UI init ----
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.dark_mode = True
        self._build_ui()
        load_dark_theme(QApplication.instance())

        # --- AI Agent wiring ---
        self.ai_agent = SsosAIAgent(
            self.node,
            base_url=os.environ.get("SSOS_LLM_BASE_URL", "https://integrate.api.nvidia.com/v1"),
            model=os.environ.get("SSOS_LLM_MODEL", "openai/gpt-oss-20b"),
            api_key=os.environ.get("NVIDIA_API_KEY"),
            request_timeout_s=10.0,
        )
        self.left_panel.ask_ai.connect(self.ai_agent.ask)
        self.ai_agent.ai_reply.connect(self.left_panel.append_ai_response)

        # Startup splash
        self._play_startup_video()

    # ---------------- ROS pump ----------------
    def _spin_ros_once(self):
        if self._ros_ctx.ok():
            try:
                self.executor.spin_once(timeout_sec=0.0)
            except Exception:
                pass

    # ---------------- UI ----------------
    def _build_ui(self):
        main_layout = QVBoxLayout()

        # Header
        header = QHBoxLayout()
        logo_label = QLabel()
        pixmap = QPixmap()
        try:
            if _USE_NEW_RESOURCES_API:
                logo_resource = files("space_station.assets") / "SSOSlogo.jpg"
                with as_file(logo_resource) as path:
                    pixmap = QPixmap(str(path))
            else:
                with pkg_resources.path("space_station.assets", "SSOSlogo.jpg") as path:
                    pixmap = QPixmap(str(path))
        except Exception:
            pixmap = QPixmap()

        if not pixmap.isNull():
            pixmap = pixmap.scaledToHeight(50, Qt.SmoothTransformation)
            logo_label.setPixmap(pixmap)
            logo_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)

        self.toggle_button = QPushButton("Light Mode")
        self.toggle_button.clicked.connect(self._toggle_theme)

        header.addWidget(logo_label)
        header.addStretch()
        header.addWidget(self.toggle_button)

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.addTab(GncWidget(self.node), "GNC")
        self.tabs.addTab(EclssWidget(self.node), "ECLSS")
        self.tabs.addTab(ThermalWidget(self.node), "THERMAL")
        self.tabs.addTab(EPSWidget(self.node), "EPS")  
        self.tabs.addTab(CommsWidget(self.node), "COMMS")
        self.tabs.addTab(SystemStatusWidget(self.node), "SYSTEM STATUS")
      

        # Split with LeftPanel
        split_layout = QHBoxLayout()
        self.left_panel = LeftPanel()
        self.left_panel.setFixedWidth(250)
        split_layout.addWidget(self.left_panel)
        split_layout.addWidget(self.tabs)
        split_layout.setStretch(0, 0)
        split_layout.setStretch(1, 1)

        # Footer
        footer = QHBoxLayout()
        self.crew_label = QLabel("Crew: 4")
        self.crew_label.setStyleSheet("color: white;")
        self.day_label = QLabel("Mission Day: 125")
        self.day_label.setStyleSheet("color: white;")
        self.shutdown_button = QPushButton("Shutdown GUI")
        self.diagnose_button = QPushButton("Run Diagnose")
        self.shutdown_button.clicked.connect(self._play_shutdown_video)

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

    def _toggle_theme(self):
        if self.dark_mode:
            load_light_theme(QApplication.instance())
            self.toggle_button.setText("Dark Mode")
        else:
            load_dark_theme(QApplication.instance())
            self.toggle_button.setText("Light Mode")
        self.dark_mode = not self.dark_mode

    # ---------------- Videos ----------------
    def _play_startup_video(self):
        def after_video():
            pass
        try:
            if _USE_NEW_RESOURCES_API:
                video_resource = files("space_station.assets") / "Ssos_begin.mp4"
                with as_file(video_resource) as path:
                    VideoPlayer(str(path), on_finished_callback=after_video).play()
            else:
                with pkg_resources.path("space_station.assets", "Ssos_begin.mp4") as path:
                    VideoPlayer(str(path), on_finished_callback=after_video).play()
        except Exception:
            pass

    def _play_shutdown_video(self):
        def after_video():
            self._shutdown_ros()
            QApplication.quit()
        try:
            if _USE_NEW_RESOURCES_API:
                video_resource = files("space_station.assets") / "exit_vid.mp4"
                with as_file(video_resource) as path:
                    VideoPlayer(str(path), on_finished_callback=after_video).play()
            else:
                with pkg_resources.path("space_station.assets", "exit_vid.mp4") as path:
                    VideoPlayer(str(path), on_finished_callback=after_video).play()
        except Exception:
            self._shutdown_ros()
            QApplication.quit()

    # ---------------- ROS shutdown ----------------
    def _shutdown_ros(self):
        try:
            self._ros_timer.stop()
        except Exception:
            pass
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self._ros_ctx.ok():
                rclpy.shutdown(context=self._ros_ctx)
        except Exception:
            pass

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
