#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication

from space_station.main_window import MainWindow
from space_station.video_player import VideoPlayer

# NEW: importlib.resources for robust, package-relative paths
try:
    # Py3.9+ recommended API
    from importlib.resources import files, as_file
    _USE_NEW_RESOURCES_API = True
except ImportError:
    # Fallback for older Python (still fine if you’re on 3.8)
    import importlib.resources as pkg_resources
    _USE_NEW_RESOURCES_API = False


class GuiNode(Node):
    def __init__(self):
        super().__init__('space_station_gui_node')


def play_video_splash():
    """
    Load Ssos_begin.mp4 from the package data:
    package: space_station.assets
    file:    Ssos_begin.mp4
    """
    def after_video():
        print("Splash video completed. Launching GUI...")

    if _USE_NEW_RESOURCES_API:
        video_resource = files("space_station.assets") / "Ssos_begin.mp4"
        # as_file() yields a real filesystem path even if packaged in a zip
        with as_file(video_resource) as path:
            player = VideoPlayer(str(path), on_finished_callback=after_video)
            player.play()  # Blocking until finished
    else:
        # Legacy API
        with pkg_resources.path("space_station.assets", "Ssos_begin.mp4") as path:
            player = VideoPlayer(str(path), on_finished_callback=after_video)
            player.play()  # Blocking until finished


def ros_spin(node):
    rclpy.spin(node)  # keeps subscriptions alive in background


def main():
    # Step 1: Play splash screen
    play_video_splash()

    # Step 2: Init ROS and GUI
    rclpy.init(args=None)
    node = GuiNode()

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Step 3: Launch Qt GUI
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
