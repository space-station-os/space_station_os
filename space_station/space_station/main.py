#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication

from space_station.main_window import MainWindow
from space_station.video_player import VideoPlayer

# importlib.resources setup
try:
    from importlib.resources import files, as_file
    _USE_NEW_RESOURCES_API = True
except ImportError:
    import importlib.resources as pkg_resources
    _USE_NEW_RESOURCES_API = False


class GuiNode(Node):
    def __init__(self):
        super().__init__('space_station_gui_node')


def play_video_splash():
    """Play Ssos_begin.mp4 using the Qt-based VideoPlayer (requires QApplication)."""
    def after_video():
        print("Splash video completed. Launching GUI...")

    if _USE_NEW_RESOURCES_API:
        video_resource = files("space_station.assets") / "Ssos_begin.mp4"
        with as_file(video_resource) as path:
            player = VideoPlayer(str(path), on_finished_callback=after_video)
            player.play()  # blocking until finished
    else:
        with pkg_resources.path("space_station.assets", "Ssos_begin.mp4") as path:
            player = VideoPlayer(str(path), on_finished_callback=after_video)
            player.play()  # blocking until finished


def ros_spin(node: Node):
    rclpy.spin(node)


def main():
    # 1) Start Qt first because VideoPlayer is Qt-based
    app = QApplication(sys.argv)

    # 2) Play splash (blocking)
    play_video_splash()

    # 3) Init ROS and start a shared Node
    rclpy.init(args=None)
    node = GuiNode()

    # 4) Start ROS spinning in a background thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # 5) Create and show the main window (pass the shared node)
    window = MainWindow(node)
    window.show()

    # 6) Run Qt event loop
    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        # 7) Clean shutdown of ROS and join the spin thread
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
