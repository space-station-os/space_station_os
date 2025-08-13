#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication

from space_station.main_window import MainWindow
from space_station.video_player import VideoPlayer  # OpenCV-based player


class GuiNode(Node):
    def __init__(self):
        super().__init__('space_station_gui_node')


def play_video_splash():
    # Path to your source-level assets folder
    video_path = os.path.join(os.path.dirname(__file__), "assets", "Ssos_begin.mp4")

    def after_video():
        print(" Splash video completed. Launching GUI...")

    player = VideoPlayer(video_path, on_finished_callback=after_video)
    player.play()  # This blocks until video ends


def main():
    # Step 1: Play splash video (blocking)
    play_video_splash()

    # Step 2: Initialize ROS and GUI
    rclpy.init(args=None)
    node = GuiNode()

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    try:
        sys.exit(app.exec_())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
