#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Bool

from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import QObject, pyqtSignal, QTimer

from ament_index_python.packages import get_package_share_directory


class DiagnosticSignalEmitter(QObject):
    failure_detected = pyqtSignal(str, str, int)


class DiagnosticListenerNode(Node):
    def __init__(self, emitter):
        super().__init__('diagnostic_listener_node')
        self.emitter = emitter
        self.shown_failures = set()

        self.create_subscription(DiagnosticStatus, '/ars/diagnostics', self.make_callback("ARS"), 10)
        self.create_subscription(DiagnosticStatus, '/ogs/diagnostics', self.make_callback("OGS"), 10)
        self.create_subscription(DiagnosticStatus, '/wrs/diagnostics', self.make_callback("WRS"), 10)

    def make_callback(self, subsystem):
        def callback(msg: DiagnosticStatus):
            if msg.level >= DiagnosticStatus.ERROR:
                unique_id = f"{subsystem}:{msg.hardware_id or msg.name}"
                if unique_id not in self.shown_failures:
                    self.get_logger().warn(f"[{subsystem} DIAG] {msg.name}: {msg.message}")
                    self.emitter.failure_detected.emit(subsystem, msg.message, msg.level)
                    self.shown_failures.add(unique_id)
            else:
                self.shown_failures.discard(f"{subsystem}:{msg.hardware_id or msg.name}")
        return callback


def main():
    rclpy.init()

    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)

    emitter = DiagnosticSignalEmitter()
    node = DiagnosticListenerNode(emitter)

    popups = []

    def show_failure_popup(subsystem, message, level):
        node.get_logger().error(f"[Popup] {subsystem}: {message}")
        box = QMessageBox()
        box.setWindowTitle(f"{subsystem} FAILURE")
        box.setText(f"{subsystem} failure detected:\n\n{message}")
        box.setIcon(QMessageBox.Critical)

        diagnose_btn = box.addButton("Self Diagnose", QMessageBox.AcceptRole)
        warn_btn = box.addButton("Issue Warning", QMessageBox.RejectRole)
        box.setStandardButtons(QMessageBox.NoButton)

        def handle_response():
            topic = f"/{subsystem.lower()}/self_diagnosis"
            pub = node.create_publisher(Bool, topic, 10)
            msg = Bool()
            if box.clickedButton() == diagnose_btn:
                msg.data = False
                node.get_logger().info(f"[{subsystem}] Self-diagnosis published.")
            elif box.clickedButton() == warn_btn:
                msg.data = True
                node.get_logger().warn(f"Astronaut confirmed failure in {subsystem}")
            pub.publish(msg)
            popups.remove(box)

        box.buttonClicked.connect(handle_response)
        box.show()
        popups.append(box)

    emitter.failure_detected.connect(show_failure_popup)

    # ROS spin via Qt timer
    timer = QTimer()
    def safe_spin():
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    timer.timeout.connect(safe_spin)
    timer.start(100)

    def cleanup():
        node.get_logger().info("Diagnostic GUI shutting down...")
        timer.stop()
        node.destroy_node()
        rclpy.shutdown()

    app.aboutToQuit.connect(cleanup)

    try:
        app.exec_()
    except KeyboardInterrupt:
        cleanup()



if __name__ == '__main__':
    main()
