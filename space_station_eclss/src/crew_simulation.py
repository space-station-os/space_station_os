#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QSpinBox, QComboBox,
    QDoubleSpinBox, QTextEdit, QHBoxLayout, QGroupBox, QFormLayout, QScrollArea
)
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from PyQt5.QtGui import QPixmap
import random
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from space_station_eclss.action import AirRevitalisation, WaterRecovery
from space_station_eclss.srv import O2Request, RequestProductWater
from astro_mode import AstronautSimGui
from developer_mode import DeveloperSimGui
from user_mode import UserSimGui
from ament_index_python.packages import get_package_share_directory
import os


class HumanSimGuiNode(Node):
    def __init__(self):
        super().__init__('crew_simulation_node')

        self.declare_parameters('', [
            ('crew_size', 4),
            ('events_per_day', 7),
            ('number_of_days', 1),
            ('mode', 'rest'),
            ('calorie_intake', 2000.0),
            ('potable_water_intake', 2.5),
            ('ars_failure_enabled', False),
            ('ogs_enable_failure', False),
            ('ogs_o2_efficiency', 0.95),
            ('ogs_sabatier_efficiency', 0.75),
            ('wrs_max_capacity', 100.0),
        ])

        self.ars_client = ActionClient(self, AirRevitalisation, 'air_revitalisation')
        self.wrs_client = ActionClient(self, WaterRecovery, 'water_recovery_systems')
        self.o2_client = self.create_client(O2Request, 'ogs/request_o2')
        self.water_client = self.create_client(RequestProductWater, 'wrs/product_water_request')

        self.o2_sub = self.create_subscription(Float64, 'o2_storage', self.o2_cb, 10)
        self.water_sub = self.create_subscription(Float64, 'wrs/product_water_reserve', self.water_cb, 10)

        self.o2_reserve = 0.0
        self.water_reserve = 0.0
        self.current_day = 0
        self.events_done = 0
        self.ars_sent = False
        self.wrs_sent = False
        self.ogs_sent = False

    def o2_cb(self, msg): self.o2_reserve = msg.data
    def water_cb(self, msg): self.water_reserve = msg.data

    def run_simulation_step(self, gui):
        if self.current_day >= self.get_parameter('number_of_days').value:
            gui.log_box.append("Simulation complete.")
            return

        crew_size = self.get_parameter('crew_size').value
        mode = self.get_parameter('mode').value
        calorie = self.get_parameter('calorie_intake').value
        water_intake = self.get_parameter('potable_water_intake').value
        o2_needed = 0.771 * crew_size + (0.376 * crew_size if mode == 'exercise' else 0.0)

        if not self.ars_sent:
            self.send_ars_goal(gui)
            self.ars_sent = True

        if not self.wrs_sent and self.water_reserve >= water_intake * crew_size:
            self.send_wrs_goal(gui)
            self.send_water_request(water_intake * crew_size, gui)
            self.wrs_sent = True

        if not self.ogs_sent and self.o2_reserve >= o2_needed:
            self.send_o2_request(o2_needed, gui)
            self.ogs_sent = True

        if self.ars_sent and self.wrs_sent and self.ogs_sent:
            self.events_done += 1
            self.ars_sent = self.wrs_sent = self.ogs_sent = False
            if self.events_done >= self.get_parameter('events_per_day').value:
                self.current_day += 1
                self.events_done = 0
                gui.log_box.append(f"Day {self.current_day} completed")

    def send_ars_goal(self, gui):
        goal = AirRevitalisation.Goal()
        crew_size = self.get_parameter('crew_size').value
        calorie = self.get_parameter('calorie_intake').value
        water = self.get_parameter('potable_water_intake').value
        co2_ppm = 350.0 if calorie >= 2000 else 262.0 if calorie >= 1500 else 175.0
        goal.initial_co2_mass = co2_ppm * crew_size
        goal.initial_moisture_content = 0.8 * water * crew_size
        goal.initial_contaminants = 5.0
        self.ars_client.wait_for_server()
        self.ars_client.send_goal_async(goal).add_done_callback(lambda fut: gui.log_box.append("ARS goal sent."))

    def send_wrs_goal(self, gui):
        goal = WaterRecovery.Goal()
        crew_size = self.get_parameter('crew_size').value
        events = self.get_parameter('events_per_day').value
        goal.urine_volume = 1.0 * crew_size * events
        self.wrs_client.wait_for_server()
        self.wrs_client.send_goal_async(goal).add_done_callback(lambda fut: gui.log_box.append("WRS goal sent."))

    def send_o2_request(self, amount, gui):
        req = O2Request.Request(); req.o2_req = amount
        self.o2_client.wait_for_service()
        fut = self.o2_client.call_async(req)
        fut.add_done_callback(lambda f: gui.log_box.append(f"O2 granted: {f.result().o2_resp}" if f.result().success else "O2 request failed"))

    def send_water_request(self, amount, gui):
        req = RequestProductWater.Request(); req.amount = amount
        self.water_client.wait_for_service()
        fut = self.water_client.call_async(req)
        fut.add_done_callback(lambda f: gui.log_box.append(f"Water granted: {f.result().water_granted}" if f.result().success else "Water request failed"))


class ModeSelectionWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Simulation Mode Selector")
        self.setFixedSize(500, 600)

        layout = QVBoxLayout()

       
        
        
        package_dir = get_package_share_directory('space_station_eclss')
        image_path = os.path.join(package_dir, 'assets', 'ssos.png')

        image_label = QLabel()
        pixmap = QPixmap(image_path)
        if not pixmap.isNull():
            pixmap = pixmap.scaled(480, 360, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(image_label)
        else:
            layout.addWidget(QLabel("Image failed to load."))

        # Title label
        title = QLabel("Choose a simulation mode:")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-weight: bold; font-size: 16px; margin-top: 20px;")
        layout.addWidget(title)

        # Mode buttons
        user_btn = QPushButton("User Mode")
        dev_btn = QPushButton("Developer Mode")
        astro_btn = QPushButton("Astronaut Mode")

        user_btn.clicked.connect(self.launch_user)
        dev_btn.clicked.connect(self.launch_dev)
        astro_btn.clicked.connect(self.launch_astro)

        layout.addWidget(user_btn)
        layout.addWidget(dev_btn)
        layout.addWidget(astro_btn)

        layout.setAlignment(Qt.AlignTop)
        self.setLayout(layout)

    def launch_user(self):
        self.hide()
        self.ui = UserSimGui(self.node)
        self.ui.show()

    def launch_dev(self):
        self.hide()
        self.ui = DeveloperSimGui(self.node)
        self.ui.show()

    def launch_astro(self):
        self.hide()
        self.ui = AstronautSimGui(self.node)
        self.ui.show()

### -----------------------------
### main()
### -----------------------------
def main():
    rclpy.init()
    node = HumanSimGuiNode()
    app = QApplication(sys.argv)
    selector = ModeSelectionWindow(node); selector.show()
    from threading import Thread; Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
