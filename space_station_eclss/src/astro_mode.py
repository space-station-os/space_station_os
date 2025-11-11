# astronaut_sim_gui.py

import os
import random
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QSpinBox, QMessageBox, QTextEdit
from PyQt5.QtCore import QTimer
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from space_station_interfaces.action import AirRevitalisation, WaterRecovery
from space_station_interfaces.srv import RequestProductWater, O2Request


class AstronautSimGui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Astronaut Simulation")
        self.setFixedSize(500, 400)

        layout = QVBoxLayout()
        self.inputs = {}

        layout.addWidget(QLabel("Crew Size"))
        crew_box = QSpinBox(); crew_box.setValue(4); crew_box.setMaximum(10)
        self.inputs["crew_size"] = crew_box; layout.addWidget(crew_box)

        layout.addWidget(QLabel("Number of Days"))
        day_box = QSpinBox(); day_box.setValue(3); day_box.setMaximum(100)
        self.inputs["number_of_days"] = day_box; layout.addWidget(day_box)

        self.sim_button = QPushButton("Start Simulation")
        self.sim_button.clicked.connect(self.start_simulation)
        layout.addWidget(self.sim_button)

        self.status = QLabel("Status: Ready")
        layout.addWidget(self.status)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(self.log_box)

        self.setLayout(layout)
        self.timer = QTimer()

        self.astronaut = Astronaut(node=self.node, gui=self)
        self.timer.timeout.connect(self.astronaut.simulate_event)

    def log(self, msg: str):
        self.log_box.append(msg)
        self.node.get_logger().info(msg)

    def show_completion_popup(self):
        popup = QMessageBox(self)
        popup.setWindowTitle("Simulation Complete")
        popup.setText("All events completed successfully.")
        popup.setIcon(QMessageBox.Information)
        popup.exec_()

    def start_simulation(self):
        crew_size = self.inputs["crew_size"].value()
        num_days = self.inputs["number_of_days"].value()
        self.node.set_parameters([
            Parameter("crew_size", Parameter.Type.INTEGER, crew_size),
            Parameter("number_of_days", Parameter.Type.INTEGER, num_days)
        ])
        self.astronaut.configure(crew_size, num_days)
        self.status.setText("Status: Running...")
        self.log("Simulation started.")
        self.timer.start(2000)


class Astronaut:
    def __init__(self, node, gui):
        self.node = node
        self.gui = gui

        self.co2_level = 400.0
        self.urine_volume = 0.0
        self.tick = 0
        self.day = 1

        self.crew_size = 1
        self.total_days = 1
        self.exercise_event = 3

        self.ars_client = ActionClient(node, AirRevitalisation, "air_revitalisation")
        self.wrs_client = ActionClient(node, WaterRecovery, "water_recovery_systems")
        self.water_client = node.create_client(RequestProductWater, "wrs/product_water_request")
        self.o2_client = node.create_client(O2Request, "ogs/request_o2")

    def configure(self, crew_size, number_of_days):
        self.crew_size = crew_size
        self.total_days = number_of_days
        self.co2_level = 400.0
        self.tick = 0
        self.day = 1
        self.urine_volume = 0.0

    def log(self, msg):
        self.gui.log(msg)

    def simulate_event(self):
        if self.day > self.total_days:
            self.gui.timer.stop()
            self.gui.status.setText("Status: Simulation Complete")
            self.log("[Sim] All days complete.")
            self.gui.show_completion_popup()
            return

        self.tick += 1
        self.log(f"[Sim] Day {self.day}, Event {self.tick}")

        # === Dynamic Activity Mode ===
        high_metabolism = random.random() < 0.25  # 25% chance
        mode_tag = " (High Metabolism)" if high_metabolism else ""

        # === CO2 Simulation ===
        base_co2 = 100.0 if not high_metabolism else 160.0
        exercise_boost = 80.0 if self.tick == self.exercise_event else 0.0
        co2_generated = (base_co2 + exercise_boost) * self.crew_size

        if exercise_boost > 0:
            self.log("[Sim] Astronauts exercised this event.")

        self.co2_level += co2_generated
        self.log(f"[CO2{mode_tag}] Current level: {self.co2_level:.2f} g")

        if self.co2_level >= 2000.0:
            goal = AirRevitalisation.Goal()
            goal.initial_co2_mass = self.co2_level
            goal.initial_moisture_content = 0.8 * 2.5 * self.crew_size
            goal.initial_contaminants = 5.0
            if self.ars_client.server_is_ready():
                self.ars_client.send_goal_async(goal).add_done_callback(self.handle_ars_result)
                self.log("[ARS] Vents open. CO2 goal sent.")
                self.co2_level = 400.0
            else:
                self.log("[ARS] Server not ready. Skipping CO2 goal.")

        # === Water Request ===
        daily_gallons = 12.0 if not high_metabolism else 15.0
        daily_liters = daily_gallons * 3.78541
        water_per_event = daily_liters / 7.0

        if self.water_client.service_is_ready():
            req = RequestProductWater.Request()
            req.amount = water_per_event
            self.water_client.call_async(req).add_done_callback(self.handle_water_response)
        else:
            self.log("[Water] Service not ready. Skipping.")

        self.urine_volume += water_per_event * (0.9 if high_metabolism else 0.85)

        # === WRS ===
        hygiene_water = (1.59 if not high_metabolism else 2.0) * self.crew_size
        excess_use = (0.2 if not high_metabolism else 0.5) * self.crew_size
        total_waste = self.urine_volume + hygiene_water + excess_use

        wrs_goal = WaterRecovery.Goal()
        wrs_goal.urine_volume = total_waste
        if self.wrs_client.server_is_ready():
            self.wrs_client.send_goal_async(wrs_goal).add_done_callback(self.handle_wrs_result)
        else:
            self.log("[WRS] Server not ready. Skipping WRS goal.")

        self.urine_volume = 0.0

        # === O2 Request ===
        if self.o2_client.service_is_ready():
            req = O2Request.Request()
            req.o2_req = (800.0 if not high_metabolism else 1000.0) * self.crew_size
            self.o2_client.call_async(req).add_done_callback(self.handle_o2_response)
        else:
            self.log("[O2] Service not ready. Skipping.")

        # === Day End ===
        if self.tick >= 7:
            self.day += 1
            self.tick = 0
            self.log('<span style="color:blue;">[Sim] Day {} completed.</span>'.format(self.day - 1))

    def handle_ars_result(self, future):
        result = future.result().get_result()
        self.log(f"[ARS] CO2 Vented: {result.total_co2_vented:.2f}g | Msg: {result.summary_message}")

    def handle_wrs_result(self, future):
        result = future.result().get_result()
        self.log(f"[WRS] Water: {result.total_purified_water:.2f}L | Msg: {result.summary_message}")

    def handle_o2_response(self, future):
        result = future.result()
        if result.success:
            self.log(f"[O2] Granted: {result.o2_resp:.2f}g")
        else:
            self.log(f"[O2] Request failed: {result.message}")

    def handle_water_response(self, future):
        result = future.result()
        if result.success:
            self.log(f"[Water] Granted: {result.water_granted:.2f}L")
        else:
            self.log("[Water] Request failed.")