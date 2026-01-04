# space_station/eps.py

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGraphicsView, 
    QGraphicsScene, QDialog, QTableWidget, QTableWidgetItem, 
    QGridLayout, QSplitter, QSizePolicy, QGroupBox, QFormLayout
)
from PyQt5.QtGui import QPixmap, QColor
from PyQt5.QtCore import Qt, QTimer

import matplotlib
matplotlib.use("Qt5Agg")  # <<< REQUIRED FIX

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from collections import deque
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import BatteryState
from space_station_interfaces.msg import BCDUStatus   


class EPSWidget(QWidget):
    def __init__(self, node: Node, parent=None, use_dummy=False):
        super().__init__(parent)
        self.node = node
        self.use_dummy = use_dummy

        # --- Cached state ---
        self.batteries = {
            i: {"soc": 0.0, "mode": "idle", "voltage": 0.0, "current": 0.0}
            for i in range(24)
        }
        self.bcdu_mode = "SAFE"
        self.mbsu_channels = {i: 0.0 for i in range(12)}
        self.ddcu = {"vin": 0.0, "vout": 0.0, "temp": 0.0}

        # histories for plots
        self.hist_time = deque(maxlen=200)
        self.hist_bus_voltage = deque(maxlen=200)
        self.hist_soc = deque(maxlen=200)
        self._time_counter = 0
        self._blink_state = True

        # --- Build UI ---
        self._build_ui()

        # --- ROS interfaces ---
        self._init_ros_interfaces()

        # --- Timer for GUI updates ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_gui)
        self.timer.start(1000)  # 1 Hz

    
    def _build_ui(self):
        root_splitter = QSplitter(Qt.Vertical)
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(root_splitter)

        # Top half splitter (Plots + Battery Health)
        top_splitter = QSplitter(Qt.Horizontal)
        root_splitter.addWidget(top_splitter)

        # Bottom half splitter (BCDU/DDCU + MBSU)
        bottom_splitter = QSplitter(Qt.Horizontal)
        root_splitter.addWidget(bottom_splitter)

        # --- Quadrant 1: Plots ---
        plots_frame = QWidget()
        plots_layout = QVBoxLayout(plots_frame)
        fig, self.ax = plt.subplots(2, 1, figsize=(4, 4), dpi=100)
        self.canvas = FigureCanvas(fig)
        plots_layout.addWidget(self.canvas)
        top_splitter.addWidget(plots_frame)

        # --- Quadrant 2: Battery Health Grid ---
        self.battery_frame = QWidget()
        self.battery_grid = QGridLayout(self.battery_frame)
        self.battery_labels = []
        for i in range(24):
            lbl = QLabel(f"B{i}: -- %")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background: gray; color:white; padding:4px;")
            lbl.mousePressEvent = lambda e, idx=i: self._show_battery_popup(idx)
            self.battery_labels.append(lbl)
            self.battery_grid.addWidget(lbl, i // 6, i % 6)
        top_splitter.addWidget(self.battery_frame)

        # --- Quadrant 3: BCDU + DDCU ---
        status_group = QGroupBox("BCDU / DDCU Status")
        status_layout = QFormLayout()

        self.lbl_bcdu = QLabel("SAFE")
        self.lbl_bcdu.setStyleSheet("color: green; font-weight: bold;")

        self.lbl_ddcu_in = QLabel("-- V")
        self.lbl_ddcu_out = QLabel("-- V")
        self.lbl_ddcu_temp = QLabel("-- °C")

        status_layout.addRow("BCDU Status:", self.lbl_bcdu)
        status_layout.addRow("DDCU Input:", self.lbl_ddcu_in)
        status_layout.addRow("DDCU Output:", self.lbl_ddcu_out)
        status_layout.addRow("DDCU Temp:", self.lbl_ddcu_temp)

        status_group.setLayout(status_layout)
        bottom_splitter.addWidget(status_group)

        # --- Quadrant 4: MBSU Channels ---
        mbsu_group = QGroupBox("MBSU Channels")
        self.mbsu_table = QTableWidget(12, 2)
        self.mbsu_table.setHorizontalHeaderLabels(["Channel", "Voltage (V)"])
        for i in range(12):
            self.mbsu_table.setItem(i, 0, QTableWidgetItem(str(i)))
            self.mbsu_table.setItem(i, 1, QTableWidgetItem("--"))
        mbsu_layout = QVBoxLayout()
        mbsu_layout.addWidget(self.mbsu_table)
        mbsu_group.setLayout(mbsu_layout)
        bottom_splitter.addWidget(mbsu_group)


    # ------------------- ROS -------------------
    def _init_ros_interfaces(self):
        if self.use_dummy:
            return

        # Battery health
        for i in range(24):
            topic = f"/battery/battery_bms_{i}/health"
            self.node.create_subscription(BatteryState, topic,
                                          lambda msg, idx=i: self._battery_cb(idx, msg), 10)

        # BCDU
        self.node.create_subscription(BCDUStatus, "/bcdu/status", self._bcdu_cb, 10)

        # MBSU
        for i in range(12):
            topic = f"/mbsu/channel_{i}/voltage"
            self.node.create_subscription(Float64, topic,
                                          lambda msg, idx=i: self._mbsu_cb(idx, msg), 10)

        # DDCU
        self.node.create_subscription(Float64, "/ddcu/input_voltage", self._ddcu_vin_cb, 10)
        self.node.create_subscription(Float64, "/ddcu/output_voltage", self._ddcu_vout_cb, 10)
        self.node.create_subscription(Float64, "/ddcu/temperature", self._ddcu_temp_cb, 10)

    # ------------------- Callbacks -------------------
    def _battery_cb(self, idx, msg: BatteryState):
        self.batteries[idx]["soc"] = msg.percentage * 100.0
        self.batteries[idx]["voltage"] = msg.voltage
        self.batteries[idx]["current"] = msg.current
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            self.batteries[idx]["mode"] = "charging"
        elif msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
            self.batteries[idx]["mode"] = "discharging"
        else:
            self.batteries[idx]["mode"] = "idle"

    def _bcdu_cb(self, msg: String):
        self.bcdu_mode = msg.data

    def _mbsu_cb(self, idx, msg: Float64):
        self.mbsu_channels[idx] = msg.data

    def _ddcu_vin_cb(self, msg: Float64):
        self.ddcu["vin"] = msg.data

    def _ddcu_vout_cb(self, msg: Float64):
        self.ddcu["vout"] = msg.data

    def _ddcu_temp_cb(self, msg: Float64):
        self.ddcu["temp"] = msg.data

    
    def _update_gui(self):
       
        if self.bcdu_mode == "SAFE":
            self.lbl_bcdu.setText("SAFE")
            self.lbl_bcdu.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.lbl_bcdu.setText(self.bcdu_mode)
            self.lbl_bcdu.setStyleSheet("color: red; font-weight: bold;")

        # Update DDCU
        self.lbl_ddcu_in.setText(f"{self.ddcu['vin']:.1f} V")
        self.lbl_ddcu_out.setText(f"{self.ddcu['vout']:.1f} V")
        self.lbl_ddcu_temp.setText(f"{self.ddcu['temp']:.1f} °C")

       
        if not (120.0 <= self.ddcu['vin'] <= 180.0):
            self.lbl_ddcu_in.setStyleSheet("color: orange; font-weight: bold;")
        else:
            self.lbl_ddcu_in.setStyleSheet("color: white;")


        # Update plots
        self._time_counter += 1
        self.hist_time.append(self._time_counter)
        self.hist_bus_voltage.append(self.ddcu["vin"])
        avg_soc = sum([b["soc"] for b in self.batteries.values()]) / 24.0
        self.hist_soc.append(avg_soc)

        self.ax[0].cla()
        self.ax[0].plot(self.hist_time, self.hist_bus_voltage, color="blue")
        self.ax[0].set_title("Bus Voltage (DDCU Vin)")

        self.ax[1].cla()
        self.ax[1].plot(self.hist_time, self.hist_soc, color="green")
        self.ax[1].set_title("Avg Battery SOC")
        self.canvas.draw()

        # Update battery labels
        self._blink_state = not self._blink_state
        for i, lbl in enumerate(self.battery_labels):
            b = self.batteries[i]
            lbl.setText(f"B{i}: {b['soc']:.1f}%")
            lbl.setToolTip(
                f"Voltage: {b['voltage']:.1f} V\nCurrent: {b['current']:.1f} A\nSOC: {b['soc']:.1f}%\nMode: {b['mode']}"
            )
            style = self._battery_style(b)
            if b["soc"] < 15 and self._blink_state:
                style = "background:red; color:white; padding:4px;"
            lbl.setStyleSheet(style)

        # Update MBSU
        for i in range(12):
            val = f"{self.mbsu_channels[i]:.1f}"
            item = QTableWidgetItem(val)
            if not (120.0 <= self.mbsu_channels[i] <= 180.0):
                item.setBackground(QColor("red"))
            elif self.mbsu_channels[i] > 120.0:
                item.setBackground(QColor("lightgreen"))
            self.mbsu_table.setItem(i, 1, item)


    def _battery_style(self, b):
        if b["mode"] == "charging":
            color = "green"
        elif b["mode"] == "discharging":
            color = "orange"
        else:
            color = "gray"
        if b["soc"] < 15:
            color = "red"
        return f"background:{color}; color:white; padding:4px;"

    def _show_battery_popup(self, idx):
        b = self.batteries[idx]
        dlg = QDialog(self)
        dlg.setWindowTitle(f"Battery {idx} Details")
        layout = QVBoxLayout(dlg)
        layout.addWidget(QLabel(f"Voltage: {b['voltage']:.1f} V"))
        layout.addWidget(QLabel(f"Current: {b['current']:.1f} A"))
        layout.addWidget(QLabel(f"SOC: {b['soc']:.1f}%"))
        layout.addWidget(QLabel(f"Mode: {b['mode']}"))
        dlg.exec_()
