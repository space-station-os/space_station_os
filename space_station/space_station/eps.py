# eps_widget.py

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGraphicsView,
    QGraphicsScene, QDialog, QTableWidget, QTableWidgetItem, QGridLayout
)
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtCore import Qt, QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from collections import deque
import random


class EPSWidget(QWidget):
    """
    Electrical Power System (EPS) visualization.
    - Flow diagram over a background image
    - Buttons for Battery Health (popup), MBSU (popup)
    - Inline labels for BCDU and DDCU
    - Right panel with time-series plots
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # --- Cached state (dummy data for now) ---
        self.batteries = {i: {"soc": random.uniform(20, 100),
                              "mode": random.choice(["idle", "charging", "discharging"])}
                          for i in range(24)}
        self.bcdu_mode = "SAFE"
        self.mbsu_channels = {i: random.uniform(110, 130) for i in range(12)}
        self.ddcu = {"vin": 150.0, "vout": 124.5, "temp": 35.0}

        # histories for plots
        self.hist_time = deque(maxlen=100)
        self.hist_bus_voltage = deque(maxlen=100)
        self.hist_soc = deque(maxlen=100)

        self._time_counter = 0

        # --- Build UI ---
        self._build_ui()

        # --- Timer for dummy updates ---
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_dummy_data)
        self.timer.start(1000)  # 1 Hz updates

    def _build_ui(self):
        layout = QHBoxLayout(self)

        # Left schematic (background image + overlay widgets)
        scene = QGraphicsScene()
        self.view = QGraphicsView(scene)
        try:
            bg = QPixmap("assets/eps_bg.png")  # replace with your background
            scene.addPixmap(bg)
        except Exception:
            pass

        # Battery button
        self.btn_battery = QPushButton("Battery Health")
        self.btn_battery.clicked.connect(self._open_battery_popup)
        scene.addWidget(self.btn_battery).setPos(100, 100)

        # BCDU button (inline status only)
        self.btn_bcdu = QPushButton("BCDU: SAFE")
        self.btn_bcdu.setEnabled(False)  # not clickable
        scene.addWidget(self.btn_bcdu).setPos(300, 120)

        # MBSU button
        self.btn_mbsu = QPushButton("MBSU")
        self.btn_mbsu.clicked.connect(self._open_mbsu_popup)
        scene.addWidget(self.btn_mbsu).setPos(500, 120)

        # DDCU label
        self.lbl_ddcu = QLabel("DDCU: In 150V | Out 124.5V")
        scene.addWidget(self.lbl_ddcu).setPos(700, 120)

        layout.addWidget(self.view, stretch=3)

        # Right side plots
        plot_layout = QVBoxLayout()
        fig, self.ax = plt.subplots(2, 1, figsize=(4, 4), dpi=100)
        self.canvas = FigureCanvas(fig)
        plot_layout.addWidget(self.canvas)
        layout.addLayout(plot_layout, stretch=1)

        self.setLayout(layout)

    # ------------------- Popups -------------------

    def _open_battery_popup(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("Battery Health")
        grid = QGridLayout(dlg)
        self.batt_labels = []
        for i in range(24):
            lbl = QLabel(f"B{i}: {self.batteries[i]['soc']:.1f}%")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet(self._battery_style(self.batteries[i]))
            grid.addWidget(lbl, i // 6, i % 6)
            self.batt_labels.append(lbl)
        dlg.setLayout(grid)
        dlg.exec()

    def _open_mbsu_popup(self):
        dlg = QDialog(self)
        dlg.setWindowTitle("MBSU Channels")
        table = QTableWidget(12, 2)
        table.setHorizontalHeaderLabels(["Channel", "Voltage (V)"])
        for i in range(12):
            table.setItem(i, 0, QTableWidgetItem(str(i)))
            val = f"{self.mbsu_channels[i]:.1f}"
            item = QTableWidgetItem(val)
            if self.mbsu_channels[i] > 120.0:  # highlight active channels
                item.setBackground(QColor("lightgreen"))
            table.setItem(i, 1, item)
        dlg_layout = QVBoxLayout(dlg)
        dlg_layout.addWidget(table)
        dlg.exec()

    # ------------------- Helpers -------------------

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

    # ------------------- Dummy Update Loop -------------------

    def _update_dummy_data(self):
        # Fake updates for demo purposes
        for i in range(24):
            delta = random.uniform(-5, 5)
            self.batteries[i]["soc"] = max(0, min(100, self.batteries[i]["soc"] + delta))
            self.batteries[i]["mode"] = random.choice(["charging", "discharging", "idle"])

        self.bcdu_mode = random.choice(["CHARGE", "DISCHARGE", "SAFE", "FAULT"])
        self.btn_bcdu.setText(f"BCDU: {self.bcdu_mode}")

        for i in range(12):
            self.mbsu_channels[i] = random.uniform(100, 130)

        self.ddcu["vin"] = random.uniform(140, 160)
        self.ddcu["vout"] = 124.5 + random.uniform(-1, 1)
        self.ddcu["temp"] = random.uniform(30, 40)
        self.lbl_ddcu.setText(f"DDCU: In {self.ddcu['vin']:.1f}V | Out {self.ddcu['vout']:.1f}V")

        # update plots
        self._time_counter += 1
        self.hist_time.append(self._time_counter)
        self.hist_bus_voltage.append(self.ddcu["vin"])
        avg_soc = sum([b["soc"] for b in self.batteries.values()]) / 24.0
        self.hist_soc.append(avg_soc)

        self._update_plots()

    def _update_plots(self):
        self.ax[0].cla()
        self.ax[1].cla()

        self.ax[0].plot(self.hist_time, self.hist_bus_voltage, color="blue")
        self.ax[0].set_title("Bus Voltage (DDCU Vin)")
        self.ax[0].set_ylabel("V")

        self.ax[1].plot(self.hist_time, self.hist_soc, color="green")
        self.ax[1].set_title("Avg Battery SOC")
        self.ax[1].set_ylabel("%")

        self.canvas.draw()
