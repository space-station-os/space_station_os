from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox,
    QSizePolicy
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import matplotlib.image as mpimg
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from geometry_msgs.msg import PoseStamped  # if you later want pose


class GncWidget(QWidget):
    """
    Pass a shared rclpy Node (e.g., self.node from MainWindow) to the ctor:
        self.gnc = GncWidget(self.node)
    """
    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self.node = node
        self._img_ok = False
        self._earth2d = None
        self._earth_radius = 6371e3  # meters

        self._load_2d_bg_image()
        self._init_ui()
        self._init_plot_data()
        self._start_plot_updater()
        self._init_ros_subscribers()

    # ---------- UI ----------
    def _init_ui(self):
        main_layout = QHBoxLayout(self)

        # Center: 3D Orbit (full area)
        self.canvas3d = FigureCanvas(Figure(figsize=(6, 6)))
        self.ax3d = self.canvas3d.figure.add_subplot(111, projection='3d')
        self.canvas3d.figure.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.98)
        self.canvas3d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Base 3D styling (we re-apply after each clear)
        self.ax3d.set_facecolor('black')
        self.ax3d.grid(False)
        self.ax3d.set_xticks([])
        self.ax3d.set_yticks([])
        self.ax3d.set_zticks([])
        for axis in (self.ax3d.xaxis, self.ax3d.yaxis, self.ax3d.zaxis):
            axis.line.set_color((0, 0, 0, 0))

        # Right panel: labels, buttons, then 2D plot
        right_panel = QVBoxLayout()

        title = QLabel("Guidance, Navigation & Control (GNC)")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        right_panel.addWidget(title)
        right_panel.addSpacing(10)

        orbit_box = QGroupBox("Orbital Parameters")
        orbit_box.setStyleSheet("color: white;")
        orbit_layout = QVBoxLayout()
        self.altitude_label = QLabel("Altitude: --- km")
        self.inclination_label = QLabel("Inclination: ---°")  # placeholder
        self.velocity_label = QLabel("Velocity: --- km/s")    # placeholder
        for lbl in [self.altitude_label, self.inclination_label, self.velocity_label]:
            lbl.setStyleSheet("color: lightgray;")
            orbit_layout.addWidget(lbl)
        orbit_box.setLayout(orbit_layout)
        right_panel.addWidget(orbit_box)

        attitude_box = QGroupBox("Attitude")
        attitude_box.setStyleSheet("color: white;")
        attitude_layout = QVBoxLayout()
        self.attitude_label = QLabel("Quaternion: [x: 0.0, y: 0.0, z: 0.0, w: 1.0]")
        self.angvel_label = QLabel("Angular Vel: [0.0, 0.0, 0.0] rad/s")
        for lbl in [self.attitude_label, self.angvel_label]:
            lbl.setStyleSheet("color: lightgray;")
            attitude_layout.addWidget(lbl)
        attitude_box.setLayout(attitude_layout)
        right_panel.addWidget(attitude_box)

        control_box = QGroupBox("Control Inputs")
        control_box.setStyleSheet("color: white;")
        control_layout = QHBoxLayout()
        self.thrust_btn = QPushButton("Thrust")
        self.reset_btn = QPushButton("Reset Attitude")
        self.sim_btn = QPushButton("Forward Sim")
        for btn in (self.thrust_btn, self.reset_btn, self.sim_btn):
            btn.setStyleSheet("padding: 6px; font-weight: bold;")
            control_layout.addWidget(btn)
        control_box.setLayout(control_layout)
        right_panel.addWidget(control_box)

        # 2D plot sits BELOW buttons
        self.canvas2d = FigureCanvas(Figure(figsize=(4, 2)))
        self.ax2d = self.canvas2d.figure.add_subplot(111)
        self.canvas2d.figure.subplots_adjust(left=0.12, right=0.98, bottom=0.18, top=0.92)
        right_panel.addWidget(self.canvas2d)

        right_panel.addStretch()

        # Add to main
        main_layout.addWidget(self.canvas3d, stretch=3)
        right_container = QWidget()
        right_container.setLayout(right_panel)
        right_container.setMaximumWidth(360)
        main_layout.addWidget(right_container, stretch=1)

        self.setLayout(main_layout)

    # ---------- Data/ROS ----------
    def _load_2d_bg_image(self):
        # Load ./assets/earth2d.png relative to this file
        base_dir = os.path.dirname(os.path.abspath(__file__))
        img_path = os.path.join(base_dir, "assets", "earth2d.png")
        try:
            self._earth2d = mpimg.imread(img_path)
            self._img_ok = True
        except Exception:
            self._earth2d = None
            self._img_ok = False

    def _init_plot_data(self):
        self.eci_x, self.eci_y, self.eci_z = [], [], []

    def _start_plot_updater(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_plots)
        self.timer.start(500)  # ms

    def _init_ros_subscribers(self):
        # Use /gnc/pos_eci for position trace; keep others for labels
        self.node.create_subscription(Vector3, '/gnc/pos_eci', self._eci_cb, 10)
        self.node.create_subscription(Quaternion, '/gnc/attitude_LVLH', self._att_cb, 10)
        self.node.create_subscription(Vector3, '/gnc/angvel_body', self._w_cb, 10)
        # self.node.create_subscription(PoseStamped, 'gnc/pose_all', self._pose_cb, 10)

    # ---------- Callbacks ----------
    def _eci_cb(self, msg: Vector3):
        self._append_position(msg.x, msg.y, msg.z)

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._append_position(p.x, p.y, p.z)

    def _append_position(self, x, y, z):
        self.eci_x.append(x)
        self.eci_y.append(y)
        self.eci_z.append(z)

        # Cap history
        max_pts = 2000
        if len(self.eci_x) > max_pts:
            self.eci_x = self.eci_x[-max_pts:]
            self.eci_y = self.eci_y[-max_pts:]
            self.eci_z = self.eci_z[-max_pts:]

        # Quick derived label (altitude ~ |r| - R_earth)
        r = (x**2 + y**2 + z**2) ** 0.5
        self.altitude_label.setText(f"Altitude: {r/1000.0 - 6371:.2f} km")
        # Velocity label left as placeholder unless you publish /gnc/vel_eci
        self.velocity_label.setText("Velocity: --- km/s")

    def _att_cb(self, q: Quaternion):
        self.attitude_label.setText(
            f"Quaternion: [x: {q.x:.2f}, y: {q.y:.2f}, z: {q.z:.2f}, w: {q.w:.2f}]"
        )

    def _w_cb(self, w: Vector3):
        self.angvel_label.setText(
            f"Angular Vel: [{w.x:.2f}, {w.y:.2f}, {w.z:.2f}] rad/s"
        )

    # ---------- Plot helpers ----------
    def _style_ax3d(self):
        ax = self.ax3d
        self.canvas3d.figure.patch.set_facecolor('black')
        ax.set_facecolor('black')
        ax.grid(False)
        try:
            ax.set_axis_off()        # preferred
        except Exception:
            ax._axis3don = False     # fallback
        # Make panes/lines transparent across versions
        for a in (ax.xaxis, ax.yaxis, ax.zaxis):
            try:
                a.pane.fill = False
                a.pane.set_edgecolor((0, 0, 0, 0))
            except Exception:
                pass
            try:
                a.line.set_color((0, 0, 0, 0))
            except Exception:
                pass
        for setter in (getattr(ax, "w_xaxis", None), getattr(ax, "w_yaxis", None), getattr(ax, "w_zaxis", None)):
            if setter is not None:
                try:
                    setter.set_pane_color((0, 0, 0, 0))
                except Exception:
                    pass

    def _set_equal_3d_limits(self, max_extent):
        self.ax3d.set_xlim([-max_extent, max_extent])
        self.ax3d.set_ylim([-max_extent, max_extent])
        self.ax3d.set_zlim([-max_extent, max_extent])

    # ---------- Plotting ----------
    def _update_plots(self):
        # --- 3D: draw Earth sphere first ---
        self.ax3d.cla()

        R = self._earth_radius
        u = np.linspace(0, 2*np.pi, 180)
        v = np.linspace(0, np.pi, 90)
        xs = R*np.outer(np.cos(u), np.sin(v))
        ys = R*np.outer(np.sin(u), np.sin(v))
        zs = R*np.outer(np.ones_like(u), np.cos(v))
        self.ax3d.plot_surface(
            xs, ys, zs,
            linewidth=0, antialiased=False,
            alpha=0.35, color='#0b5fff', edgecolor='none', shade=True
        )

        # Orbit trace (yellow), if any points exist
        if self.eci_x:
            x = np.asarray(self.eci_x); y = np.asarray(self.eci_y); z = np.asarray(self.eci_z)
            self.ax3d.plot(x, y, z, linewidth=2.0, color='yellow')
            max_extent = max(np.max(np.abs(x)), np.max(np.abs(y)), np.max(np.abs(z)), R*1.1)
        else:
            max_extent = R * 1.1

        self._set_equal_3d_limits(max_extent)
        self._style_ax3d()  # IMPORTANT: call after cla() and limits

        # --- 2D: background Earth + radius line ---
        self.ax2d.cla()
        if self._img_ok and self._earth2d is not None:
            y_min, y_max = (R*0.95, R*1.1)
            x_max = max(1, len(self.eci_x))
            self.ax2d.imshow(self._earth2d, extent=[0, x_max, y_min, y_max], aspect='auto', zorder=0)

        if self.eci_x:
            r = np.sqrt(np.asarray(self.eci_x)**2 + np.asarray(self.eci_y)**2 + np.asarray(self.eci_z)**2)
            self.ax2d.plot(r, linewidth=2.0, color='green', zorder=1)

        self.ax2d.set_title("Motion Radius")
        self.ax2d.set_xlabel("Time Step")
        self.ax2d.set_ylabel("Radius [m]")

        self.canvas3d.draw()
        self.canvas2d.draw()
