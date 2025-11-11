#!/usr/bin/env python3
import os
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox, QSizePolicy
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt, QTimer

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData, GLMeshItem, GLLinePlotItem, GLGridItem

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion


class GncWidget(QWidget):
    """
    Cockpit GNC Visualization Widget
    --------------------------------
    - 3D Earth + orbit path (OpenGL)
    - Real-time position and attitude updates from ROS2
    - Optional station mesh (OBJ/STL)
    """

    def __init__(self, node: Node, parent=None):
        super().__init__(parent)
        self.node = node
        self.eci_x, self.eci_y, self.eci_z = [], [], []
        self.earth_radius = 6371e3  # meters
        self.station_item = None
        self._setup_ui()
        self._setup_gl_scene()
        self._setup_ros_subs()
        self._setup_timer()

    # ------------------------ UI SETUP ------------------------
    def _setup_ui(self):
        main_layout = QHBoxLayout(self)
        self.view3d = gl.GLViewWidget()
        self.view3d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.view3d.opts['distance'] = 1.5e7
        self.view3d.setBackgroundColor('k')
        main_layout.addWidget(self.view3d, stretch=3)

        # Right panel
        right_panel = QVBoxLayout()
        title = QLabel("Cockpit: Guidance, Navigation & Control")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setStyleSheet("color: white;")
        right_panel.addWidget(title)
        right_panel.addSpacing(10)

        # Orbit data
        orbit_box = QGroupBox("Orbit Parameters")
        orbit_box.setStyleSheet("color: white;")
        orbit_layout = QVBoxLayout()
        self.altitude_label = QLabel("Altitude: --- km")
        self.velocity_label = QLabel("Velocity: --- km/s")
        self.inclination_label = QLabel("Inclination: ---°")
        for lbl in [self.altitude_label, self.velocity_label, self.inclination_label]:
            lbl.setStyleSheet("color: lightgray;")
            orbit_layout.addWidget(lbl)
        orbit_box.setLayout(orbit_layout)
        right_panel.addWidget(orbit_box)

        # Attitude box
        att_box = QGroupBox("Attitude")
        att_box.setStyleSheet("color: white;")
        att_layout = QVBoxLayout()
        self.attitude_label = QLabel("Quaternion: [0,0,0,1]")
        self.angvel_label = QLabel("Angular Vel: [0,0,0] rad/s")
        for lbl in [self.attitude_label, self.angvel_label]:
            lbl.setStyleSheet("color: lightgray;")
            att_layout.addWidget(lbl)
        att_box.setLayout(att_layout)
        right_panel.addWidget(att_box)

        # Control buttons
        control_box = QGroupBox("Control")
        control_box.setStyleSheet("color: white;")
        ctl_layout = QHBoxLayout()
        for name in ["Thrust", "Reset", "Forward Sim"]:
            btn = QPushButton(name)
            btn.setStyleSheet("padding:6px;font-weight:bold;")
            ctl_layout.addWidget(btn)
        control_box.setLayout(ctl_layout)
        right_panel.addWidget(control_box)

        # Spacer
        right_panel.addStretch()
        right_container = QWidget()
        right_container.setLayout(right_panel)
        right_container.setMaximumWidth(360)
        main_layout.addWidget(right_container, stretch=1)
        self.setLayout(main_layout)

    # ------------------------ GL SCENE ------------------------
    def _setup_gl_scene(self):
        # Earth (sphere mesh)
        md = MeshData.sphere(rows=100, cols=100, radius=self.earth_radius)
        earth_mesh = GLMeshItem(
            meshdata=md,
            smooth=True,
            color=(0.05, 0.2, 1.0, 0.6),
            shader='shaded',
            glOptions='opaque'
        )
        self.view3d.addItem(earth_mesh)

        # Orbit trail
        self.orbit_trail = GLLinePlotItem(
            pos=np.zeros((1, 3)),
            color=(1, 1, 0, 1),
            width=2.0,
            antialias=True,
            mode='line_strip'
        )
        self.view3d.addItem(self.orbit_trail)

        # Grid (reference)
        grid = GLGridItem()
        grid.scale(1e6, 1e6, 1e6)
        grid.setDepthValue(10)
        self.view3d.addItem(grid)

        # Station (optional)
        self._load_station_model()

    def _load_station_model(self):
        """Optional ISS .stl or .obj mesh"""
        try:
            import trimesh
            from stl import mesh
            base_dir = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(base_dir, "assets", "iss.obj")
            if not os.path.exists(path):
                return

            obj = trimesh.load(path)
            verts = np.array(obj.vertices)
            faces = np.array(obj.faces)
            md = MeshData(vertexes=verts, faces=faces)
            station_item = GLMeshItem(
                meshdata=md,
                color=(0.9, 0.9, 0.9, 1),
                shader='shaded',
                smooth=True,
                glOptions='opaque'
            )
            station_item.scale(50000, 50000, 50000)
            self.view3d.addItem(station_item)
            self.station_item = station_item
            print("✅ ISS model loaded.")
        except Exception as e:
            print(f"⚠️ Could not load ISS model: {e}")

    # ------------------------ ROS SUBS ------------------------
    def _setup_ros_subs(self):
        self.node.create_subscription(Vector3, '/gnc/pos_eci', self._pos_cb, 10)
        self.node.create_subscription(Quaternion, '/gnc/attitude_LVLH', self._att_cb, 10)
        self.node.create_subscription(Vector3, '/gnc/angvel_body', self._w_cb, 10)

    # ------------------------ CALLBACKS ------------------------
    def _pos_cb(self, msg: Vector3):
        x, y, z = msg.x, msg.y, msg.z
        self.eci_x.append(x)
        self.eci_y.append(y)
        self.eci_z.append(z)
        max_pts = 2000
        if len(self.eci_x) > max_pts:
            self.eci_x = self.eci_x[-max_pts:]
            self.eci_y = self.eci_y[-max_pts:]
            self.eci_z = self.eci_z[-max_pts:]
        r = (x**2 + y**2 + z**2)**0.5
        self.altitude_label.setText(f"Altitude: {r/1000 - 6371:.2f} km")
        self._update_orbit_plot()
        if self.station_item:
            self.station_item.translate(x, y, z)

    def _att_cb(self, q: Quaternion):
        self.attitude_label.setText(
            f"Quaternion: [x:{q.x:.2f}, y:{q.y:.2f}, z:{q.z:.2f}, w:{q.w:.2f}]"
        )

    def _w_cb(self, w: Vector3):
        self.angvel_label.setText(
            f"Angular Vel: [{w.x:.2f}, {w.y:.2f}, {w.z:.2f}] rad/s"
        )

    # ------------------------ UPDATE LOGIC ------------------------
    def _setup_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._animate_camera)
        self.timer.start(100)  # 10 Hz camera rotation

    def _update_orbit_plot(self):
        if not self.eci_x:
            return
        pts = np.vstack((self.eci_x, self.eci_y, self.eci_z)).T
        self.orbit_trail.setData(pos=pts, color=(1, 1, 0, 1), width=2.0)

    def _animate_camera(self):
        # Rotate camera slowly around Z-axis for cinematic effect
        az = self.view3d.opts['azimuth'] + 0.2
        self.view3d.opts['azimuth'] = az
        self.view3d.update()


# ------------------------ Example Launch ------------------------
if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    rclpy.init()
    node = rclpy.create_node("gnc_cockpit")
    app = QApplication(sys.argv)
    w = GncWidget(node)
    w.show()
    app.exec_()
    node.destroy_node()
    rclpy.shutdown()
