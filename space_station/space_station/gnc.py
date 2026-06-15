#!/usr/bin/env python3
import os
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QFrame, QSizePolicy
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import (
    MeshData, GLMeshItem, GLLinePlotItem, GLGridItem, GLScatterPlotItem,
)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion

from space_station import theme
from space_station.widgets import MetricCard, SpecRow, page_header


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
        self._station_scale = 6.0e5
        self._setup_ui()
        self._setup_gl_scene()
        self._setup_ros_subs()
        self._setup_timer()

    # ------------------------ UI SETUP ------------------------
    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(16)

        main_layout.addWidget(page_header(
            "Guidance, Navigation & Control",
            "Orbit · Attitude · State Vector", "GNC"))

        body = QHBoxLayout()
        body.setSpacing(16)

        # 3D view in a themed card
        view_card = QFrame()
        view_card.setProperty("class", "card")
        vcl = QVBoxLayout(view_card)
        vcl.setContentsMargins(1, 1, 1, 1)
        self.view3d = gl.GLViewWidget()
        self.view3d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.view3d.opts['distance'] = 1.5e7
        self.view3d.setBackgroundColor(theme.color("bg"))
        vcl.addWidget(self.view3d)
        body.addWidget(view_card, 3)

        # Right panel
        right_panel = QVBoxLayout()
        right_panel.setSpacing(12)

        # Orbit data (spec cards)
        self.altitude_card = MetricCard("Altitude", "—", "km", "AWAITING TM", "muted")
        self.velocity_card = MetricCard("Velocity", "—", "km/s", "AWAITING TM", "muted")
        self.inclination_card = MetricCard("Inclination", "—", "°", "AWAITING TM", "muted")
        right_panel.addWidget(SpecRow([self.altitude_card, self.velocity_card,
                                       self.inclination_card]))

        # Attitude card
        att_box = QFrame()
        att_box.setProperty("class", "card")
        att_layout = QVBoxLayout(att_box)
        att_layout.setContentsMargins(18, 14, 18, 14)
        att_layout.setSpacing(8)
        att_hdr = QLabel("ATTITUDE")
        att_hdr.setProperty("class", "label")
        att_hdr.setFont(theme.label_font(8, tracking=2.0))
        att_layout.addWidget(att_hdr)
        self.attitude_label = QLabel("QUAT  [0.00, 0.00, 0.00, 1.00]")
        self.angvel_label = QLabel("ω     [0.00, 0.00, 0.00] rad/s")
        for lbl in [self.attitude_label, self.angvel_label]:
            lbl.setProperty("class", "value")
            lbl.setFont(theme.mono_font(11))
            att_layout.addWidget(lbl)
        right_panel.addWidget(att_box)

        # Control buttons
        ctl_box = QFrame()
        ctl_box.setProperty("class", "card")
        ctl_outer = QVBoxLayout(ctl_box)
        ctl_outer.setContentsMargins(18, 14, 18, 14)
        ctl_outer.setSpacing(8)
        ctl_hdr = QLabel("CONTROL")
        ctl_hdr.setProperty("class", "label")
        ctl_hdr.setFont(theme.label_font(8, tracking=2.0))
        ctl_outer.addWidget(ctl_hdr)
        ctl_layout = QHBoxLayout()
        for name in ["Thrust", "Reset", "Forward Sim"]:
            btn = QPushButton(name)
            ctl_layout.addWidget(btn)
        ctl_outer.addLayout(ctl_layout)
        right_panel.addWidget(ctl_box)

        right_panel.addStretch()
        right_container = QWidget()
        right_container.setLayout(right_panel)
        right_container.setMaximumWidth(400)
        body.addWidget(right_container, 1)

        main_layout.addLayout(body)

    # ------------------------ GL SCENE ------------------------
    def _setup_gl_scene(self):
        # Earth (sphere mesh) textured from the equirectangular earth2d.png via
        # per-vertex colors so continents are visible (no flat "blue dot").
        md = MeshData.sphere(rows=90, cols=180, radius=self.earth_radius)
        self._apply_earth_texture(md)
        earth_mesh = GLMeshItem(
            meshdata=md,
            smooth=True,
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

        # ISS marker — a bright point that revolves along the orbit as
        # /gnc/pos_eci updates (run gnc + orbit_dynamics to drive it).
        a = theme.qcolor("accent")
        # Small locator dot (visible when zoomed out where the mesh is tiny);
        # the detailed ISS structure mesh is the primary marker when zoomed in.
        self.iss_marker = GLScatterPlotItem(
            pos=np.zeros((1, 3)),
            size=9.0,
            color=(a.redF(), a.greenF(), a.blueF(), 1.0),
            pxMode=True,
        )
        self.iss_marker.setDepthValue(20)
        self.view3d.addItem(self.iss_marker)

        # Grid (reference)
        grid = GLGridItem()
        grid.scale(1e6, 1e6, 1e6)
        grid.setDepthValue(10)
        self.view3d.addItem(grid)

        # Station mesh (optional, if an iss.obj asset is present)
        self._load_station_model()

    def _apply_earth_texture(self, md):
        """Colour each sphere vertex by sampling the equirectangular earth2d.png.
        Falls back to a solid earth-blue if the image can't be loaded."""
        verts = md.vertexes()
        n = len(verts)
        colors = np.ones((n, 4), dtype=float)
        img = None
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(base_dir, "assets", "earth2d.png")
            qimg = QImage(path)
            if not qimg.isNull():
                img = qimg
        except Exception:
            img = None

        if img is None:
            # Graceful fallback: ocean-blue tint.
            colors[:, 0] = 0.10
            colors[:, 1] = 0.28
            colors[:, 2] = 0.55
            md.setVertexColors(colors)
            return

        w, h = img.width(), img.height()
        two_pi = 2.0 * np.pi
        for i in range(n):
            x, y, z = verts[i]
            r = max((x * x + y * y + z * z) ** 0.5, 1e-9)
            lon = np.arctan2(y, x)            # -pi..pi
            lat = np.arcsin(max(-1.0, min(1.0, z / r)))  # -pi/2..pi/2
            u = (lon + np.pi) / two_pi        # 0..1
            v = (np.pi / 2.0 - lat) / np.pi   # 0..1 (north at top)
            px = min(w - 1, max(0, int(u * (w - 1))))
            py = min(h - 1, max(0, int(v * (h - 1))))
            c = img.pixelColor(px, py)
            colors[i] = [c.redF(), c.greenF(), c.blueF(), 1.0]
        md.setVertexColors(colors)

    def _load_station_model(self):
        """Load the bundled ISS structure (decimated from the space_data.urdf
        SD_SpaceStation_Ver05 mesh) and render it at the orbit position. Falls
        back silently to the scatter marker if the mesh can't be loaded."""
        try:
            import trimesh
            base_dir = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(base_dir, "assets", "iss", "iss_decimated.ply")
            if not os.path.exists(path):
                self.node.get_logger().info(
                    "[gnc] ISS mesh not bundled; using marker only")
                return

            obj = trimesh.load(path, force='mesh')
            verts = np.asarray(obj.vertices, dtype=float)
            faces = np.asarray(obj.faces)
            md = MeshData(vertexes=verts, faces=faces)
            station_item = GLMeshItem(
                meshdata=md,
                color=(0.82, 0.84, 0.88, 1.0),   # light spacecraft grey
                shader='shaded',
                smooth=True,
                glOptions='opaque',
            )
            station_item.setDepthValue(15)
            self.view3d.addItem(station_item)
            self.station_item = station_item
            # The model spans ~1.5 units; exaggerate so it's visible at the
            # orbital camera distance (~6e5 m/unit => ~900 km apparent span).
            self._station_scale = 6.0e5
            self.node.get_logger().info("[gnc] ISS structure loaded")
        except Exception as e:
            self.node.get_logger().warn(f"[gnc] could not load ISS model: {e}")

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
        self.altitude_card.set_value(f"{r/1000 - 6371:.1f}")
        self.altitude_card.set_footer("LIVE", "green")
        self._update_orbit_plot()
        # Move the ISS marker to the current ECI position (it revolves as new
        # positions stream in from the orbit dynamics).
        self.iss_marker.setData(pos=np.array([[x, y, z]], dtype=float))
        if self.station_item is not None:
            # Build an absolute transform each update: rotate (URDF yaw), scale to
            # the visible size, then translate to the ECI position.
            s = self._station_scale
            tr = pg.Transform3D()
            tr.translate(x, y, z)
            tr.scale(s, s, s)
            tr.rotate(180, 0, 0, 1)
            self.station_item.setTransform(tr)

    def _att_cb(self, q: Quaternion):
        self.attitude_label.setText(
            f"QUAT  [{q.x:.2f}, {q.y:.2f}, {q.z:.2f}, {q.w:.2f}]"
        )

    def _w_cb(self, w: Vector3):
        self.angvel_label.setText(
            f"ω     [{w.x:.2f}, {w.y:.2f}, {w.z:.2f}] rad/s"
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
