# thermal.py  (PyQt6 version)
# Drop-in replacement for your ThermalWidget with upgraded visualization + live plot fixes.

from PyQt6.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout,
    QGraphicsScene, QGraphicsView, QGraphicsEllipseItem, QGraphicsLineItem,
    QGraphicsTextItem, QGraphicsItem, QFrame, QSplitter, QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer, QSize
from PyQt6.QtGui import QFont, QPen, QBrush, QColor, QTransform, QPainter

import math, random, threading
from collections import deque

import rclpy
from rclpy.node import Node

from space_station_thermal_control.msg import (
    ThermalNodeDataArray, ThermalLinkFlowsArray, TankStatus
)

# Matplotlib (Qt6 backend)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt


# ---------- Zoomable view ----------
class ZoomableGraphicsView(QGraphicsView):
    MIN_SCALE = 0.2
    MAX_SCALE = 5.0

    def __init__(self, *args, min_scale=MIN_SCALE, max_scale=MAX_SCALE, **kwargs):
        super().__init__(*args, **kwargs)
        self._zoom = 1.0
        self._min_scale = min_scale
        self._max_scale = max_scale
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

    def wheelEvent(self, event):
        factor = 1.25 if event.angleDelta().y() > 0 else 1/1.25
        new_zoom = self._zoom * factor
        if self._min_scale <= new_zoom <= self._max_scale:
            self._zoom = new_zoom
            self.scale(factor, factor)

    def reset_zoom(self):
        self.setTransform(QTransform())
        self._zoom = 1.0


# ---------- Graphics items ----------
class ThermalNodeItem(QGraphicsEllipseItem):
    # Node circle size bounds
    MIN_SIZE, MAX_SIZE, SIZE_SCALE = 20, 60, 40
    DEFAULT_FONT_SIZE = 7

    def __init__(self, name, temperature_K, heat_capacity, internal_power, x=0, y=0, hover_callback=None):
        super().__init__()
        self.name = name
        self.temperature_K = temperature_K
        self.heat_capacity = heat_capacity
        self.internal_power = internal_power
        self.hover_callback = hover_callback

        # color mapping range (Kelvin)
        self._min_temp_K = 270.0
        self._max_temp_K = 330.0
        self._cold_color = QColor("blue")
        self._hot_color  = QColor("red")

        size = self._compute_size(heat_capacity)
        self.size = size
        self.setRect(-size/2, -size/2, size, size)
        self.setPos(x, y)

        self._apply_color()
        self.setPen(QPen(Qt.GlobalColor.black, 2))
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setAcceptHoverEvents(True)
        self.setZValue(2)

        self.text_item = QGraphicsTextItem(self.name, parent=self)
        self.text_item.setDefaultTextColor(Qt.GlobalColor.white)
        self.text_item.setFont(QFont("Arial", self.DEFAULT_FONT_SIZE))
        tr = self.text_item.boundingRect()
        self.text_item.setPos(-tr.width()/2, -tr.height()/2)

        self.setToolTip(self._tooltip_text())

    def set_temp_range(self, min_K, max_K, cold_color: QColor, hot_color: QColor):
        self._min_temp_K = float(min_K)
        self._max_temp_K = float(max_K)
        self._cold_color = cold_color
        self._hot_color = hot_color
        self._apply_color()

    def _compute_size(self, heat_capacity):
        # Map heat capacity to node diameter within bounds
        return max(self.MIN_SIZE, min(self.MAX_SIZE, heat_capacity/1000.0*self.SIZE_SCALE))

    def _tooltip_text(self):
        return (f"{self.name}\n"
                f"T: {self.temperature_K:.1f} K\n"
                f"Heat Cap: {self.heat_capacity:.1f} J/K\n"
                f"Power: {self.internal_power:.1f} W")

    def _apply_color(self):
        # Smooth gradient between cold_color and hot_color
        lo, hi = self._min_temp_K, self._max_temp_K
        t = (self.temperature_K - lo) / (hi - lo) if hi > lo else 0.0
        t = max(0.0, min(1.0, t))
        r = int(self._cold_color.red()   + t*(self._hot_color.red()   - self._cold_color.red()))
        g = int(self._cold_color.green() + t*(self._hot_color.green() - self._cold_color.green()))
        b = int(self._cold_color.blue()  + t*(self._hot_color.blue()  - self._cold_color.blue()))
        self.setBrush(QBrush(QColor(r, g, b)))

    def update_data(self, temperature_K, heat_capacity, internal_power):
        self.temperature_K = temperature_K
        self.heat_capacity = heat_capacity
        self.internal_power = internal_power
        size = self._compute_size(heat_capacity)
        self.size = size
        self.setRect(-size/2, -size/2, size, size)
        self._apply_color()
        self.setToolTip(self._tooltip_text())

    # Hover UI
    def hoverEnterEvent(self, e):
        if callable(self.hover_callback):
            self.hover_callback(self, 'node')
        self.setPen(QPen(Qt.GlobalColor.yellow, 2))
        super().hoverEnterEvent(e)

    def hoverMoveEvent(self, e):
        if callable(self.hover_callback):
            self.hover_callback(self, 'node')
        self.setPen(QPen(Qt.GlobalColor.yellow, 2))
        super().hoverMoveEvent(e)

    def hoverLeaveEvent(self, e):
        if callable(self.hover_callback):
            self.hover_callback(None, None)
        self.setPen(QPen(Qt.GlobalColor.black, 2))
        super().hoverLeaveEvent(e)


class ThermalLinkItem(QGraphicsLineItem):
    """
    Link between two nodes.
    - Thickness based on conductance (W/K)
    - Animated dash offset speed based on heat_flow / conductance
    """
    def __init__(self, from_node_item: ThermalNodeItem, to_node_item: ThermalNodeItem,
                 conductance, heat_flow, hover_callback=None):
        super().__init__()
        self.from_node_item = from_node_item
        self.to_node_item = to_node_item
        self.conductance = max(1e-9, float(conductance))
        self.heat_flow = float(heat_flow)
        self.dash_offset = 0.0
        self._is_hovered = False
        self.hover_callback = hover_callback

        self.setAcceptHoverEvents(True)
        self.setZValue(1)
        self.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

        self._apply_pen()
        self.update_line()

    def _apply_pen(self):
        # Width by conductance (clamped), cosmetic so it stays same width regardless of zoom
        width = max(1.0, min(8.0, self.conductance * 7.5))
        pen = QPen(Qt.GlobalColor.yellow if self._is_hovered else Qt.GlobalColor.white, width)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setCosmetic(True)
        pen.setDashPattern([2, 2])
        pen.setDashOffset(self.dash_offset)
        self.setPen(pen)

    def advance_dash(self, dt_seconds: float):
        # Dash speed proportional to (heat_flow / conductance)
        speed = (self.heat_flow / self.conductance) * 0.03  # tune factor for readability
        self.dash_offset += speed * dt_seconds * 60.0       # normalize to ~60 FPS pacing
        self._apply_pen()

    def update_data(self, conductance, heat_flow):
        self.conductance = max(1e-9, float(conductance))
        self.heat_flow = float(heat_flow)
        self._apply_pen()
        self.update_line()

    def update_line(self):
        a = self.from_node_item.scenePos()
        b = self.to_node_item.scenePos()
        self.setLine(a.x(), a.y(), b.x(), b.y())

    # Hover UI
    def hoverEnterEvent(self, e):
        self._is_hovered = True
        if callable(self.hover_callback):
            self.hover_callback(self, 'link')
        self._apply_pen()
        super().hoverEnterEvent(e)

    def hoverMoveEvent(self, e):
        self._is_hovered = True
        if callable(self.hover_callback):
            self.hover_callback(self, 'link')
        self._apply_pen()
        super().hoverMoveEvent(e)

    def hoverLeaveEvent(self, e):
        self._is_hovered = False
        if callable(self.hover_callback):
            self.hover_callback(None, None)
        self._apply_pen()
        super().hoverLeaveEvent(e)


# ---------- Main widget (Qt + ROS) ----------
class ThermalWidget(QWidget):
    """
    PyQt6 thermal visualization tab.
    Keeps the original ROS wiring, upgrades the rendering/UX, and fixes the live plot.
    """
    NODE_PLACEHOLDER = {'temperature_K': 300.0, 'heat_capacity': 1000.0, 'internal_power': 0.0}
    GRID_SPACING = 80
    FORCE_LAYOUT_ITER = 200
    VIEW_MARGIN = 40

    def __init__(self, gui_node: Node, parent=None):
        super().__init__(parent)
        self.node = gui_node  # shared ROS2 node (spun by MainWindow)
        self.node.get_logger().info("[ThermalWidget] Initializing (PyQt6)")

        # Thread-safe data (written by ROS callbacks, read by QTimer)
        self._lock = threading.Lock()
        self.thermal_nodes = {}     # name -> dict
        self.thermal_links = []     # list of dicts
        self.tank_status = None
        self.loop_status = None
        self.node_data_received = False
        self.link_data_received = False

        # ROS subscriptions
        self._subs = []
        self._subs.append(self.node.create_subscription(
            ThermalNodeDataArray, '/thermal/nodes/state', self._node_cb, 10))
        self._subs.append(self.node.create_subscription(
            ThermalLinkFlowsArray, '/thermal/links/flux', self._link_cb, 10))
        self._subs.append(self.node.create_subscription(
            TankStatus, '/tcs/ammonia_status', self._tank_cb, 10))
        
        # Qt UI
        self._build_ui()

        # Graphics state
        self.node_items = {}  # name -> ThermalNodeItem
        self.link_items = {}  # "a__b" -> ThermalLinkItem
        self._known_topology = set()
        self.layout_computed = False

        # Timers (GUI thread)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_display)     # data + plots + redraw
        self.timer.start(100)  # 10 Hz is responsive, keeps CPU low

        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick_animation)  # animate dashed links
        self._anim_timer.start(33)  # ~30 FPS

    # -------- ROS callbacks (no Qt access here!) --------
    def _node_cb(self, msg: ThermalNodeDataArray):
        with self._lock:
            self.thermal_nodes.clear()
            for n in msg.nodes:
                self.thermal_nodes[n.name] = {
                    'temperature_K': n.temperature,     # assuming K
                    'heat_capacity': n.heat_capacity,
                    'internal_power': n.internal_power
                }
            self.node_data_received = True

    def _link_cb(self, msg: ThermalLinkFlowsArray):
        with self._lock:
            self.thermal_links = [{
                'node_a': l.node_a, 'node_b': l.node_b,
                'conductance': l.conductance, 'heat_flow': l.heat_flow
            } for l in msg.links]
            self.link_data_received = True

    def _tank_cb(self, msg: TankStatus):
        with self._lock:
            self.tank_status = {
                'capacity': msg.tank_capacity,
                'temperature_K': msg.tank_temperature.temperature,  # K
                'pressure_Pa': msg.tank_pressure.fluid_pressure,
                'heater_on': msg.tank_heater_on
            }

    
    # -------- Qt UI & render loop --------
    def _build_ui(self):
        self.setWindowTitle("Space Station Thermal Visualization")

        # Layout
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)
        splitter.setStretchFactor(0, 3)  # graph
        splitter.setStretchFactor(1, 1)  # info panel

        # Graph area
        graph_frame = QFrame()
        graph_layout = QVBoxLayout(graph_frame)

        title = QLabel("Thermal Network Visualization")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        graph_layout.addWidget(title)

        self.scene = QGraphicsScene()
        self.scene.setBackgroundBrush(QColor("#202020"))

        self.view = ZoomableGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.view.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.MinimalViewportUpdate)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.AsNeeded)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.AsNeeded)

        graph_layout.addWidget(self.view)
        splitter.addWidget(graph_frame)

        # Right info panel
        info_frame = QFrame()
        info_frame.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        info_layout = QVBoxLayout(info_frame)

        # System status
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout(status_group)
        self.status_label = QLabel("Waiting for thermal data...")
        self.status_label.setStyleSheet("color: orange; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        info_layout.addWidget(status_group)

        # Coolant information
        tanks_group = QGroupBox("Coolant Information")
        tanks_layout = QFormLayout(tanks_group)
        self.ammonia_temp_label = QLabel("--")
        self.ammonia_pressure_label = QLabel("--")
        self.ammonia_heater_label = QLabel("--")
        self.loop_a_temp_label = QLabel("--")
        self.loop_b_temp_label = QLabel("--")
        tanks_layout.addRow("Ammonia Temp (°C):", self.ammonia_temp_label)
        tanks_layout.addRow("Ammonia Pressure (Pa):", self.ammonia_pressure_label)
        tanks_layout.addRow("Heater Status:", self.ammonia_heater_label)
        tanks_layout.addRow("Loop A Temp (°C):", self.loop_a_temp_label)
        tanks_layout.addRow("Loop B Temp (°C):", self.loop_b_temp_label)
        info_layout.addWidget(tanks_group)

        # Hover details (node/link)
        details_group = QGroupBox("")
        self.details_group = details_group
        details_layout = QFormLayout(details_group)
        # Node fields
        self._node_fields = [(QLabel("Node:"), QLabel("")),
                             (QLabel("Temperature:"), QLabel("")),
                             (QLabel("Heat Capacity:"), QLabel("")),
                             (QLabel("Internal Power:"), QLabel(""))]
        for l, v in self._node_fields:
            l.hide(); v.hide()
            details_layout.addRow(l, v)
        # Link fields
        self._link_fields = [(QLabel("Link:"), QLabel("")),
                             (QLabel("Conductance:"), QLabel("")),
                             (QLabel("Heat Flow:"), QLabel(""))]
        for l, v in self._link_fields:
            l.hide(); v.hide()
            details_layout.addRow(l, v)
        info_layout.addWidget(details_group)

        # Legend
        legend_group = QGroupBox("Legend")
        legend_layout = QVBoxLayout()
        legend_text = (
            "<b>Node Colors:</b><br>"
            "• Blue = Cold &nbsp;&nbsp;&nbsp;→&nbsp; Red = Hot<br><br>"
            "<b>Node Size:</b><br>"
            "• Larger = Higher heat capacity<br><br>"
            "<b>Link Style:</b><br>"
            "• Thicker = Higher conductance (W/K)<br>"
            "• Dash motion speed ∝ heat flow / conductance"
        )
        legend_label = QLabel(legend_text)
        legend_label.setStyleSheet("background-color: #2a2a2a; color: #eee; padding: 8px;")
        legend_layout.addWidget(legend_label)
        legend_group.setLayout(legend_layout)
        info_layout.addWidget(legend_group)

        # Live plot (Avg Node Temp in K)
        plot_group = QGroupBox("Avg Node Temp (Live, K)")
        plot_layout = QVBoxLayout()
        self.temp_plot = self._create_temp_plot()
        plot_layout.addWidget(self.temp_plot)
        plot_group.setLayout(plot_layout)
        info_layout.addWidget(plot_group)

        splitter.addWidget(info_frame)

        # Initial split sizes
        splitter.setSizes([900, 360])
        layout.addWidget(splitter)

        # Scene rect
        self.scene.setSceneRect(-600, -400, 1200, 800)

    def _create_temp_plot(self):
        self.temp_history = deque(maxlen=300)
        self.temp_time = deque(maxlen=300)

        fig, self.temp_ax = plt.subplots(figsize=(3.5, 2.5), dpi=100)
        # Never specify colors unless asked — but single default is OK.
        (self.temp_line,) = self.temp_ax.plot([], [], '-')

        self.temp_ax.set_title("Avg Thermal Node Temp (K)")
        self.temp_ax.set_xlabel("Time (s)")
        self.temp_ax.set_ylabel("Temperature (K)")
        self.temp_ax.set_ylim(270, 340)
        self.temp_ax.grid(True)

        canvas = FigureCanvas(fig)
        return canvas

    # Preferred sizes for the tab
    def sizeHint(self) -> QSize:
        return QSize(960, 540)

    def minimumSizeHint(self) -> QSize:
        return QSize(640, 400)

    # -------- Display / Animation --------
    def _tick_animation(self):
        # Smooth link dash animation (~30 FPS)
        for link in self.link_items.values():
            link.advance_dash(dt_seconds=1.0/30.0)
            link.update_line()
        # Minimal scene update
        if self.link_items:
            self.scene.update()

    def _update_display(self):
        # Copy shared state under lock
        with self._lock:
            nodes = dict(self.thermal_nodes)
            links = list(self.thermal_links)
            node_ok = self.node_data_received
            link_ok = self.link_data_received
            tank = dict(self.tank_status) if self.tank_status else None
            loop = dict(self.loop_status) if self.loop_status else None

        if node_ok:
            self.status_label.setText("Thermal system active — data received")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")

            self._update_graph(nodes, links)
            if link_ok:
                self._update_links(links)

            # Update link geometry after node moves
            for l in self.link_items.values():
                l.update_line()

        self._update_tank_info(tank, loop)

        # Live plot
        if node_ok and nodes:
            tempsK = [n["temperature_K"] for n in nodes.values()]
            if tempsK:
                avgK = sum(tempsK) / len(tempsK)
                self.temp_history.append(avgK)
                self.temp_time.append(len(self.temp_time))
                self.temp_line.set_data(self.temp_time, self.temp_history)
                self.temp_ax.set_xlim(0, max(30, len(self.temp_time)))
                # draw_idle prevents recursive repaint storms
                self.temp_ax.figure.canvas.draw_idle()

    def _topology_changed(self, current_names: set) -> bool:
        if current_names != self._known_topology:
            self._known_topology = set(current_names)
            return True
        return False

    def _update_graph(self, nodes, links):
        declared = set(nodes.keys())
        referenced = set()
        for l in links:
            referenced.add(l['node_a'])
            referenced.add(l['node_b'])
        all_names = declared.union(referenced)

        # Remove vanished nodes
        for name in list(self.node_items.keys()):
            if name not in all_names:
                self.scene.removeItem(self.node_items[name])
                del self.node_items[name]
                # links referencing this node will be cleaned below

        # Create/update nodes
        for name in all_names:
            data = nodes.get(name, self.NODE_PLACEHOLDER)
            if name not in self.node_items:
                x = random.uniform(-150, 150)
                y = random.uniform(-100, 100)
                item = ThermalNodeItem(
                    name,
                    data.get('temperature_K', self.NODE_PLACEHOLDER['temperature_K']),
                    data.get('heat_capacity', self.NODE_PLACEHOLDER['heat_capacity']),
                    data.get('internal_power', self.NODE_PLACEHOLDER['internal_power']),
                    x, y,
                    hover_callback=self._on_item_hover
                )
                # Default temp range/colors (can later be parameterized)
                item.set_temp_range(270.0, 330.0, QColor("blue"), QColor("red"))
                self.node_items[name] = item
                self.scene.addItem(item)
            else:
                self.node_items[name].update_data(
                    data.get('temperature_K', self.NODE_PLACEHOLDER['temperature_K']),
                    data.get('heat_capacity', self.NODE_PLACEHOLDER['heat_capacity']),
                    data.get('internal_power', self.NODE_PLACEHOLDER['internal_power'])
                )

        # Compute layout once per topology (or when it changes)
        if self._topology_changed(all_names) or not self.layout_computed:
            if len(links) > 0:
                self._force_layout(links, iterations=self.FORCE_LAYOUT_ITER)
            else:
                self._grid_layout()
            self._fit_view(self.VIEW_MARGIN)
            self.layout_computed = True

    def _grid_layout(self):
        names = list(self.node_items.keys())
        N = len(names)
        cols = max(1, int(math.ceil(math.sqrt(N))))
        spacing = self.GRID_SPACING
        for i, name in enumerate(names):
            r = i // cols
            c = i % cols
            x = (c - cols/2)*spacing
            y = (r - cols/2)*spacing
            self.node_items[name].setPos(x, y)

    def _force_layout(self, links, iterations=60):
        names = list(self.node_items.keys())
        N = len(names)
        if N == 0:
            return
        idx = {n: i for i, n in enumerate(names)}

        # spring weights by conductance
        springs = {}
        for l in links:
            a, b = l['node_a'], l['node_b']
            if a in idx and b in idx:
                ia, ib = idx[a], idx[b]
                w = max(1e-6, float(l.get('conductance', 1.0)))
                springs[(ia, ib)] = springs.get((ia, ib), 0.0) + w
                springs[(ib, ia)] = springs.get((ib, ia), 0.0) + w

        pos = []
        for n in names:
            p = self.node_items[n].pos()
            pos.append([p.x(), p.y()])

        rect = self.scene.sceneRect()
        area = max(1.0, rect.width()*rect.height())
        k = max(40.0, math.sqrt(area/max(1, N)))
        repulsion = 5.0
        attraction_scale = 3.0
        max_disp = 30.0

        for _ in range(iterations):
            disp = [[0.0, 0.0] for _ in range(N)]
            # repulsion
            for i in range(N):
                xi, yi = pos[i]
                for j in range(i+1, N):
                    xj, yj = pos[j]
                    dx, dy = xi-xj, yi-yj
                    d2 = dx*dx + dy*dy + 1e-6
                    d = math.sqrt(d2)
                    if d < 1e-3:
                        dx, dy = random.uniform(-0.1,0.1), random.uniform(-0.1,0.1)
                        d = math.sqrt(dx*dx+dy*dy)
                    force = repulsion*(k*k)/d2
                    ux, uy = dx/d, dy/d
                    disp[i][0] += ux*force; disp[i][1] += uy*force
                    disp[j][0] -= ux*force; disp[j][1] -= uy*force
            # springs
            for (ia, ib), conduct in springs.items():
                xa, ya = pos[ia]; xb, yb = pos[ib]
                dx, dy = xa-xb, ya-yb
                d = math.sqrt(dx*dx+dy*dy) + 1e-6
                desired = k*(1.0/(1.0+math.sqrt(conduct)))
                force = attraction_scale*conduct*(d - desired)
                ux, uy = dx/d, dy/d
                disp[ia][0] -= ux*force; disp[ia][1] -= uy*force
            # apply
            for i in range(N):
                dx, dy = disp[i]
                m = math.hypot(dx, dy)
                if m > 0:
                    s = min(max_disp, m)/m
                    pos[i][0] += dx*s; pos[i][1] += dy*s

        margin = 50
        left, right = rect.left()+margin, rect.right()-margin
        top, bottom = rect.top()+margin, rect.bottom()-margin
        for i, n in enumerate(names):
            x, y = pos[i]
            x = max(left, min(right, x))
            y = max(top, min(bottom, y))
            self.node_items[n].setPos(x, y)

    def _fit_view(self, margin=20):
        if not self.node_items:
            return
        br = None
        for it in self.node_items.values():
            r = it.mapToScene(it.boundingRect()).boundingRect()
            br = r if br is None else br.united(r)
        for it in self.link_items.values():
            r = it.boundingRect()
            br = r if br is None else br.united(r)
        if br is None:
            return
        br = br.adjusted(-margin, -margin, margin, margin)
        self.view.reset_zoom()
        self.view.fitInView(br, Qt.AspectRatioMode.KeepAspectRatio)

    def _update_links(self, links):
        current = set()
        for l in links:
            a, b = l['node_a'], l['node_b']
            if a not in self.node_items or b not in self.node_items:
                continue
            key = f"{a}__{b}"
            current.add(key)
            if key not in self.link_items:
                item = ThermalLinkItem(self.node_items[a], self.node_items[b],
                                       l['conductance'], l['heat_flow'],
                                       hover_callback=self._on_item_hover)
                self.link_items[key] = item
                self.scene.addItem(item)
            else:
                self.link_items[key].update_data(l['conductance'], l['heat_flow'])

        # Remove stale links
        for k in list(self.link_items.keys()):
            if k not in current:
                self.scene.removeItem(self.link_items[k])
                del self.link_items[k]

    def _update_tank_info(self, tank, loop):
        if tank:
            temp_c = tank['temperature_K'] - 273.15
            self.ammonia_temp_label.setText(f"{temp_c:.1f}°C")
            self.ammonia_pressure_label.setText(f"{tank['pressure_Pa']:.0f} Pa")
            self.ammonia_heater_label.setText("ON" if tank['heater_on'] else "OFF")
        if loop:
            self.loop_a_temp_label.setText(f"{loop['loop_a_temp_C']:.1f}°C")
            self.loop_b_temp_label.setText(f"{loop['loop_b_temp_C']:.1f}°C")

    # Hover details
    def _on_item_hover(self, item, kind):
        if item is None:
            self._set_details(None, None)
        elif kind == 'node':
            vals = [item.name,
                    f"{item.temperature_K:.1f} K",
                    f"{item.heat_capacity:.1f} J/K",
                    f"{item.internal_power:.1f} W"]
            self._set_details(vals, None)
        elif kind == 'link':
            a, b = item.from_node_item.name, item.to_node_item.name
            vals = [f"{a} ↔ {b}",
                    f"{item.conductance:.3f} W/K",
                    f"{item.heat_flow:.2f} W"]
            self._set_details(None, vals)

    def _set_details(self, node_vals, link_vals):
        self.details_group.setTitle("Node Details" if node_vals else ("Link Details" if link_vals else ""))
        for (l, v) in self._node_fields:
            l.hide(); v.hide(); v.setText("")
        for (l, v) in self._link_fields:
            l.hide(); v.hide(); v.setText("")
        if node_vals:
            for (l, v), val in zip(self._node_fields, node_vals):
                v.setText(val); l.show(); v.show()
        if link_vals:
            for (l, v), val in zip(self._link_fields, link_vals):
                v.setText(val); l.show(); v.show()

