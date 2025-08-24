from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout,
    QGraphicsScene, QGraphicsView, QGraphicsEllipseItem, QGraphicsLineItem,
    QGraphicsTextItem, QGraphicsItem, QFrame, QSplitter, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QFont, QPen, QBrush, QColor, QTransform, QPainter
import math, random, threading

import rclpy
from rclpy.node import Node
from space_station_thermal_control.msg import (
    ThermalNodeDataArray, ThermalLinkFlowsArray, TankStatus, InternalLoopStatus
)

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,   
    durability=DurabilityPolicy.TRANSIENT_LOCAL  
)
# ---------- Zoomable view ----------
class ZoomableGraphicsView(QGraphicsView):
    MIN_SCALE = 0.2
    MAX_SCALE = 5.0
    def __init__(self, *args, min_scale=MIN_SCALE, max_scale=MAX_SCALE, **kwargs):
        super().__init__(*args, **kwargs)
        self._zoom = 1.0
        self._min_scale = min_scale
        self._max_scale = max_scale
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
    def wheelEvent(self, event):
        factor = 1.25 if event.angleDelta().y() > 0 else 1/1.25
        new_zoom = self._zoom * factor
        if self._min_scale <= new_zoom <= self._max_scale:
            self._zoom = new_zoom
            self.scale(factor, factor)
    def reset_zoom(self):
        self.setTransform(QTransform()); self._zoom = 1.0

# ---------- Graphics items ----------
class ThermalNodeItem(QGraphicsEllipseItem):
    MIN_SIZE, MAX_SIZE, SIZE_SCALE = 20, 60, 40
    DEFAULT_FONT_SIZE = 7
    def __init__(self, name, temperature, heat_capacity, internal_power, x=0, y=0, hover_callback=None):
        super().__init__()
        self.name = name
        self.temperature = temperature
        self.heat_capacity = heat_capacity
        self.internal_power = internal_power
        self.hover_callback = hover_callback
        size = self._compute_size(heat_capacity)
        self.size = size
        self.setRect(-size/2, -size/2, size, size)
        self.setPos(x, y)
        self._apply_color()
        self.setPen(QPen(Qt.black, 2))
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setAcceptHoverEvents(True)
        self.setZValue(2)
        self.text_item = QGraphicsTextItem(self.name, parent=self)
        self.text_item.setDefaultTextColor(Qt.white)
        self.text_item.setFont(QFont("Arial", self.DEFAULT_FONT_SIZE))
        tr = self.text_item.boundingRect()
        self.text_item.setPos(-tr.width()/2, -tr.height()/2)
        self.setToolTip(self._tooltip_text())
    def _compute_size(self, heat_capacity):
        return max(self.MIN_SIZE, min(self.MAX_SIZE, heat_capacity/1000.0*self.SIZE_SCALE))
    def _tooltip_text(self):
        return (f"{self.name}\nT: {self.temperature:.1f} K\n"
                f"Heat Cap: {self.heat_capacity:.1f} J/K\nPower: {self.internal_power:.1f} W")
    def _apply_color(self):
        temp_ratio = max(0.0, min(1.0, (self.temperature - 275.0)/50.0))
        red = int(255*temp_ratio); blue = int(255*(1.0-temp_ratio))
        self.setBrush(QBrush(QColor(red, 0, blue)))
    def update_data(self, temperature, heat_capacity, internal_power):
        self.temperature = temperature; self.heat_capacity = heat_capacity; self.internal_power = internal_power
        size = self._compute_size(heat_capacity)
        self.size = size
        self.setRect(-size/2, -size/2, size, size)
        self._apply_color()
        self.setToolTip(self._tooltip_text())
    def hoverEnterEvent(self, e):
        if callable(self.hover_callback): self.hover_callback(self, 'node')
        self.setPen(QPen(Qt.yellow, 2)); super().hoverEnterEvent(e)
    def hoverMoveEvent(self, e):
        if callable(self.hover_callback): self.hover_callback(self, 'node')
        self.setPen(QPen(Qt.yellow, 2)); super().hoverMoveEvent(e)
    def hoverLeaveEvent(self, e):
        if callable(self.hover_callback): self.hover_callback(None, None)
        self.setPen(QPen(Qt.black, 2)); super().hoverLeaveEvent(e)

class ThermalLinkItem(QGraphicsLineItem):
    MIN_WIDTH, MAX_WIDTH = 1, 8
    def __init__(self, from_node_item, to_node_item, conductance, heat_flow, hover_callback=None):
        super().__init__()
        self.from_node_item = from_node_item
        self.to_node_item = to_node_item
        self.conductance = conductance
        self.heat_flow = heat_flow
        self.hover_callback = hover_callback
        self.setAcceptHoverEvents(True); self.setZValue(1)
        self._apply_pen(); self.update_line()
        self.setAcceptedMouseButtons(Qt.NoButton)
    def _apply_pen(self):
        width = max(self.MIN_WIDTH, min(self.MAX_WIDTH, abs(self.heat_flow)))
        pen = QPen(Qt.white, width); pen.setCapStyle(Qt.RoundCap)
        self.setPen(pen)
    def update_data(self, conductance, heat_flow):
        self.conductance = conductance; self.heat_flow = heat_flow
        self._apply_pen(); self.update_line()
    def update_line(self):
        a = self.from_node_item.scenePos(); b = self.to_node_item.scenePos()
        self.setLine(a.x(), a.y(), b.x(), b.y())
    def hoverEnterEvent(self, e):
        if callable(self.hover_callback): self.hover_callback(self, 'link')
        p = self.pen(); p.setColor(Qt.yellow); self.setPen(p); super().hoverEnterEvent(e)
    def hoverMoveEvent(self, e):
        if callable(self.hover_callback): self.hover_callback(self, 'link')
        p = self.pen(); p.setColor(Qt.yellow); self.setPen(p); super().hoverMoveEvent(e)
    def hoverLeaveEvent(self, e):
        if callable(self.hover_callback): self.hover_callback(None, None)
        self._apply_pen(); super().hoverLeaveEvent(e)

# ---------- Main widget (Qt + ROS) ----------
class ThermalWidget(QWidget):
    NODE_PLACEHOLDER = {'temperature': 300.0, 'heat_capacity': 1000.0, 'internal_power': 0.0}
    GRID_SPACING = 80
    FORCE_LAYOUT_ITER = 200
    VIEW_MARGIN = 40

    def __init__(self, gui_node: Node, parent=None):
        super().__init__(parent)
        self.node = gui_node  # shared ROS2 node
        self.node.get_logger().info("[ThermalWidget] Initializing")

        # Thread-safe data (written by ROS callbacks, read by QTimer)
        self._lock = threading.Lock()
        self.thermal_nodes = {}     # name -> dict
        self.thermal_links = []     # list of dicts
        self.tank_status = None
        self.loop_status = None
        self.node_data_received = False
        self.link_data_received = False

        # ROS subscriptions (created on the shared Node)
        self._subs = []
        self._subs.append(self.node.create_subscription(
            ThermalNodeDataArray, '/thermal/nodes/state', self._node_cb, 10))
        self._subs.append(self.node.create_subscription(
            ThermalLinkFlowsArray, '/thermal/links/flux', self._link_cb, 10))
        self._subs.append(self.node.create_subscription(
            TankStatus, '/tcs/ammonia_status', self._tank_cb, 10))
        self._subs.append(self.node.create_subscription(
            InternalLoopStatus, '/tcs/internal_loop_heat', self._loop_cb, 10))

        # Qt UI
        self._build_ui()

        # Graphics state
        self.node_items = {}  # name -> ThermalNodeItem
        self.link_items = {}  # "a__b" -> ThermalLinkItem
        self.layout_computed = False

        # QTimer in GUI thread
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_display)
        self.timer.start(100)  # 10 Hz feels responsive

    # -------- ROS callbacks (no Qt access here!) --------
    def _node_cb(self, msg: ThermalNodeDataArray):
        with self._lock:
            self.thermal_nodes.clear()
            for n in msg.nodes:
                self.thermal_nodes[n.name] = {
                    'temperature': n.temperature,
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
                'temperature': msg.tank_temperature.temperature,
                'pressure': msg.tank_pressure.fluid_pressure,
                'heater_on': msg.tank_heater_on
            }

    def _loop_cb(self, msg: InternalLoopStatus):
        with self._lock:
            self.loop_status = {
                'loop_a_temp': msg.loop_a.temperature,
                'loop_b_temp': msg.loop_b.temperature
            }

    # -------- Qt UI & render loop --------
    def _build_ui(self):
        self.setWindowTitle("Space Station Thermal Visualization")

        # Layout: small margins so content doesn't look oversized
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)
        splitter.setStretchFactor(0, 3)  # graph
        splitter.setStretchFactor(1, 1)  # info panel

        # Graph area
        graph_frame = QFrame()
        graph_layout = QVBoxLayout(graph_frame)
        title = QLabel("Thermal Network Visualization")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        graph_layout.addWidget(title)

        self.scene = QGraphicsScene()
        self.scene.setBackgroundBrush(QColor("#202020"))

        self.view = ZoomableGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        # keep the view flexible instead of forcing a big minimum
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # lighter update mode helps performance and avoids forcing full repaints
        self.view.setViewportUpdateMode(QGraphicsView.MinimalViewportUpdate)

        graph_layout.addWidget(self.view)
        splitter.addWidget(graph_frame)

        # Right info panel
        info_frame = QFrame()
        info_frame.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        info_layout = QVBoxLayout(info_frame)

        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout(status_group)
        self.status_label = QLabel("Waiting for thermal data...")
        self.status_label.setStyleSheet("color: orange; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        info_layout.addWidget(status_group)

        tanks_group = QGroupBox("Coolant Information")
        tanks_layout = QFormLayout(tanks_group)
        self.ammonia_temp_label = QLabel("--")
        self.ammonia_pressure_label = QLabel("--")
        self.ammonia_heater_label = QLabel("--")
        self.loop_a_temp_label = QLabel("--")
        self.loop_b_temp_label = QLabel("--")
        tanks_layout.addRow("Ammonia Temp:", self.ammonia_temp_label)
        tanks_layout.addRow("Ammonia Pressure:", self.ammonia_pressure_label)
        tanks_layout.addRow("Heater Status:", self.ammonia_heater_label)
        tanks_layout.addRow("Loop A Temp:", self.loop_a_temp_label)
        tanks_layout.addRow("Loop B Temp:", self.loop_b_temp_label)
        info_layout.addWidget(tanks_group)

        details_group = QGroupBox("")
        self.details_group = details_group
        details_layout = QFormLayout(details_group)
        # Node details
        self._node_fields = [(QLabel("Node:"), QLabel("")),
                             (QLabel("Temperature:"), QLabel("")),
                             (QLabel("Heat Capacity:"), QLabel("")),
                             (QLabel("Internal Power:"), QLabel(""))]
        for l, v in self._node_fields:
            l.hide(); v.hide(); details_layout.addRow(l, v)
        # Link details
        self._link_fields = [(QLabel("Link:"), QLabel("")),
                             (QLabel("Conductance:"), QLabel("")),
                             (QLabel("Heat Flow:"), QLabel(""))]
        for l, v in self._link_fields:
            l.hide(); v.hide(); details_layout.addRow(l, v)
        info_layout.addWidget(details_group)

        legend_group = QGroupBox("Legend")
        legend_layout = QVBoxLayout(legend_group)
        legend_label = QLabel(
            "<b>Node Colors:</b><br>• Blue = Cold (275K)<br>• Red = Hot (325K)<br><br>"
            "<b>Node Size:</b><br>• Larger = Higher heat capacity<br><br>"
            "<b>Arrow Width:</b><br>• Thicker = Higher heat flow<br><br>"
        )
        legend_label.setStyleSheet("background-color: #f0f0f0; padding: 10px;")
        legend_layout.addWidget(legend_label)
        info_layout.addWidget(legend_group)

        splitter.addWidget(info_frame)

        # Set initial proportion between graph and info panel
        splitter.setSizes([900, 360])

        layout.addWidget(splitter)

        # Scene rect is fine; doesn't force widget size
        self.scene.setSceneRect(-600, -400, 1200, 800)

    # Preferred sizes for the tab (window can still resize freely)
    def sizeHint(self) -> QSize:
        return QSize(960, 540)

    def minimumSizeHint(self) -> QSize:
        return QSize(640, 400)

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
            self.status_label.setText("Thermal system active - Data received")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self._update_graph(nodes, links)
            if link_ok:
                self._update_links(links)
            for l in self.link_items.values():
                l.update_line()

        self._update_tank_info(tank, loop)

    def _update_graph(self, nodes, links):
        declared = set(nodes.keys())
        referenced = set()
        for l in links:
            referenced.add(l['node_a']); referenced.add(l['node_b'])
        all_names = list(declared.union(referenced))

        # Remove vanished nodes
        for name in list(self.node_items.keys()):
            if name not in all_names:
                self.scene.removeItem(self.node_items[name])
                del self.node_items[name]

        # Create/update nodes
        for name in all_names:
            data = nodes.get(name, self.NODE_PLACEHOLDER)
            if name not in self.node_items:
                x = random.uniform(-150, 150); y = random.uniform(-100, 100)
                item = ThermalNodeItem(name, data['temperature'], data['heat_capacity'],
                                       data['internal_power'], x, y, hover_callback=self._on_item_hover)
                self.node_items[name] = item
                self.scene.addItem(item)
            else:
                self.node_items[name].update_data(
                    data['temperature'], data['heat_capacity'], data['internal_power'])

        # Layout on first topology build
        if not self.layout_computed and len(self.node_items) > 0:
            if len(links) > 0:
                self._force_layout(links, iterations=self.FORCE_LAYOUT_ITER)
            else:
                self._grid_layout()
            self._fit_view(self.VIEW_MARGIN)
            self.layout_computed = True

    def _grid_layout(self):
        names = list(self.node_items.keys())
        N = len(names); cols = max(1, int(math.ceil(math.sqrt(N))))
        spacing = self.GRID_SPACING
        for i, name in enumerate(names):
            r = i // cols; c = i % cols
            x = (c - cols/2)*spacing; y = (r - cols/2)*spacing
            self.node_items[name].setPos(x, y)

    def _force_layout(self, links, iterations=60):
        names = list(self.node_items.keys()); N = len(names)
        if N == 0: return
        idx = {n:i for i,n in enumerate(names)}
        springs = {}
        for l in links:
            a,b = l['node_a'], l['node_b']
            if a in idx and b in idx:
                ia, ib = idx[a], idx[b]
                w = max(1e-6, l.get('conductance', 1.0))
                springs[(ia, ib)] = springs.get((ia, ib), 0.0) + w
                springs[(ib, ia)] = springs.get((ib, ia), 0.0) + w
        pos = []
        for n in names:
            p = self.node_items[n].pos(); pos.append([p.x(), p.y()])

        rect = self.scene.sceneRect()
        area = max(1.0, rect.width()*rect.height())
        k = max(40.0, math.sqrt(area/max(1, N)))
        repulsion = 5.0; attraction_scale = 3.0; max_disp = 30.0

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
        if not self.node_items: return
        br = None
        for it in self.node_items.values():
            r = it.mapToScene(it.boundingRect()).boundingRect()
            br = r if br is None else br.united(r)
        for it in self.link_items.values():
            r = it.boundingRect()
            br = r if br is None else br.united(r)
        if br is None: return
        br = br.adjusted(-margin, -margin, margin, margin)
        self.view.reset_zoom()
        self.view.fitInView(br, Qt.KeepAspectRatio)

    def _update_links(self, links):
        current = set()
        for l in links:
            a, b = l['node_a'], l['node_b']
            if a not in self.node_items or b not in self.node_items:
                continue
            key = f"{a}__{b}"; current.add(key)
            if key not in self.link_items:
                item = ThermalLinkItem(self.node_items[a], self.node_items[b],
                                       l['conductance'], l['heat_flow'],
                                       hover_callback=self._on_item_hover)
                self.link_items[key] = item
                self.scene.addItem(item)
            else:
                self.link_items[key].update_data(l['conductance'], l['heat_flow'])
        for k in list(self.link_items.keys()):
            if k not in current:
                self.scene.removeItem(self.link_items[k]); del self.link_items[k]

    def _update_tank_info(self, tank, loop):
        if tank:
            self.ammonia_temp_label.setText(f"{tank['temperature']:.1f}°C")
            self.ammonia_pressure_label.setText(f"{tank['pressure']:.0f} Pa")
            self.ammonia_heater_label.setText("ON" if tank['heater_on'] else "OFF")
        if loop:
            self.loop_a_temp_label.setText(f"{loop['loop_a_temp']:.1f}°C")
            self.loop_b_temp_label.setText(f"{loop['loop_b_temp']:.1f}°C")

    # Details panel helpers
    def _on_item_hover(self, item, kind):
        if item is None:
            self._set_details(None, None)
        elif kind == 'node':
            vals = [item.name,
                    f"{item.temperature:.1f} K",
                    f"{item.heat_capacity:.1f} J/K",
                    f"{item.internal_power:.1f} W"]
            self._set_details(vals, None)
        elif kind == 'link':
            a, b = item.from_node_item.name, item.to_node_item.name
            vals = [f"{a} ↔ {b}", f"{item.conductance:.3f} W/K", f"{item.heat_flow:.2f} W"]
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
