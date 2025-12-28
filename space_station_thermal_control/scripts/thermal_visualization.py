#!/usr/bin/env python3
"""
Space Station Thermal Visualization GUI

This module provides a PyQt5-based visualization for the thermal network of a space station,
including nodes, links, and coolant tank status. It subscribes to ROS2 topics for live data.
"""

import sys
import math
import random
import threading
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox,
    QFormLayout, QGraphicsScene, QGraphicsView, QGraphicsEllipseItem,
    QGraphicsLineItem, QGraphicsTextItem, QGraphicsItem, QFrame, QSplitter
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QPen, QBrush, QColor, QTransform, QPainter
import rclpy
from rclpy.node import Node
from space_station_interfaces.msg import (
    ThermalNodeDataArray, ThermalLinkFlowsArray, TankStatus,
)

class ZoomableGraphicsView(QGraphicsView):
    """Graphics view with scroll-to-zoom and pan support."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._zoom = 1.0
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setDragMode(QGraphicsView.ScrollHandDrag)

    def wheelEvent(self, event):
        """Zoom in/out with mouse wheel."""
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        self._zoom *= factor
        self.scale(factor, factor)

    def reset_zoom(self):
        """Reset transform to identity (useful after fitInView)."""
        self.setTransform(QTransform())
        self._zoom = 1.0

class ThermalNodeItem(QGraphicsEllipseItem):
    """
    Graphics item representing a thermal node.

    Handles hover events to trigger callbacks for displaying node details.
    """

    SIZE_SCALE = 0.04
    DEFAULT_FONT_SIZE = 7
    MIN_TEMP = 0.0
    MAX_TEMP = 80.0
    TEMP_RANGE = 40.0

    def __init__(self, name, temperature, heat_capacity, internal_power, x=0, y=0, hover_callback=None):
        super().__init__()
        self.name = name
        self.temperature = temperature
        self.heat_capacity = heat_capacity
        self.internal_power = internal_power
        self.hover_callback = hover_callback

        size = self._compute_size(heat_capacity)
        self.size = size
        self.setRect(-size / 2, -size / 2, size, size)
        self.setPos(x, y)

        
        self.setPen(QPen(Qt.black, 2))
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setAcceptHoverEvents(True)
        self.setZValue(2)

        self.text_item = QGraphicsTextItem(self.name, parent=self)
        self.text_item.setDefaultTextColor(Qt.white)
        self.text_item.setFont(QFont("Arial", self.DEFAULT_FONT_SIZE))
        text_rect = self.text_item.boundingRect()
        self.text_item.setPos(-text_rect.width() / 2, -text_rect.height() / 2)
        self.text_item.setAcceptHoverEvents(False)

        self.setToolTip(self._tooltip_text())

    def _compute_size(self, heat_capacity):
        """Compute node size based on heat capacity."""
        return heat_capacity * self.SIZE_SCALE

    def _tooltip_text(self):
        """Return tooltip summary for node."""
        return (
            f"{self.name}\n"
            f"T: {self.temperature:.1f} K\n"
            f"Heat Cap: {self.heat_capacity:.1f} J/K\n"
            f"Power: {self.internal_power:.1f} W"
        )

    def set_temp_range(self, min_temp, max_temp, cold_color, hot_color):
        self.MIN_TEMP = min_temp
        self.MAX_TEMP = max_temp
        self.cold_color = cold_color
        self.hot_color = hot_color

    def _apply_color(self):
        ratio = 0.0
        if self.MAX_TEMP > self.MIN_TEMP:
            ratio = (self.temperature - self.MIN_TEMP) / (self.MAX_TEMP - self.MIN_TEMP)
            ratio = max(0.0, min(1.0, ratio))

        r = int(self.cold_color.red()   + ratio * (self.hot_color.red()   - self.cold_color.red()))
        g = int(self.cold_color.green() + ratio * (self.hot_color.green() - self.cold_color.green()))
        b = int(self.cold_color.blue()  + ratio * (self.hot_color.blue()  - self.cold_color.blue()))

        self.setBrush(QBrush(QColor(r, g, b)))

    def update_data(self, temperature, heat_capacity, internal_power):
        """Update node data and appearance."""
        self.temperature = temperature
        self.heat_capacity = heat_capacity
        self.internal_power = internal_power
        size = self._compute_size(heat_capacity)
        self.size = size
        self.setRect(-size / 2, -size / 2, size, size)
        self._apply_color()
        self.setToolTip(self._tooltip_text())

    def hoverEnterEvent(self, event):
        """Handle hover enter event for node ellipse."""
        if event.type() == event.GraphicsSceneHoverEnter and self.contains(event.pos()):
            if callable(self.hover_callback):
                self.hover_callback(self, 'node')
            self.setPen(QPen(Qt.yellow, 2))
        super().hoverEnterEvent(event)

    def hoverMoveEvent(self, event):
        """Handle hover move event for node ellipse."""
        if self.contains(event.pos()):
            if callable(self.hover_callback):
                self.hover_callback(self, 'node')
            self.setPen(QPen(Qt.yellow, 2))
        else:
            if callable(self.hover_callback):
                self.hover_callback(None, None)
            self.setPen(QPen(Qt.black, 2))
        super().hoverMoveEvent(event)

    def hoverLeaveEvent(self, event):
        """Handle hover leave event for node ellipse."""
        if callable(self.hover_callback):
            self.hover_callback(None, None)
        self.setPen(QPen(Qt.black, 2))
        super().hoverLeaveEvent(event)

class ThermalLinkItem(QGraphicsLineItem):
    """
    Graphics item representing a thermal link.

    Handles hover events to trigger callbacks for displaying link details.
    Animated dashed line, width by conductance, dash offset by heat flow.
    """

    def __init__(self, from_node_item, to_node_item, conductance, heat_flow, hover_callback=None):
        super().__init__()
        self.from_node_item = from_node_item
        self.to_node_item = to_node_item
        self.conductance = conductance
        self.heat_flow = heat_flow
        self._is_hovered = False
        self.hover_callback = hover_callback
        self.setAcceptHoverEvents(True)
        self.setZValue(1)
        self.dash_offset = 0.0
        self._animation_timer = QTimer()
        self._animation_timer.timeout.connect(self._animate_dash)
        self._animation_timer.start(30)
        self._apply_pen()
        self.update_line()
        self.setAcceptedMouseButtons(Qt.NoButton)

    def _apply_pen(self):
        """Set pen width by conductance, fixed dash/gap size in pixels, white."""
        width = self.conductance * 7.5
        pen = QPen(Qt.yellow if self._is_hovered else Qt.white, width)
        pen.setCapStyle(Qt.RoundCap)
        pen.setCosmetic(True)
        pen.setDashPattern([2, 2])  
        pen.setDashOffset(self.dash_offset)
        self.setPen(pen)

    def _animate_dash(self):
        """Animate dash offset so that dashed area per unit time ∝ heat flow."""
        if self.conductance > 0:
            speed = (self.heat_flow / self.conductance) * 0.01
        else:
            speed = 0
        self.dash_offset += speed
        self._apply_pen()
        self.update()

    def update_data(self, conductance, heat_flow):
        """Update link data and appearance."""
        self.conductance = conductance
        self.heat_flow = heat_flow
        self._apply_pen()
        self.update_line()

    def update_line(self):
        """Update line endpoints based on node positions."""
        start = self.from_node_item.scenePos()
        end = self.to_node_item.scenePos()
        self.setLine(start.x(), start.y(), end.x(), end.y())

    def hoverEnterEvent(self, event):
        """Handle hover enter event for link."""
        self._is_hovered = True
        if callable(self.hover_callback):
            self.hover_callback(self, 'link')
        pen = self.pen()
        pen.setColor(Qt.yellow)
        self.setPen(pen)
        super().hoverEnterEvent(event)

    def hoverMoveEvent(self, event):
        """Handle hover move event for link."""
        self._is_hovered = True
        if callable(self.hover_callback):
            self.hover_callback(self, 'link')
        pen = self.pen()
        pen.setColor(Qt.yellow)
        self.setPen(pen)
        super().hoverMoveEvent(event)

    def hoverLeaveEvent(self, event):
        """Handle hover leave event for link."""
        self._is_hovered = False
        if callable(self.hover_callback):
            self.hover_callback(None, None)
        self._apply_pen()
        super().hoverLeaveEvent(event)

class ThermalVisualizationNode(Node):
    """ROS2 node for receiving thermal system data."""

    def __init__(self):
        super().__init__('thermal_visualization_node')
        self.node_sub = self.create_subscription(
            ThermalNodeDataArray,
            '/thermal/nodes/state',
            self.node_callback,
            10
        )
        self.link_sub = self.create_subscription(
            ThermalLinkFlowsArray,
            '/thermal/links/flux',
            self.link_callback,
            10
        )
        self.tank_sub = self.create_subscription(
            TankStatus,
            '/tcs/ammonia_status',
            self.tank_callback,
            10
        )
        
        self.declare_parameter("min_temp", 270.0)
        self.declare_parameter("max_temp", 330.0)
        self.declare_parameter("cold_color", "blue")
        self.declare_parameter("hot_color", "red")

        self.min_temp = self.get_parameter("min_temp").value
        self.max_temp = self.get_parameter("max_temp").value
        self.cold_color = QColor(self.get_parameter("cold_color").value)
        self.hot_color = QColor(self.get_parameter("hot_color").value)
        self.add_on_set_parameters_callback(self._on_param_update)
        
        
        self.thermal_nodes = {}
        self.thermal_links = []
        self.tank_status = None
        self.loop_status = None
        self.node_data_received = False
        self.link_data_received = False

    def _on_param_update(self, params):
        for p in params:
            if p.name == "min_temp":
                self.min_temp = p.value
            elif p.name == "max_temp":
                self.max_temp = p.value
            elif p.name == "cold_color":
                self.cold_color = QColor(p.value)
            elif p.name == "hot_color":
                self.hot_color = QColor(p.value)
        # force refresh of node colors
        for item in self.node_items.values():
            item.set_temp_range(self.min_temp, self.max_temp,
                                self.cold_color, self.hot_color)
            item._apply_color()
        return rclpy.parameter.SetParametersResult(successful=True)

    def node_callback(self, msg):
        """Callback for thermal node data."""
        self.thermal_nodes.clear()
        for node in msg.nodes:
            self.thermal_nodes[node.name] = {
                'temperature': node.temperature,
                'heat_capacity': node.heat_capacity,
                'internal_power': node.internal_power
            }
        self.node_data_received = True
        self.get_logger().info(f"Received thermal data for {len(self.thermal_nodes)} nodes",once=True)

    def link_callback(self, msg):
        """Callback for thermal link data."""
        self.thermal_links = []
        for link in msg.links:
            self.thermal_links.append({
                'node_a': link.node_a,
                'node_b': link.node_b,
                'conductance': link.conductance,
                'heat_flow': link.heat_flow
            })
        self.link_data_received = True
        self.get_logger().info(f"Received thermal data for {len(self.thermal_links)} links",once=True)

    def tank_callback(self, msg):
        """Callback for tank status."""
        self.tank_status = {
            'capacity': msg.tank_capacity,
            'temperature': msg.tank_temperature.temperature,
            'pressure': msg.tank_pressure.fluid_pressure,
            'heater_on': msg.tank_heater_on
        }

    

class ThermalVisualizationGUI(QWidget):
    """
    Main GUI widget for thermal visualization.

    Handles layout, updates, and details panel logic.
    """

    # NODE_PLACEHOLDER = {'temperature': 300.0, 'heat_capacity': 1000.0, 'internal_power': 0.0}
    NODE_PLACEHOLDER = {'temperature': 27.0, 'heat_capacity': 1000.0, 'internal_power': 0.0}
    FORCE_LAYOUT_ITER = 200
    VIEW_MARGIN = 40

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Space Station Thermal Visualization")
        self.resize(1400, 900)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(1000)
        self.node_items = {}
        self.link_items = {}
        self.layout_computed = False
        self.selected_type = None
        self.selected_item = None
        self.init_ui()
        
    def init_ui(self):
        """Initialize the user interface and details panel."""
        layout = QHBoxLayout()
        splitter = QSplitter(Qt.Horizontal)

        graph_frame = QFrame()
        graph_layout = QVBoxLayout()
        title = QLabel("Thermal Network Visualization")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 16, QFont.Bold))
        graph_layout.addWidget(title)

        self.scene = QGraphicsScene()
        self.scene.setBackgroundBrush(QColor("#202020"))
        self.view = ZoomableGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.view.setMinimumSize(800, 600)
        graph_layout.addWidget(self.view)
        graph_frame.setLayout(graph_layout)
        splitter.addWidget(graph_frame)

        info_frame = QFrame()
        info_layout = QVBoxLayout()

        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout()
        self.status_label = QLabel("Waiting for thermal data...")
        self.status_label.setStyleSheet("color: orange; font-weight: bold;")
        status_layout.addWidget(self.status_label)
        status_group.setLayout(status_layout)
        info_layout.addWidget(status_group)

        tanks_group = QGroupBox("Coolant Information")
        tanks_layout = QFormLayout()
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
        tanks_group.setLayout(tanks_layout)
        info_layout.addWidget(tanks_group)
        self.hovered_item = None
        details_group = QGroupBox("")
        self.details_group = details_group
        details_layout = QFormLayout()
        self.node_fields = {
            "Node:": (QLabel("Node:"), QLabel("")),
            "Temperature:": (QLabel("Temperature:"), QLabel("")),
            "Heat Capacity:": (QLabel("Heat Capacity:"), QLabel("")),
            "Internal Power:": (QLabel("Internal Power:"), QLabel(""))
        }
        for label_widget, value_widget in self.node_fields.values():
            details_layout.addRow(label_widget, value_widget)
        self.link_fields = {
            "Parent:": (QLabel("Parent:"), QLabel("")),
            "Child:": (QLabel("Child:"), QLabel("")),
            "Conductance:": (QLabel("Conductance:"), QLabel("")),
            "Heat Flow:": (QLabel("Heat Flow:"), QLabel(""))
        }
        for label_widget, value_widget in self.link_fields.values():
            details_layout.addRow(label_widget, value_widget)
        details_group.setLayout(details_layout)
        info_layout.addWidget(details_group)

        legend_group = QGroupBox("Legend")
        legend_layout = QVBoxLayout()
        legend_text = (
            "<b>Node Colors:</b><br>"
            "• Blue = Cold <br>"
            "• Red = Hot <br><br>"
            "<b>Node Size:</b><br>"
            "• Larger = Higher heat capacity<br><br>"
            "<b>Arrow Width:</b><br>"
            "• Thicker = Higher heat flow<br><br>"
        )
        legend_label = QLabel(legend_text)
        legend_label.setStyleSheet("background-color: #f0f0f0; padding: 10px;")
        legend_layout.addWidget(legend_label)
        legend_group.setLayout(legend_layout)
        info_layout.addWidget(legend_group)

        info_frame.setLayout(info_layout)
        splitter.addWidget(info_frame)
        layout.addWidget(splitter)
        self.setLayout(layout)
        self.scene.setSceneRect(-600, -400, 1200, 800)

        
    def update_display(self):
        """Update the display with latest thermal data."""
        if self.node.node_data_received:
            self.status_label.setText("Thermal system active - Data received")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.update_graph()
            self.update_tank_info()
            if self.node.link_data_received:
                self.update_links()
            for link in self.link_items.values():
                link.update_line()
        
        if self.hovered_item is not None and self.hovered_type is not None:
            if self.hovered_type == 'node':
                self.update_node_details(self.hovered_item)
            elif self.hovered_type == 'link':
                self.update_link_details(self.hovered_item)
        else:
            self.clear_details()

    def update_graph(self):
        """Create/update node items and compute layout."""
        declared_nodes = set(self.node.thermal_nodes.keys())
        referenced_nodes = set()
        for l in self.node.thermal_links:
            referenced_nodes.add(l['node_a'])
            referenced_nodes.add(l['node_b'])
        all_node_names = list(declared_nodes.union(referenced_nodes))

        # Remove items not present anymore
        for name in list(self.node_items.keys()):
            if name not in all_node_names:
                self.scene.removeItem(self.node_items[name])
                del self.node_items[name]

        # Create missing node items
        for name in all_node_names:
            data = self.node.thermal_nodes.get(name, None)
            if data is None:
                data = self.NODE_PLACEHOLDER.copy()
            if name not in self.node_items:
                x = random.uniform(-150, 150)
                y = random.uniform(-100, 100)
                item = ThermalNodeItem(
                    name,
                    data['temperature'],
                    data['heat_capacity'],
                    data['internal_power'],
                    x, y,
                    hover_callback=self._on_item_hover
                )
                item.set_temp_range(self.node.min_temp, self.node.max_temp,
                    self.node.cold_color, self.node.hot_color)
                item._apply_color()

                self.node_items[name] = item
                self.scene.addItem(item)
            else:
                item = self.node_items[name]
                item.update_data(data['temperature'], data['heat_capacity'], data['internal_power'])

        # Compute layout only once (or when topology changes)
        if not self.layout_computed and len(self.node_items) > 0:
            self._apply_force_directed_layout(iterations=self.FORCE_LAYOUT_ITER)
            self._fit_view_to_items(margin=self.VIEW_MARGIN)
            self.layout_computed = True

    def _apply_force_directed_layout(self, iterations=60):
        """
        Force-directed layout:
        - Repulsive Coulomb force between all nodes
        - Attractive spring force along each link; strength proportional to conductance
        """
        names = list(self.node_items.keys())
        N = len(names)
        if N == 0:
            return

        idx = {name: i for i, name in enumerate(names)}
        springs = {}
        for l in self.node.thermal_links:
            a = l['node_a']
            b = l['node_b']
            if a in idx and b in idx:
                ia = idx[a]
                ib = idx[b]
                springs[(ia, ib)] = springs.get((ia, ib), 0.0) + max(1e-6, l.get('conductance', 1.0))
                springs[(ib, ia)] = springs.get((ib, ia), 0.0) + max(1e-6, l.get('conductance', 1.0))

        pos = []
        for name in names:
            p = self.node_items[name].pos()
            pos.append([p.x(), p.y()])

        rect = self.scene.sceneRect()
        area = max(1.0, rect.width() * rect.height())
        k = max(40.0, math.sqrt(area / max(1, N)))
        repulsion = 15.0
        attraction_scale = 3
        max_disp = 30.0

        for _ in range(iterations):
            disp = [[0.0, 0.0] for _ in range(N)]
            # Repulsion
            for i in range(N):
                xi, yi = pos[i]
                for j in range(i + 1, N):
                    xj, yj = pos[j]
                    dx = xi - xj
                    dy = yi - yj
                    dist2 = dx * dx + dy * dy + 1e-6
                    dist = math.sqrt(dist2)
                    if dist < 1e-3:
                        dx = random.uniform(-1, 1) * 0.1
                        dy = random.uniform(-1, 1) * 0.1
                        dist = math.sqrt(dx * dx + dy * dy)
                    force = repulsion * (k * k) / dist2
                    ux = dx / dist
                    uy = dy / dist
                    disp[i][0] += ux * force
                    disp[i][1] += uy * force
                    disp[j][0] -= ux * force
                    disp[j][1] -= uy * force
            # Springs
            for (ia, ib), conduct in springs.items():
                xa, ya = pos[ia]
                xb, yb = pos[ib]
                dx = xa - xb
                dy = ya - yb
                dist = math.sqrt(dx * dx + dy * dy) + 1e-6
                desired = k * (1.0 / (1.0 + math.sqrt(conduct)))
                force = attraction_scale * conduct * (dist - desired)
                ux = dx / dist
                uy = dy / dist
                disp[ia][0] -= ux * force
                disp[ia][1] -= uy * force
                disp[ib][0] += ux * force
                disp[ib][1] += uy * force
            # Apply
            for i in range(N):
                dx, dy = disp[i]
                dlen = math.sqrt(dx * dx + dy * dy)
                if dlen > 0:
                    scale = min(max_disp, dlen) / dlen
                    pos[i][0] += dx * scale
                    pos[i][1] += dy * scale

        margin = 50
        left = rect.left() + margin
        right = rect.right() - margin
        top = rect.top() + margin
        bottom = rect.bottom() - margin
        for i, name in enumerate(names):
            x, y = pos[i]
            x = max(left, min(right, x))
            y = max(top, min(bottom, y))
            self.node_items[name].setPos(x, y)

    def _fit_view_to_items(self, margin=20):
        """Fit the view to the bounding rect of all items."""
        if len(self.node_items) == 0:
            return
        br = None
        for item in self.node_items.values():
            r = item.mapToScene(item.boundingRect()).boundingRect()
            br = r if br is None else br.united(r)
        for link in self.link_items.values():
            r = link.boundingRect()
            br = r if br is None else br.united(r)
        if br is None:
            return
        br = br.adjusted(-margin, -margin, margin, margin)
        self.view.reset_zoom()
        self.view.fitInView(br, Qt.KeepAspectRatio)

    def update_links(self):
        """Create/update link graphics for every reported link."""
        current_keys = set()
        for l in self.node.thermal_links:
            a = l['node_b']
            b = l['node_a']
            key = f"{a}__{b}"
            current_keys.add(key)
            if a not in self.node_items or b not in self.node_items:
                continue
            if key not in self.link_items:
                item = ThermalLinkItem(
                    self.node_items[a], self.node_items[b],
                    l['conductance'], l['heat_flow'],
                    hover_callback=self._on_item_hover
                )
                self.link_items[key] = item
                self.scene.addItem(item)
            else:
                item = self.link_items[key]
                item.update_data(l['conductance'], l['heat_flow'])
        for k in list(self.link_items.keys()):
            if k not in current_keys:
                self.scene.removeItem(self.link_items[k])
                del self.link_items[k]

    def update_tank_info(self):
        """Update coolant tank information."""
        if self.node.tank_status:
            tank = self.node.tank_status
            self.ammonia_temp_label.setText(f"{tank['temperature']:.1f}°C")
            self.ammonia_pressure_label.setText(f"{tank['pressure']:.0f} kPa")
            self.ammonia_heater_label.setText("ON" if tank['heater_on'] else "OFF")
        if self.node.loop_status:
            loop = self.node.loop_status
            self.loop_a_temp_label.setText(f"{loop['loop_a_temp']:.1f}°C")
            self.loop_b_temp_label.setText(f"{loop['loop_b_temp']:.1f}°C")

    def update_node_details(self, node_item):
        """Show node details, hide link details."""
        self.details_group.setTitle("Node Details")
        self._set_fields(self.node_fields, [
            node_item.name,
            f"{node_item.temperature:.1f} °C",
            f"{node_item.heat_capacity:.1f} J/°C",
            f"{node_item.internal_power:.1f} W"
        ])
        self._set_fields(self.link_fields, None)

    def update_link_details(self, link_item):
        """Show link details, hide node details."""
        self.details_group.setTitle("Link Details")
        a = link_item.from_node_item.name
        b = link_item.to_node_item.name
        # Show both directions as parent/child for clarity
        self._set_fields(self.link_fields, [
            a,
            b,
            f"{link_item.conductance:.3f} W/°C",
            f"{link_item.heat_flow:.2f} W"
        ])
        self._set_fields(self.node_fields, None)

    def clear_details(self):
        """Show default info when nothing is selected."""
        self.details_group.setTitle("Hover over something for info")
        self._set_fields(self.node_fields, None)
        self._set_fields(self.link_fields, None)


    def _on_item_hover(self, item, item_type):
        """
        Callback for hover events from node/link items.

        Shows details for the hovered item, or clears details if not hovering.
        """
        self.hovered_item = item
        self.hovered_type = item_type
        if item is None:
            self.clear_details()
        elif item_type == 'node':
            self.update_node_details(item)
        elif item_type == 'link':
            self.update_link_details(item)

    def _set_fields(self, field_dict, values):
        """
        Show/hide fields in the details panel.

        If values is None, hide both label and value; else show both and set text.
        """
        if values is None:
            for label_widget, value_widget in field_dict.values():
                label_widget.hide()
                value_widget.hide()
                value_widget.setText("")
        else:
            for (label_widget, value_widget), val in zip(field_dict.values(), values):
                value_widget.setText(val)
                label_widget.show()
                value_widget.show()

def main():
    """Main entry point for the application."""
    rclpy.init()
    node = ThermalVisualizationNode()
    app = QApplication(sys.argv)
    gui = ThermalVisualizationGUI(node)
    gui.show()

    def spin_node():
        """Spin the ROS node in a separate thread."""
        rclpy.spin(node)

    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()