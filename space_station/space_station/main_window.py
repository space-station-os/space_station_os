import os
import sys
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QStackedWidget,
    QFrame, QApplication, QScrollArea
)
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from space_station.video_player import VideoPlayer
from space_station import theme

# Subsystem panels (existing ROS wiring preserved inside each)
from space_station.overview import OverviewWidget
from space_station.eclss import EclssWidget
from space_station.gnc import GncWidget
from space_station.eps import EPSWidget
from space_station.thermal import ThermalWidget
from space_station.comms import CommsWidget

# Mission-control shell widgets
from space_station.widgets import StatusBar, NavBar, EventFeed, SubsystemRoster

from space_station.left_panel import LeftPanel
from space_station.agent import SsosAIAgent

# Global SSOS interfaces (optional — degrade gracefully if not built)
try:
    from space_station_interfaces.msg import (
        SystemState, FaultEvent, SubsystemHeartbeat
    )
    _HAVE_SSOS_MSGS = True
except Exception:
    SystemState = FaultEvent = SubsystemHeartbeat = None
    _HAVE_SSOS_MSGS = False

# --- Resources API (Py 3.9+ files/as_file) ---
try:
    from importlib.resources import files, as_file
    _USE_NEW_RESOURCES_API = True
except ImportError:
    import importlib.resources as pkg_resources
    _USE_NEW_RESOURCES_API = False


_STATE_NAMES = {0: "INIT", 1: "NOMINAL", 2: "DEGRADED", 3: "RECOVERY", 4: "SAFE"}
_SEVERITY_NAMES = {0: "warning", 1: "critical", 2: "emergency"}

# Map subsystem_name strings published by ssos nodes -> roster row names.
_SUBSYS_ALIASES = {
    "ars": "ECLSS", "ogs": "ECLSS", "wrs": "ECLSS", "cabin": "ECLSS",
    "eclss": "ECLSS", "gnc": "GNC", "eps": "EPS",
    "thermal": "Thermal", "comms": "Comms",
}


class MainWindow(QMainWindow):
    """
    Mission-control shell: StatusBar + NavBar + stacked subsystem content +
    persistent right sidebar (EventFeed + SubsystemRoster + AI assist).

    Owns its ROS 2 context/node/executor; callbacks pumped via a GUI-thread
    QTimer (no Python threads).
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("SSOS Mission Control")
        # Screen-adaptive sizing: never larger than the available screen, with a
        # usable minimum so the layout stays coherent on small laptops.
        self.setMinimumSize(960, 600)
        screen = QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            self.resize(min(1440, avail.width()), min(900, avail.height()))
        else:
            self.resize(1280, 800)

        # ---- ROS init ----
        self._ros_ctx = rclpy.context.Context()
        rclpy.init(args=None, context=self._ros_ctx)
        self.node: Node = rclpy.create_node('space_station_gui_node',
                                            context=self._ros_ctx)
        self.executor = MultiThreadedExecutor(context=self._ros_ctx, num_threads=4)
        self.executor.add_node(self.node)

        self._ros_timer = QTimer(self)
        self._ros_timer.setInterval(20)
        self._ros_timer.timeout.connect(self._spin_ros_once)
        self._ros_timer.start()

        # ---- UI init ----
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self._build_ui()
        theme.apply_theme(QApplication.instance())

        # ---- Global mission-control telemetry (shell-level subscriptions) ----
        self._init_global_subs()

        # --- AI Agent wiring (preserved) ---
        self.ai_agent = SsosAIAgent(
            self.node,
            base_url=os.environ.get("SSOS_LLM_BASE_URL",
                                    "https://integrate.api.nvidia.com/v1"),
            model=os.environ.get("SSOS_LLM_MODEL", "openai/gpt-oss-20b"),
            api_key=os.environ.get("NVIDIA_API_KEY"),
            request_timeout_s=10.0,
        )
        self.left_panel.ask_ai.connect(self.ai_agent.ask)
        self.ai_agent.ai_reply.connect(self.left_panel.append_ai_response)

        # Startup splash
        self._play_startup_video()

    # ---------------- ROS pump ----------------
    def _spin_ros_once(self):
        if self._ros_ctx.ok():
            try:
                self.executor.spin_once(timeout_sec=0.0)
            except Exception:
                pass

    # ---------------- UI ----------------
    def _build_ui(self):
        root = QVBoxLayout(self.central_widget)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # --- Status bar ---
        self.status_bar = StatusBar()
        root.addWidget(self.status_bar)

        # --- Nav bar ---
        self._nav_labels = ["Overview", "ECLSS", "GNC", "EPS", "Thermal", "Comms"]
        self.nav_bar = NavBar(self._nav_labels)
        self.nav_bar.tab_changed.connect(self._on_tab_changed)
        root.addWidget(self.nav_bar)

        # --- Body: content stack + right sidebar ---
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)

        # Content stack (order MUST match nav labels)
        self.stack = QStackedWidget()
        self.eclss_panel = EclssWidget(self.node)
        self.gnc_panel = GncWidget(self.node)
        self.eps_panel = EPSWidget(self.node)
        self.thermal_panel = ThermalWidget(self.node)
        self.comms_panel = CommsWidget(self.node)
        self.overview_panel = OverviewWidget(self.node)

        self.stack.addWidget(self.overview_panel)  # 0
        self.stack.addWidget(self.eclss_panel)     # 1
        self.stack.addWidget(self.gnc_panel)        # 2
        self.stack.addWidget(self.eps_panel)        # 3
        self.stack.addWidget(self.thermal_panel)    # 4
        self.stack.addWidget(self.comms_panel)      # 5

        content = QWidget()
        cl = QVBoxLayout(content)
        cl.setContentsMargins(20, 18, 20, 18)
        cl.addWidget(self.stack)
        # Wrap content in a resizable scroll area: panels expand to fill on large
        # screens and scroll instead of clipping on small ones.
        content_scroll = QScrollArea()
        content_scroll.setWidgetResizable(True)
        content_scroll.setFrameShape(QFrame.NoFrame)
        content_scroll.setWidget(content)
        body.addWidget(content_scroll, 1)

        # Vertical divider
        vdiv = QFrame()
        vdiv.setProperty("class", "vline")
        vdiv.setFixedWidth(1)
        body.addWidget(vdiv)

        # Right sidebar
        body.addWidget(self._build_sidebar())

        root.addLayout(body, 1)

        # Default landing = Overview
        self.stack.setCurrentIndex(0)
        self.nav_bar.set_active(0)

    def _build_sidebar(self):
        sidebar = QWidget()
        # Responsive sidebar: scales modestly with window width instead of a hard
        # fixed size, so the console adapts to different desktop resolutions.
        sidebar.setMinimumWidth(240)
        sidebar.setMaximumWidth(360)
        sidebar.setStyleSheet(f"background-color: {theme.color('bg2')};")
        v = QVBoxLayout(sidebar)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)

        self.roster = SubsystemRoster(["ECLSS", "GNC", "EPS", "Thermal", "Comms"])
        v.addWidget(self.roster)

        hdiv = QFrame()
        hdiv.setProperty("class", "hline")
        v.addWidget(hdiv)

        self.event_feed = EventFeed(max_events=20)
        v.addWidget(self.event_feed, 1)

        hdiv2 = QFrame()
        hdiv2.setProperty("class", "hline")
        v.addWidget(hdiv2)

        # AI assist (preserved feature)
        self.left_panel = LeftPanel()
        self.left_panel.setMaximumHeight(300)
        v.addWidget(self.left_panel)

        return sidebar

    def _on_tab_changed(self, idx):
        self.stack.setCurrentIndex(idx)

    # ---------------- Global telemetry subscriptions ----------------
    def _init_global_subs(self):
        if not _HAVE_SSOS_MSGS:
            self.event_feed.add_event(
                "SHELL", "ssos interfaces not found — telemetry shell idle", "warning"
            )
            return
        try:
            self.node.create_subscription(
                SystemState, "/ssos/system_state", self._on_system_state, 10)
            self.node.create_subscription(
                FaultEvent, "/ssos/fault_event", self._on_fault_event, 10)
            for sub in ("ars", "ogs", "wrs", "cabin"):
                self.node.create_subscription(
                    SubsystemHeartbeat, f"/ssos/{sub}/heartbeat",
                    self._on_heartbeat, 10)
        except Exception as e:
            self.node.get_logger().warn(f"[shell] global subs failed: {e}")

    def _on_system_state(self, msg):
        state = _STATE_NAMES.get(int(msg.state), "INIT")
        degraded = list(msg.degraded_subsystems)
        QTimer.singleShot(0, lambda: self._apply_system_state(state, degraded))

    def _apply_system_state(self, state, degraded):
        self.status_bar.set_state(state)
        degraded_up = {d.upper() for d in degraded}
        for name in ["ECLSS", "GNC", "EPS", "Thermal", "Comms"]:
            self.roster.set_status(
                name, "degraded" if name.upper() in degraded_up else "nominal")
        self.overview_panel.set_state(state, degraded)

    def _on_fault_event(self, msg):
        sev = _SEVERITY_NAMES.get(int(msg.severity), "warning")
        sub = msg.subsystem_name or "SYSTEM"
        desc = msg.description or msg.fault_type or "fault"
        QTimer.singleShot(0, lambda: self.event_feed.add_event(sub, desc, sev))

    def _on_heartbeat(self, msg):
        row = _SUBSYS_ALIASES.get((msg.subsystem_name or "").lower(), None)
        if row is None:
            return
        status = "nominal" if msg.healthy else "fault"
        QTimer.singleShot(0, lambda: self.roster.set_status(row, status))

    # ---------------- Videos ----------------
    def _play_startup_video(self):
        def after_video():
            pass
        try:
            if _USE_NEW_RESOURCES_API:
                video_resource = files("space_station.assets") / "Ssos_begin.mp4"
                with as_file(video_resource) as path:
                    VideoPlayer(str(path), on_finished_callback=after_video).play()
            else:
                with pkg_resources.path("space_station.assets", "Ssos_begin.mp4") as path:
                    VideoPlayer(str(path), on_finished_callback=after_video).play()
        except Exception:
            pass

    # ---------------- ROS shutdown ----------------
    def _shutdown_ros(self):
        try:
            self._ros_timer.stop()
        except Exception:
            pass
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self._ros_ctx.ok():
                rclpy.shutdown(context=self._ros_ctx)
        except Exception:
            pass

    def closeEvent(self, event):
        self._shutdown_ros()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    theme.apply_theme(app)
    w = MainWindow()
    # Maximized by default so the console fills whatever screen it runs on.
    w.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
