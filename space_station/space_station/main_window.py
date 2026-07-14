import os
import sys
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QStackedWidget,
    QFrame, QApplication, QScrollArea, QPushButton
)
from PyQt5.QtCore import QTimer, Qt
import rclpy
from rclpy.executors import SingleThreadedExecutor
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
from space_station.widgets import (
    StatusBar, NavBar, EventFeed, SubsystemRoster, SimSpeedControl
)

# Live parameter setting (sim speed control)
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType

# Simulation clock (core ROS; independent of the ssos interfaces)
try:
    from rosgraph_msgs.msg import Clock
    _HAVE_CLOCK = True
except Exception:
    Clock = None
    _HAVE_CLOCK = False

from space_station.left_panel import LeftPanel
from space_station.agent import SsosAIAgent
from space_station.param_editor import ParameterEditorDialog
from space_station.fault_injector import FaultInjectorDialog

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
# SubsystemHeartbeat.LIFECYCLE_* -> transition-mode word.
_LIFECYCLE_NAMES = {0: "UNCONFIGURED", 1: "INACTIVE", 2: "ACTIVE", 3: "FINALIZED"}
_ECLSS_NODES = ("ars", "ogs", "wrs", "cabin")

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
        # Single-threaded: spin_once is pumped from the GUI-thread QTimer, so all
        # subscription callbacks run in the GUI thread. This keeps Qt updates
        # (status pill, rosters, event feed) thread-safe and reliable.
        self.executor = SingleThreadedExecutor(context=self._ros_ctx)
        self.executor.add_node(self.node)

        self._ros_timer = QTimer(self)
        self._ros_timer.setInterval(20)
        self._ros_timer.timeout.connect(self._spin_ros_once)
        self._ros_timer.start()

        # Per-node ECLSS lifecycle state cache (name -> (state_word, healthy)).
        self._eclss_states = {}

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

        # --- Nav bar (tabs left, sim-speed control right) ---
        # OGS + WRS live inside the ECLSS panel (one consolidated ECLSS console).
        self._nav_labels = ["Overview", "ECLSS", "GNC", "EPS",
                            "Thermal", "Comms"]
        self.nav_bar = NavBar(self._nav_labels)
        self.nav_bar.tab_changed.connect(self._on_tab_changed)

        nav_row = QWidget()
        nav_row.setStyleSheet(f"background-color: {theme.color('bg2')};")
        nrl = QHBoxLayout(nav_row)
        nrl.setContentsMargins(0, 0, 0, 0)
        nrl.setSpacing(0)
        nrl.addWidget(self.nav_bar)
        nrl.addStretch()
        self.sim_speed = SimSpeedControl()
        self.sim_speed.speed_changed.connect(self._on_sim_speed)
        nrl.addWidget(self.sim_speed)

        # PARAMS: open the live parameter editor (discover + push params to the
        # running sim/ECLSS nodes via the parameter services).
        self.params_btn = QPushButton("PARAMS")
        self.params_btn.setObjectName("params_button")
        self.params_btn.setCursor(Qt.PointingHandCursor)
        self.params_btn.setToolTip("View and edit live simulation parameters")
        self.params_btn.setStyleSheet(
            f"#params_button {{ color: {theme.color('accent')};"
            f" background: transparent; border: 1px solid {theme.color('accent')};"
            f" border-radius: 5px; padding: 6px 16px; font-size: 14px;"
            f" font-weight: bold; letter-spacing: 2px; margin: 0 0 0 16px; }}"
            f"#params_button:hover {{ background: {theme.color('accent')};"
            f" color: {theme.color('bg')}; }}")
        self.params_btn.clicked.connect(self._on_params_clicked)
        nrl.addWidget(self.params_btn)

        # FAULT: open the fault-injection dialog (trigger a chosen ECLSS fault).
        self.fault_btn = QPushButton("FAULT")
        self.fault_btn.setObjectName("fault_button")
        self.fault_btn.setCursor(Qt.PointingHandCursor)
        self.fault_btn.setToolTip("Inject an ECLSS fault")
        self.fault_btn.setStyleSheet(
            f"#fault_button {{ color: {theme.color('amber')};"
            f" background: transparent; border: 1px solid {theme.color('amber')};"
            f" border-radius: 5px; padding: 6px 16px; font-size: 14px;"
            f" font-weight: bold; letter-spacing: 2px; margin: 0 0 0 12px; }}"
            f"#fault_button:hover {{ background: {theme.color('amber')};"
            f" color: {theme.color('bg')}; }}")
        self.fault_btn.clicked.connect(self._on_fault_clicked)
        nrl.addWidget(self.fault_btn)

        # END: clean shutdown of the whole session (GUI exit -> launch Shutdown
        # tears down every ROS node, so nothing is left running).
        self.end_btn = QPushButton("END")
        self.end_btn.setObjectName("end_button")
        self.end_btn.setCursor(Qt.PointingHandCursor)
        self.end_btn.setToolTip("Shut down all subsystems and exit")
        self.end_btn.setStyleSheet(
            f"#end_button {{ color: {theme.color('red')};"
            f" background: transparent; border: 1px solid {theme.color('red')};"
            f" border-radius: 5px; padding: 6px 16px; font-size: 14px;"
            f" font-weight: bold; letter-spacing: 2px; margin: 0 16px; }}"
            f"#end_button:hover {{ background: {theme.color('red')};"
            f" color: {theme.color('bg')}; }}")
        self.end_btn.clicked.connect(self._on_end_clicked)
        nrl.addWidget(self.end_btn)
        root.addWidget(nav_row)

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

        self.stack.addWidget(self.overview_panel)  # 0 Overview
        self.stack.addWidget(self.eclss_panel)     # 1 ECLSS (incl. OGS + WRS)
        self.stack.addWidget(self.gnc_panel)        # 2 GNC
        self.stack.addWidget(self.eps_panel)        # 3 EPS
        self.stack.addWidget(self.thermal_panel)    # 4 Thermal
        self.stack.addWidget(self.comms_panel)      # 5 Comms

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

    # ---------------- Parameter editor ----------------
    def _on_params_clicked(self):
        dlg = ParameterEditorDialog(self.node, self.executor, self)
        dlg.exec_()

    # ---------------- Fault injection ----------------
    def _on_fault_clicked(self):
        dlg = FaultInjectorDialog(self.node, self.executor, self)
        dlg.exec_()

    # ---------------- Session end ----------------
    def _on_end_clicked(self):
        """Close the window -> closeEvent stops ROS + plays the exit splash ->
        the GUI process exits -> launch on_exit=Shutdown terminates every other
        ROS node, so nothing is left running."""
        self.close()

    # ---------------- Sim speed control ----------------
    def _on_sim_speed(self, speed):
        """Set the simulation_controller's time_scale parameter live."""
        if not hasattr(self, "_sim_param_client") or self._sim_param_client is None:
            self._sim_param_client = self.node.create_client(
                SetParameters, "/simulation_controller/set_parameters")
        if not self._sim_param_client.service_is_ready():
            # Sim not up yet; note it and skip (the chip still reflects intent).
            self.event_feed.add_event(
                "SIM", f"speed {speed:g}x pending — simulation_controller not ready",
                "warning")
            return
        req = SetParameters.Request()
        req.parameters = [RclParameter(
            name="time_scale",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(speed)))]
        self._sim_param_client.call_async(req)
        self.event_feed.add_event("SIM", f"sim time scale set to {speed:g}x", "info")

    # ---------------- Global telemetry subscriptions ----------------
    def _init_global_subs(self):
        # Simulation clock, subscribed regardless of the ssos interfaces so the
        # Sim Clock still runs in a minimal (sim-only) bring-up.
        if _HAVE_CLOCK:
            try:
                self.node.create_subscription(Clock, "/clock", self._on_clock, 10)
            except Exception as e:
                self.node.get_logger().warn(f"[shell] /clock sub failed: {e}")
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

    def _on_clock(self, msg):
        sim_secs = msg.clock.sec + msg.clock.nanosec * 1e-9
        QTimer.singleShot(0, lambda: self.status_bar.set_sim_clock(sim_secs))

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
        name = (msg.subsystem_name or "").lower()
        row = _SUBSYS_ALIASES.get(name, None)
        if row is None:
            return
        lifecycle = _LIFECYCLE_NAMES.get(int(msg.lifecycle_state), "UNKNOWN")
        healthy = bool(msg.healthy)
        QTimer.singleShot(0, lambda: self._apply_heartbeat(name, row, lifecycle, healthy))

    def _apply_heartbeat(self, name, row, lifecycle, healthy):
        if name in _ECLSS_NODES:
            # Per-node lifecycle/transition state for the overview.
            self._eclss_states[name] = (lifecycle, healthy)
            self.overview_panel.set_lifecycle(name, lifecycle, healthy)
            # Aggregate the single ECLSS sidebar-roster row.
            states = self._eclss_states
            if any(not h for (_, h) in states.values()):
                self.roster.set_status("ECLSS", "fault")
            elif len(states) >= len(_ECLSS_NODES) and all(
                    lc == "ACTIVE" for (lc, _) in states.values()):
                self.roster.set_status("ECLSS", "active")
            else:
                self.roster.set_status("ECLSS", "degraded")
        else:
            self.roster.set_status(row, "nominal" if healthy else "fault")

    # ---------------- Videos ----------------
    def _play_video(self, filename):
        """Play a bundled video (blocking) via the OpenCV VideoPlayer.
        Crash-proof: any failure just returns so the app still proceeds."""
        try:
            if _USE_NEW_RESOURCES_API:
                res = files("space_station.assets") / filename
                with as_file(res) as path:
                    VideoPlayer(str(path)).play()
            else:
                with pkg_resources.path("space_station.assets", filename) as path:
                    VideoPlayer(str(path)).play()
        except Exception:
            pass

    def _play_startup_video(self):
        # Intro splash before the main window is shown.
        self._play_video("Ssos_begin.mp4")

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
        # Stop ROS first (so timers/spin halt), play the exit splash, then close.
        self._shutdown_ros()
        self._play_video("exit_vid.mp4")
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
