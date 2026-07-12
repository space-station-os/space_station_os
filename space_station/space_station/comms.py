# space_station/comms.py

import os, json, time
from typing import Optional

from PyQt5.QtWidgets import (
    QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton,
    QGroupBox, QListWidget, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, QDateTime, QUrl
from PyQt5.QtGui import QFont
from PyQt5.QtWebSockets import QWebSocket
import os
from ament_index_python.packages import get_package_share_directory

from space_station import theme
from space_station.widgets import page_header


def led(on: bool, color_ok=None, color_off=None):
    if color_ok is None:
        color_ok = theme.color("green")
    if color_off is None:
        color_off = theme.color("txt3")
    c = color_ok if on else color_off
    return f"border-radius:6px; background:{c}; min-width:12px; max-width:12px; min-height:12px; max-height:12px;"

class CommsWidget(QWidget):
    """
    Compact comms panel:
      - Subscribed topics (from YAML)
      - Starlink relay reachability (ping ws://localhost:8080)
      - Downlink receiving (data arriving from ws://localhost:9090)
      - Uplink OK proxy (recent TM seen)
    """
    def __init__(self, ros_node=None, parent=None,
                 forward_ws="ws://localhost:9090",
                 starlink_ws="ws://localhost:8080",
                 mapping_path: Optional[str] = None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)

        self.node = ros_node  # optional (not used here, but consistent with the rest of your GUI)
        self.forward_url = forward_ws
        self.starlink_url = starlink_ws
        pkg_dir = get_package_share_directory('space_station_communication')
        apid_config_path = os.path.join(pkg_dir, 'config', 'bridge.yaml')
        self.mapping_path = apid_config_path
        # WS clients (no threads)
        self.ws_forward = QWebSocket()   # listens to ground forwarder (9090)
        self.ws_starlink = QWebSocket()  # pings Starlink relay (8080)

        # State
        self.forward_connected = False
        self.starlink_connected = False
        self.last_tm_time = 0.0  # epoch of last TM from forwarder

        self._build_ui()
        self._wire()
        self._load_yaml()

        # connect immediately
        self.ws_forward.open(QUrl(self.forward_url))
        self.ws_starlink.open(QUrl(self.starlink_url))

        # timers
        self.stat_timer = QTimer(self)
        self.stat_timer.timeout.connect(self._tick_update_labels)
        self.stat_timer.start(500)

        self.starlink_ping = QTimer(self)
        self.starlink_ping.timeout.connect(self._send_starlink_ping)
        self.starlink_ping.start(2000)

    # ---------- UI ----------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Header
        header = page_header("Communications",
                             "Starlink Relay · Downlink / Uplink", "COMMS")
        root.addWidget(header)

        # Connection badge row
        title_row = QHBoxLayout()
        title_row.addStretch()
        self.badge = QLabel("DISCONNECTED")
        self._set_badge_style(False)
        title_row.addWidget(self.badge)
        root.addLayout(title_row)

        # Status box
        box = QGroupBox("Link Status")
        bl = QVBoxLayout(); bl.setSpacing(4)

        # LEDs row
        leds = QHBoxLayout()
        leds.setSpacing(12)
        self.led_starlink = QLabel(); self.led_starlink.setStyleSheet(led(False))
        self.led_down = QLabel();     self.led_down.setStyleSheet(led(False))
        self.led_up = QLabel();       self.led_up.setStyleSheet(led(False))

        leds.addWidget(QLabel("Starlink")); leds.addWidget(self.led_starlink)
        leds.addSpacing(12)
        leds.addWidget(QLabel("Downlink")); leds.addWidget(self.led_down)
        leds.addSpacing(12)
        leds.addWidget(QLabel("Uplink")); leds.addWidget(self.led_up)
        leds.addStretch()

        bl.addLayout(leds)

        self.lbl_starlink = QLabel(f"Relay: {self.starlink_url} — probing…")
        self.lbl_down = QLabel(f"Forwarder: {self.forward_url} — connecting…")
        self.lbl_last = QLabel("Last packet: –")
        for x in (self.lbl_starlink, self.lbl_down, self.lbl_last):
            bl.addWidget(x)

        # Small control row
        ctr = QHBoxLayout()
        self.btn_reconnect = QPushButton("Reconnect")
        self.btn_reload = QPushButton("Reload YAML")
        ctr.addWidget(self.btn_reconnect); ctr.addWidget(self.btn_reload); ctr.addStretch()
        bl.addLayout(ctr)

        box.setLayout(bl)
        root.addWidget(box)

        # Subscribed topics
        tm_box = QGroupBox("Subscribed Telemetry Topics")
        tml = QVBoxLayout()
        self.tm_list = QListWidget()
        self.tm_list.setMaximumHeight(140)
        tml.addWidget(self.tm_list)
        tm_box.setLayout(tml)
        root.addWidget(tm_box)

        root.addStretch()
        self.setLayout(root)

    # ---------- Signals ----------
    def _wire(self):
        # forwarder
        self.ws_forward.connected.connect(self._on_forward_connected)
        self.ws_forward.disconnected.connect(self._on_forward_disconnected)
        self.ws_forward.textMessageReceived.connect(self._on_forward_text)
        self.ws_forward.error.connect(self._on_forward_error)

        # starlink relay
        self.ws_starlink.connected.connect(self._on_starlink_connected)
        self.ws_starlink.disconnected.connect(self._on_starlink_disconnected)
        self.ws_starlink.pong.connect(self._on_starlink_pong)  # Qt emits pong(elapsed, payload)
        self.ws_starlink.error.connect(self._on_starlink_error)

        # buttons
        self.btn_reconnect.clicked.connect(self._reconnect_all)
        self.btn_reload.clicked.connect(self._load_yaml)

    # ---------- YAML ----------
    def _load_yaml(self):
        self.tm_list.clear()
        path = self.mapping_path
        if not path or not os.path.isfile(path):
            self.tm_list.addItem("(mapping not set)")
            return
        try:
            import yaml
            with open(path, "r") as f:
                entries = yaml.safe_load(f) or []
            topics = [e["ros_topic_name"] for e in entries if e.get("communication_type") == "TM"]
            for t in topics:
                self.tm_list.addItem(t)
        except Exception as e:
            self.tm_list.addItem(f"(YAML error: {e})")

    # ---------- Forwarder (9090) handlers ----------
    def _on_forward_connected(self):
        self.forward_connected = True
        self.lbl_down.setText(f"Forwarder: {self.forward_url} — connected")
        self._update_badge()

    def _on_forward_disconnected(self):
        self.forward_connected = False
        self.lbl_down.setText(f"Forwarder: {self.forward_url} — disconnected")
        self._update_badge()
        # try to reconnect after a short pause
        QTimer.singleShot(1500, lambda: self.ws_forward.open(QUrl(self.forward_url)))

    def _on_forward_error(self, _):
        self.forward_connected = False
        self._update_badge()

    def _on_forward_text(self, text: str):
        # Expect {"topic": "...", "value": ..., "timestamp": ...}
        self.last_tm_time = time.time()
        self.lbl_last.setText(f"Last packet: just now")
        self.led_down.setStyleSheet(led(True))
        self.led_up.setStyleSheet(led(True))  # proxy: if we see TM here, uplink path is alive
        self._update_badge()
        # keep this tiny: we don't parse the payload further

    # ---------- Starlink (8080) handlers ----------
    def _on_starlink_connected(self):
        self.starlink_connected = True
        self.lbl_starlink.setText(f"Relay: {self.starlink_url} — connected")
        self._update_badge()

    def _on_starlink_disconnected(self):
        self.starlink_connected = False
        self.lbl_starlink.setText(f"Relay: {self.starlink_url} — disconnected")
        self.led_starlink.setStyleSheet(led(False))
        self._update_badge()
        QTimer.singleShot(2000, lambda: self.ws_starlink.open(QUrl(self.starlink_url)))

    def _on_starlink_error(self, _):
        self.starlink_connected = False
        self._update_badge()

    def _send_starlink_ping(self):
        if self.starlink_connected:
            try:
                self.ws_starlink.ping(b"ssos")
            except Exception:
                pass

    def _on_starlink_pong(self, elapsed_ms: int, _payload: bytes):
        # If we got a pong, the relay server is alive
        self.led_starlink.setStyleSheet(led(True))
        self.lbl_starlink.setText(f"Relay: {self.starlink_url} — pong {elapsed_ms} ms")

    # ---------- UI helpers ----------
    def _tick_update_labels(self):
        # last packet "ago"
        if self.last_tm_time > 0:
            ago = int(time.time() - self.last_tm_time)
            self.lbl_last.setText(f"Last packet: {ago}s ago" if ago else "Last packet: just now")
            # If it’s been quiet for >5s, dim uplink/downlink
            self.led_down.setStyleSheet(led(ago <= 5))
            self.led_up.setStyleSheet(led(ago <= 5))
        self._update_badge()

    def _set_badge_style(self, connected: bool):
        if connected:
            fg = theme.color("green")
        else:
            fg = theme.color("red")
        self.badge.setStyleSheet(
            f"color:{fg}; background:{theme.color('panel2')}; "
            f"border:1px solid {fg}; padding:2px 6px; border-radius:6px;"
        )

    def _update_badge(self):
        ok = self.forward_connected
        if ok:
            self.badge.setText("OK")
            self._set_badge_style(True)
        else:
            self.badge.setText("DISCONNECTED")
            self._set_badge_style(False)

    def _reconnect_all(self):
        try: self.ws_forward.close()
        except: pass
        try: self.ws_starlink.close()
        except: pass
        QTimer.singleShot(300, lambda: self.ws_forward.open(QUrl(self.forward_url)))
        QTimer.singleShot(300, lambda: self.ws_starlink.open(QUrl(self.starlink_url)))
