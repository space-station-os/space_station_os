# space_station/ai_agent.py

import os
import threading
import json
from typing import Dict, Any, Optional, Tuple
from PyQt6.QtCore import QObject, pyqtSignal
import yaml
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, String, Bool
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import Temperature, BatteryState
from dotenv import load_dotenv
from openai import OpenAI
from ament_index_python.packages import get_package_share_directory

# Try importing thermal messages
_THERMAL_MSGS_OK = True
try:
    from space_station_thermal_control.msg import (
        ThermalNodeDataArray,
        ThermalLinkFlowsArray,
        ExternalLoopStatus,
        TankStatus,
    )
except Exception:
    _THERMAL_MSGS_OK = False
    ThermalNodeDataArray = None
    ThermalLinkFlowsArray = None
    ExternalLoopStatus = None
    TankStatus = None

load_dotenv(override=True)


class SsosAIAgent(QObject):
    """
    Reads ROS 2 telemetry via shared GUI node (ECLSS + Thermal + EPS),
    and answers astronaut questions using gpt-oss-20b (via NVIDIA integrate API).
    """
    ai_reply = pyqtSignal(str)

    def __init__(
        self,
        ros_node,
        parent=None,
        base_url: Optional[str] = None,
        model: Optional[str] = None,
        api_key: Optional[str] = None,
        request_timeout_s: float = 10.0,
    ):
        super().__init__(parent)
        self.node = ros_node

        # Config
        self.base_url = base_url or os.environ.get(
            "SSOS_LLM_BASE_URL", "https://integrate.api.nvidia.com/v1"
        )
        self.model = model or os.environ.get("SSOS_LLM_MODEL", "openai/gpt-oss-20b")
        self.api_key = api_key or os.getenv("NVIDIA_API_KEY")
        self.request_timeout_s = request_timeout_s
        if not self.api_key:
            raise RuntimeError("Missing NVIDIA_API_KEY. Set it in env or pass api_key=...")

        # OpenAI client (NVIDIA integrate API)
        self.client = OpenAI(base_url=self.base_url, api_key=self.api_key)

        # Telemetry cache
        self._lock = threading.Lock()
        self._cache: Dict[str, Any] = {
            "co2_mmHg": None,
            "o2_percent": None,
            "temp_c": None,
            "water_l": None,
            "diag": {"level": 0, "name": "", "message": ""},
            "thermal": {
                "internal_loop_c": None,
                "external_loop": {
                    "received_heat_kj": None,
                    "loop_inlet_temp_c": None,
                    "loop_outlet_temp_c": None,
                    "ammonia_inlet_temp_c": None,
                    "ammonia_outlet_temp_c": None,
                },
                "nodes": {"count": 0, "avg_c": None, "max_c": None, "hottest": [], "list": []},
                "links": {"count": 0, "total_abs_heat_flow": None, "top": []},
                "diag": {"level": 0, "name": "", "message": ""},
                "ammonia": {
                    "tank_capacity_l": None,
                    "tank_temperature_c": None,
                    "tank_pressure_pa": None,
                    "tank_heater_on": None,
                },
            },
            "comms": {"uplink_ok": False, "downlink_ok": False, "tm_topics": []},
            "eps": {
                "batteries": {},   # id -> dict
                "bcdu": {"mode": "", "voltage": 0.0},
                "mbsu": {"channels": {}},   # id -> voltage
                "ddcu": {"input_v": 0.0, "output_v": 0.0, "temp_c": 0.0},
            },
        }

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ---------------- ECLSS subscriptions ----------------
        self.node.create_subscription(Float64, "/co2_storage", self._on_co2, qos)
        self.node.create_subscription(Float64, "/o2_storage", self._on_o2, qos)
        self.node.create_subscription(Float64, "/cabin_temperature_c", self._on_temp, qos)
        self.node.create_subscription(Float64, "/wrs/product_water_reserve", self._on_water, qos)
        self.node.create_subscription(DiagnosticStatus, "/system/diagnostics", self._on_diag, qos)

        # ---------------- Thermal subscriptions ----------------
        
        if _THERMAL_MSGS_OK:
            self.node.create_subscription(
                ExternalLoopStatus, "/tcs/external_loop_a/status", self._on_external_loop_status, qos
            )
            self.node.create_subscription(
                ThermalNodeDataArray, "/thermal/nodes/state", self._on_nodes_state, qos
            )
            self.node.create_subscription(
                ThermalLinkFlowsArray, "/thermal/links/flux", self._on_links_flux, qos
            )
        self.node.create_subscription(DiagnosticStatus, "/thermals/diagnostics", self._on_thermal_diag, qos)
        try:
            self.node.create_subscription(TankStatus, "/tcs/ammonia_status", self._on_ammonia_status, qos)
        except Exception:
            pass

        # ---------------- Comms subscriptions ----------------
        self.node.create_subscription(Bool, "/comms/uplink", self._on_uplink, qos)
        self.node.create_subscription(Bool, "/comms/downlink", self._on_downlink, qos)
        try:
            pkg_dir = get_package_share_directory("space_station_communication")
            apid_config_path = os.path.join(pkg_dir, "config", "bridge.yaml")
            with open(apid_config_path, "r") as f:
                raw = yaml.safe_load(f)
                topics = [entry["ros_topic_name"] for entry in raw if entry["communication_type"] == "TM"]
                with self._lock:
                    self._cache["comms"]["tm_topics"] = topics
        except Exception as e:
            self.node.get_logger().warn(f"Failed to load comms topics: {e}")

        # ---------------- EPS subscriptions ----------------
        # 24 batteries
        for i in range(24):
            topic = f"/battery/battery_bms_{i}/health"
            self.node.create_subscription(BatteryState, topic, lambda m, idx=i: self._on_battery(idx, m), qos)

        # BCDU
        self.node.create_subscription(String, "/bcdu/status", self._on_bcdu, qos)

        # MBSU
        for i in range(12):
            topic = f"/mbsu/channel_{i}/voltage"
            self.node.create_subscription(Float64, topic, lambda m, idx=i: self._on_mbsu(idx, m), qos)

        # DDCU
        self.node.create_subscription(Float64, "/ddcu/input_voltage", self._on_ddcu_in, qos)
        self.node.create_subscription(Float64, "/ddcu/output_voltage", self._on_ddcu_out, qos)
        self.node.create_subscription(Float64, "/ddcu/temperature", self._on_ddcu_temp, qos)

    # ---------------- ECLSS callbacks ----------------
    def _on_co2(self, msg: Float64):
        with self._lock:
            self._cache["co2_mmHg"] = float(msg.data)

    def _on_o2(self, msg: Float64):
        with self._lock:
            self._cache["o2_percent"] = float(msg.data)

    def _on_temp(self, msg: Float64):
        with self._lock:
            self._cache["temp_c"] = float(msg.data)

    def _on_water(self, msg: Float64):
        with self._lock:
            self._cache["water_l"] = float(msg.data)

    def _on_diag(self, msg: DiagnosticStatus):
        with self._lock:
            self._cache["diag"] = {"level": int(msg.level), "name": msg.name, "message": msg.message}

    # ---------------- Thermal callbacks ----------------
    def _on_internal_loop_heat(self, msg: Temperature):
        with self._lock:
            self._cache["thermal"]["internal_loop_c"] = float(msg.temperature)

    def _on_ammonia_status(self, msg: TankStatus):
        with self._lock:
            self._cache["thermal"]["ammonia"] = {
                "tank_capacity_l": float(msg.tank_capacity),
                "tank_temperature_c": float(msg.tank_temperature.temperature),
                "tank_pressure_pa": float(msg.tank_pressure.fluid_pressure),
                "tank_heater_on": bool(msg.tank_heater_on),
            }

    def _on_external_loop_status(self, msg: Any):
        with self._lock:
            self._cache["thermal"]["external_loop"] = {
                "received_heat_kj": float(getattr(msg, "received_heat_kj", 0.0)),
                "loop_inlet_temp_c": float(getattr(msg, "loop_inlet_temp", 0.0)),
                "loop_outlet_temp_c": float(getattr(msg, "loop_outlet_temp", 0.0)),
                "ammonia_inlet_temp_c": float(getattr(msg, "ammonia_inlet_temp", 0.0)),
                "ammonia_outlet_temp_c": float(getattr(msg, "ammonia_outlet_temp", 0.0)),
            }

    def _on_nodes_state(self, msg: Any):
        try:
            nodes = list(msg.nodes)
        except Exception:
            return
        count = len(nodes)
        if count == 0:
            summary = {"count": 0, "avg_c": 0.0, "max_c": 0.0, "hottest": [], "list": []}
        else:
            temps = [float(n.temperature) for n in nodes]
            caps = [float(n.heat_capacity) for n in nodes]
            names = [str(n.name) for n in nodes]
            avg_c = sum(temps) / count
            max_idx = max(range(count), key=lambda i: temps[i])
            hottest = [{"name": names[i], "c": temps[i], "cap": caps[i]} for i in sorted(range(count), key=lambda i: temps[i], reverse=True)[:3]]
            full_list = [{"name": names[i], "c": temps[i], "cap": caps[i]} for i in range(count)]
            summary = {"count": count, "avg_c": avg_c, "max_c": temps[max_idx], "hottest": hottest, "list": full_list}
        with self._lock:
            self._cache["thermal"]["nodes"] = summary

    def _on_links_flux(self, msg: Any):
        try:
            links = msg.links
        except Exception:
            return
        count = len(links)
        if count == 0:
            summary = {"count": 0, "total_abs_heat_flow": 0.0, "top": []}
        else:
            flows = [float(l.heat_flow) for l in links]
            conds = [float(l.conductance) for l in links]
            a = [str(l.node_a) for l in links]
            b = [str(l.node_b) for l in links]
            total_abs = sum(abs(f) for f in flows)
            order = sorted(range(count), key=lambda i: abs(flows[i]), reverse=True)[:3]
            top = [{"a": a[i], "b": b[i], "flow": flows[i], "cond": conds[i]} for i in order]
            summary = {"count": count, "total_abs_heat_flow": total_abs, "top": top}
        with self._lock:
            self._cache["thermal"]["links"] = summary

    def _on_thermal_diag(self, msg: DiagnosticStatus):
        diag = {"level": int(msg.level), "name": msg.name, "message": msg.message}
        with self._lock:
            self._cache["thermal"]["diag"] = diag

    # ---------------- Comms callbacks ----------------
    def _on_uplink(self, msg: Bool):
        with self._lock:
            self._cache["comms"]["uplink_ok"] = msg.data

    def _on_downlink(self, msg: Bool):
        with self._lock:
            self._cache["comms"]["downlink_ok"] = msg.data

    # ---------------- EPS callbacks ----------------
    def _on_battery(self, idx: int, msg: BatteryState):
        status = "idle"
        if msg.current > 0.1:
            status = "charging"
        elif msg.current < -0.1:
            status = "discharging"
        if msg.voltage < 70.0:
            status = "fault"
        with self._lock:
            self._cache["eps"]["batteries"][idx] = {
                "voltage": float(msg.voltage),
                "current": float(msg.current),
                "power": float(msg.voltage * msg.current),
                "status": status,
            }

    def _on_bcdu(self, msg: String):
        with self._lock:
            self._cache["eps"]["bcdu"]["mode"] = msg.data

    def _on_mbsu(self, idx: int, msg: Float64):
        with self._lock:
            self._cache["eps"]["mbsu"]["channels"][idx] = float(msg.data)

    def _on_ddcu_in(self, msg: Float64):
        with self._lock:
            self._cache["eps"]["ddcu"]["input_v"] = float(msg.data)

    def _on_ddcu_out(self, msg: Float64):
        with self._lock:
            self._cache["eps"]["ddcu"]["output_v"] = float(msg.data)

    def _on_ddcu_temp(self, msg: Float64):
        with self._lock:
            self._cache["eps"]["ddcu"]["temp_c"] = float(msg.data)

    # ---------------- Intent router ----------------
    def _intent_and_subset(self, question: str) -> Tuple[str, dict]:
        q = question.lower()
        full = self._snapshot()
        th = full.get("thermal", {})
        eps = full.get("eps", {})
        filtered = {}

        # EPS
        if "battery" in q or "bms" in q or "oru" in q:
            import re
            m = re.search(r"\d+", q)
            if m:
                idx = int(m.group())
                filtered["eps"] = {"battery": {idx: eps["batteries"].get(idx, {})}}
                return ("eps_battery", filtered)
        if "bcdu" in q:
            filtered["eps"] = {"bcdu": eps["bcdu"]}
            return ("eps_bcdu", filtered)
        if "mbsu" in q or "channel" in q:
            filtered["eps"] = {"mbsu": eps["mbsu"]}
            return ("eps_mbsu", filtered)
        if "ddcu" in q:
            filtered["eps"] = {"ddcu": eps["ddcu"]}
            return ("eps_ddcu", filtered)

        # Existing intents (thermal, comms, etc.) unchanged...
        if "uplink" in q or "downlink" in q or "comms" in q:
            filtered["comms"] = full["comms"]
            return ("comms", filtered)

        return ("direct", filtered)

    # ---------------- Ask interface ----------------
    def ask(self, question: str):
        t = threading.Thread(target=self._ask_worker, args=(question,), daemon=True)
        t.start()

    def _ask_worker(self, question: str):
        intent, subset = self._intent_and_subset(question)
        answer = ""

        try:
            if intent.startswith("eps"):
                if intent == "eps_battery":
                    for idx, b in subset["eps"]["battery"].items():
                        if not b:
                            answer = f"Battery {idx} data not available"
                        else:
                            answer = f"Battery {idx}: {b['voltage']:.1f} V, {b['current']:.1f} A, {b['status']}"
                elif intent == "eps_bcdu":
                    bcdu = subset["eps"]["bcdu"]
                    answer = f"BCDU: {bcdu.get('mode','unknown')} at {bcdu.get('voltage',0.0):.1f} V"
                elif intent == "eps_mbsu":
                    channels = subset["eps"]["mbsu"]["channels"]
                    active = [f"{k}: {v:.1f} V" for k, v in channels.items() if v > 0.1]
                    answer = "MBSU channels supplying: " + (", ".join(active) if active else "none")
                elif intent == "eps_ddcu":
                    ddcu = subset["eps"]["ddcu"]
                    answer = f"DDCU input {ddcu['input_v']:.1f} V → output {ddcu['output_v']:.1f} V at {ddcu['temp_c']:.1f} °C"
            elif intent == "comms":
                uplink = subset["comms"].get("uplink_ok", False)
                downlink = subset["comms"].get("downlink_ok", False)
                topics = subset["comms"].get("tm_topics", [])
                parts = [
                    f"Uplink: {'Connected' if uplink else 'Disconnected'}",
                    f"Downlink: {'Receiving' if downlink else 'Not Receiving'}",
                ]
                if topics:
                    parts.append("Streaming topics:\n" + "\n".join(f"• {t}" for t in topics))
                answer = "\n".join(parts)
            else:
                # Default path via LLM
                system_msg = (
                    "You are SSOS-AI, an operations copilot for Space Station OS. "
                    "Answer ONLY the astronaut's question using provided snapshot subset. "
                    "Do not invent values. Keep answers concise with exact units."
                )
                messages = [
                    {"role": "system", "content": system_msg},
                    {"role": "user", "content": f"Q: {question}\nData: {json.dumps(subset)}"},
                ]
                completion = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    temperature=0.1,
                    timeout=self.request_timeout_s,
                )
                answer = completion.choices[0].message.content.strip()
        except Exception as e:
            answer = f"AI unavailable. Error: {e}. Subset: {json.dumps(subset)}"

        self.ai_reply.emit(answer)

    # ---------------- Helpers ----------------
    def _snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._cache))
