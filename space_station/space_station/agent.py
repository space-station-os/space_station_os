# space_station/ai_agent.py

import os
import threading
import json
from typing import Dict, List, Any, Optional
from PyQt5.QtCore import QObject, pyqtSignal
import yaml 
from numpy import full
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, String
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import Temperature
from dotenv import load_dotenv
from openai import OpenAI  
from typing import Tuple
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

_THERMAL_MSGS_OK = True
try:
 
    from space_station_thermal_control.msg import ThermalNodeDataArray, ThermalLinkFlowsArray
    from space_station_thermal_control.msg import ExternalLoopStatus,InternalLoopStatus,TankStatus
except Exception:
    _THERMAL_MSGS_OK = False
    ThermalNodeDataArray = None
    ThermalLinkFlowsArray = None
    ExternalLoopStatus = None
    InternalLoopStatus = None




load_dotenv(override=True)
class SsosAIAgent(QObject):
    """
    Reads ROS 2 telemetry via shared GUI node (ECLSS + Thermal),
    and answers astronaut questions by calling gpt-oss-20b via NVIDIA's
    integrate API (OpenAI-compatible).
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
        
        # Resolve config (env overrides are supported)
        self.base_url = base_url or os.environ.get("SSOS_LLM_BASE_URL", "https://integrate.api.nvidia.com/v1")
        self.model    = model    or os.environ.get("SSOS_LLM_MODEL", "openai/gpt-oss-20b")
        self.api_key  = api_key  or os.getenv("NVIDIA_API_KEY")
        self.request_timeout_s = request_timeout_s

        if not self.api_key:
            raise RuntimeError("Missing NVIDIA_API_KEY. Set it in env or pass api_key=...")

        # OpenAI client pointed at NVIDIA integrate API
        self.client = OpenAI(base_url=self.base_url, api_key=self.api_key)

        # Telemetry cache
        self._lock = threading.Lock()
        self._cache: Dict[str, Any] = {
            # ECLSS snapshot
            "co2_mmHg": None,      # CO₂ in mmHg (or ppm converted upstream if desired)
            "o2_percent": None,    # O₂ reserve %
            "temp_c": None,        # Cabin temperature °C
            "water_l": None,       # Water reserve L
            "diag": {"level": 0, "name": "", "message": ""},

            # Thermal snapshot (compact + useful)
            "thermal": {
                "internal_loop_c": None,     # /tcs/internal_loop_heat (Temperature.temperature, °C)
                "external_loop": {           # ExternalLoopStatus
                    "received_heat_kj": None,
                    "loop_inlet_temp_c": None,
                    "loop_outlet_temp_c": None,
                    "ammonia_inlet_temp_c": None,
                    "ammonia_outlet_temp_c": None,
                },
                "nodes": {                   # ThermalNodeDataArray summary
                    "count": 0,
                    "avg_c": None,
                    "max_c": None,
                    "hottest": [],
                    "list": [] 
                },
                "links": {                   # ThermalLinkFlowsArray summary
                    "count": 0,
                    "total_abs_heat_flow": None,
                    "top": []               # top 3 by |heat_flow|: [{a,b,flow,cond}]
                },
                "diag": {"level": 0, "name": "", "message": ""},
                "ammonia": {
                    "tank_capacity_l": None,
                    "tank_temperature_c": None,
                    "tank_pressure_pa": None,
                    "tank_heater_on": None
                },
                "comms":{
                    "uplink_ok": False,
                    "downlink_ok": False,
                    "tm_topics": []
                }

            }
        }

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # ---------------- ECLSS subscriptions (your existing topics) ----------------
        self.sub_co2 = self.node.create_subscription(Float64, "/co2_storage", self._on_co2, qos)
        self.sub_o2  = self.node.create_subscription(Float64, "/o2_storage", self._on_o2, qos)
        self.sub_tmp = self.node.create_subscription(Float64, "/cabin_temperature_c", self._on_temp, qos)
        try:
            self.sub_h2o = self.node.create_subscription(Float64, "/wrs/product_water_reserve", self._on_water, qos)
        except Exception:
            self.sub_h2o = None

        # General system diagnostics (kept)
        self.sub_diag = self.node.create_subscription(DiagnosticStatus, "/system/diagnostics", self._on_diag, qos)

        # ---------------- Thermal subscriptions ----------------
        # 1) Internal loop heat (Temperature)
        try:
            self.sub_int_heat = self.node.create_subscription(
                InternalLoopStatus, "/tcs/internal_loop_heat", self._on_internal_loop_heat, qos
            )
        except Exception:
            self.sub_int_heat = None

        # 2) External loop A status (ExternalLoopStatus custom)
        if _THERMAL_MSGS_OK:
            try:
                self.sub_ext_status = self.node.create_subscription(
                    ExternalLoopStatus, "/tcs/external_loop_a/status", self._on_external_loop_status, qos
                )
            except Exception:
                self.sub_ext_status = None
        else:
            self.sub_ext_status = None

        # 3) Thermal nodes state (ThermalNodeDataArray)
        if _THERMAL_MSGS_OK:
            try:
                self.sub_nodes = self.node.create_subscription(
                    ThermalNodeDataArray, "/thermal/nodes/state", self._on_nodes_state, qos
                )
            except Exception:
                self.sub_nodes = None
        else:
            self.sub_nodes = None

        # 4) Thermal links flux (ThermalLinkFlowsArray)
        if _THERMAL_MSGS_OK:
            try:
                self.sub_links = self.node.create_subscription(
                    ThermalLinkFlowsArray, "/thermal/links/flux", self._on_links_flux, qos
                )
            except Exception:
                self.sub_links = None
        else:
            self.sub_links = None

        # 5) Thermal diagnostics (DiagnosticStatus)
        try:
            self.sub_th_diag = self.node.create_subscription(
                DiagnosticStatus, "/thermals/diagnostics", self._on_thermal_diag, qos
            )
        except Exception:
            self.sub_th_diag = None

        # 6) Optional extras (ignored in snapshot but here if you want):
        #    /solar_controller/commands (String) and /tcs/ammonia_status (type unknown)
        try:
            self.sub_solar_cmd = self.node.create_subscription(
                String, "/solar_controller/commands", lambda _m: None, qos
            )
        except Exception:
            self.sub_solar_cmd = None
        try:
            self.sub_ammonia = self.node.create_subscription(
                TankStatus,
                "/tcs/ammonia_status",
                self._on_ammonia_status,
                qos
            )
        except Exception:
            self.sub_ammonia = None

        self.sub_uplink = self.node.create_subscription(Bool, "/comms/uplink", self._on_uplink, qos)
        self.sub_downlink = self.node.create_subscription(Bool, "/comms/downlink", self._on_downlink, qos)

        
        
        try:
            pkg_dir = get_package_share_directory('space_station_communication')
            apid_config_path = os.path.join(pkg_dir, 'config', 'bridge.yaml')
            topics = []

            with open(apid_config_path, 'r') as file:
                raw = yaml.safe_load(file)
                topics = [entry['ros_topic_name'] for entry in raw if entry['communication_type'] == 'TM']

            with self._lock:
                self._cache["comms"]["tm_topics"] = topics

        except Exception as e:
            self.node.get_logger().warn(f"Failed to load comms topics from bridge.yaml: {e}")



    
        
    # ----- ECLSS callbacks -----
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

    # ----- Thermal callbacks -----
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
        # msg.nodes[] with fields: name, temperature, heat_capacity, internal_power
        try:
            nodes = list(msg.nodes)  # type: ignore[attr-defined]
        except Exception:
            return

        count = len(nodes)
        if count == 0:
            summary = {"count": 0, "avg_c": 0.0, "max_c": 0.0, "hottest": [], "list": []}
        else:
            temps = [float(n.temperature) for n in nodes]
            caps  = [float(n.heat_capacity) for n in nodes]
            names = [str(n.name) for n in nodes]

            avg_c = sum(temps) / count
            max_idx = max(range(count), key=lambda i: temps[i])
            max_c = temps[max_idx]

            # top 3 hottest
            order = sorted(range(count), key=lambda i: temps[i], reverse=True)[:3]
            hottest = [{"name": names[i], "c": temps[i], "cap": caps[i]} for i in order]

            # full list (all nodes)
            full_list = [{"name": names[i], "c": temps[i], "cap": caps[i]} for i in range(count)]

            summary = {"count": count, "avg_c": avg_c, "max_c": max_c, "hottest": hottest, "list": full_list}

        with self._lock:
            self._cache["thermal"]["nodes"] = summary


    def _on_links_flux(self, msg: Any):
        # msg.links[] with fields: node_a, node_b, conductance, heat_flow
        try:
            links = msg.links  # type: ignore[attr-defined]
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
            top = [
                {"a": a[i], "b": b[i], "flow": flows[i], "cond": conds[i]}
                for i in order
            ]
            summary = {"count": count, "total_abs_heat_flow": total_abs, "top": top}

        with self._lock:
            self._cache["thermal"]["links"] = summary

    def _on_thermal_diag(self, msg: DiagnosticStatus):
        diag = {
            "level": int(msg.level),
            "name": msg.name,
            "message": msg.message
        }
        with self._lock:
            current = self._cache["thermal"].get("extra_diagnostics", [])
            updated = [d for d in current if d["name"] != msg.name] + [diag]
            self._cache["thermal"]["extra_diagnostics"] = updated
            self._cache["thermal"]["diag"] = diag  
            
    def _on_uplink(self, msg: Bool):
        with self._lock:
            self._cache["comms"]["uplink_ok"] = msg.data

    def _on_downlink(self, msg: Bool):
        with self._lock:
            self._cache["comms"]["downlink_ok"] = msg.data

    
    def _intent_and_subset(self, question: str) -> Tuple[str, dict]:
        """
        Very lightweight intent router: decide which parts of the snapshot to expose
        based on keywords in the astronaut's question. Returns (intent, filtered_snapshot).
        """
        q = question.lower()

        # Buckets you can extend
        want_water   = any(k in q for k in ["water", "h2o", "wrs", "hydration"])
        want_o2      = any(k in q for k in ["o2", "oxygen", "ogs"])
        want_co2     = any(k in q for k in ["co2", "carbon dioxide", "ars", "scrubber"])
        want_temp    = any(k in q for k in ["cabin temp", "temperature", "cabin"])
        want_diag    = any(k in q for k in ["diag", "failure", "status", "warning", "crit"])
        want_nodes   = any(k in q for k in ["thermal node", "nodes", "hottest"])
        want_list    = any(k in q for k in ["list", "all", "show all"])
        want_links   = any(k in q for k in ["heat flow", "links", "conductance"])
        want_internal= any(k in q for k in ["internal loop", "internal heat"])
        want_external = any(k in q for k in [
            "external loop", "ammonia", "radiator", "received heat", "inlet",
            "outlet", "vent", "tank", "heater", "pressure"
        ])
        want_comms = any(k in q for k in ["uplink", "downlink", "comms", "relay", "starlink"])



        # Default: if the question is generic like "system status", prefer ECLSS summary + diag
        generic_status = ("status" in q or "overall" in q or "system" in q) and not any(
            [want_water, want_o2, want_co2, want_nodes, want_links, want_internal, want_external]
        )

        full = self._snapshot()
        th = full.get("thermal", {})

        filtered = {}
        if generic_status:
            filtered.update({
                "co2_mmHg": full["co2_mmHg"],
                "o2_percent": full["o2_percent"],
                "water_l": full["water_l"],
                "temp_c": full["temp_c"],
            })
            return ("status", filtered)
        
        if want_nodes or want_list:
            filtered.setdefault("thermal", {})
            nodes_block = th.get("nodes", {})
            if want_list:
            
                filtered["thermal"]["nodes"] = {"list": nodes_block.get("list", []), "count": nodes_block.get("count", 0)}
            else:
               
                filtered["thermal"]["nodes"] = {"hottest": nodes_block.get("hottest", []), "count": nodes_block.get("count", 0)}

        # Only add what was asked
        if want_water:   filtered["water_l"] = full["water_l"]
        if want_o2:      filtered["o2_percent"] = full["o2_percent"]
        if want_co2:     filtered["co2_mmHg"] = full["co2_mmHg"]
        if want_temp:    filtered["temp_c"] = full["temp_c"]
        if want_diag:    filtered["diag"] = full["diag"]

        if any([want_nodes, want_links, want_internal, want_external]):
            filtered["thermal"] = {}
            if want_internal: filtered["thermal"]["internal_loop_c"] = th.get("internal_loop_c", 0.0)
            if want_external:
                filtered["thermal"]["external_loop"] = th.get("external_loop", {})
                filtered["thermal"]["ammonia"] = th.get("ammonia", {})
                filtered["thermal"]["extra_diagnostics"] = th.get("extra_diagnostics", [])
            if want_nodes:    filtered["thermal"]["nodes"] = th.get("nodes", {})
            if want_links:    filtered["thermal"]["links"] = th.get("links", {})

        if want_comms:
            filtered["comms"] = self._cache.get("comms", {})
            return ("comms", filtered)

        return ("direct", filtered)
    # ----- Public API called by LeftPanel -----
    def ask(self, question: str):
        t = threading.Thread(target=self._ask_worker, args=(question,), daemon=True)
        t.start()

    def _ask_worker(self, question: str):
        intent, subset = self._intent_and_subset(question)

        
        system_msg = (
            "You are SSOS-AI, an operations copilot for Space Station OS. "
            "Answer ONLY the astronaut's question. Do not add extra sections or unrelated data. "
            "Use the provided snapshot subset; do not invent values. "
            "If the needed value is missing, say so briefly and ask a single follow-up question. "
            "Keep answers concise and include exact units."
        )

        messages = [
            {"role": "system", "content": system_msg},
            {
                "role": "user",
                "content": (
                    f"Astronaut question: {question}\n"
                    f"Snapshot subset: {json.dumps(subset, separators=(',', ':'))}"
                ),
            },
        ]

        try:
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.1,
                timeout=self.request_timeout_s,
            )
            answer = completion.choices[0].message.content.strip()
        except Exception:
            answer = f"AI unavailable. Snapshot subset: {json.dumps(subset)}"
            
            
        if intent == "comms":
            uplink = subset["comms"].get("uplink_ok", False)
            downlink = subset["comms"].get("downlink_ok", False)
            topics = subset["comms"].get("tm_topics", [])

            parts = []
            parts.append(f"Uplink: {' Connected' if uplink else ' Disconnected'}")
            parts.append(f"Downlink: {' Receiving' if downlink else ' Not Receiving'}")
            if topics:
                parts.append("Streaming topics:\n" + "\n".join(f"• {t}" for t in topics))
            else:
                parts.append("No TM topics listed")

            answer = "\n".join(parts)


        # Emit as minimal HTML (bold key value pairs render nicely in QTextEdit)
        self.ai_reply.emit(answer) 

    def _to_html(self, text: str) -> str:
        """Very small sanitizer: convert double-asterisk to bold; preserve newlines."""
        
        html = (
            text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                .replace("**", "<b>").replace("<b>", "</b>", 1)  
                .replace("\n", "<br>")
        )
       
        if "<" not in html or ">" not in html:
            html = f"<span>{html}</span>"
        return html
    # ----- Helpers -----
    def _snapshot(self) -> Dict[str, Any]:
        with self._lock:
            snap = json.loads(json.dumps(self._cache)) 

        snap["co2_mmHg"]   = 0.0 if snap["co2_mmHg"]   is None else snap["co2_mmHg"]
        snap["o2_percent"] = 0.0 if snap["o2_percent"] is None else snap["o2_percent"]
        snap["temp_c"]     = 0.0 if snap["temp_c"]     is None else snap["temp_c"]
        snap["water_l"]    = 0.0 if snap["water_l"]    is None else snap["water_l"]

        th = snap["thermal"]
        if th["internal_loop_c"] is None: th["internal_loop_c"] = 0.0
        ext = th["external_loop"]
        for k in list(ext.keys()):
            if ext[k] is None: ext[k] = 0.0

        nodes = th["nodes"]
        if nodes.get("avg_c") is None: nodes["avg_c"] = 0.0
        if nodes.get("max_c") is None: nodes["max_c"] = 0.0
        if nodes.get("count") is None: nodes["count"] = 0
        if nodes.get("hottest") is None: nodes["hottest"] = []
        if nodes.get("list") is None: nodes["list"] = []  
        links = th["links"]
        if links.get("count") is None: links["count"] = 0
        if links.get("total_abs_heat_flow") is None: links["total_abs_heat_flow"] = 0.0
        if links.get("top") is None: links["top"] = []

        am = th.get("ammonia", {})
        for key in ["tank_capacity_l", "tank_temperature_c", "tank_pressure_pa"]:
            if am.get(key) is None:
                am[key] = 0.0
        am["tank_heater_on"] = bool(am.get("tank_heater_on", False))
       
        return snap

    def _fallback_summary(self, s: Dict[str, Any]) -> str:
        parts = [
            f"CO2 {s['co2_mmHg']:.2f} mmHg",
            f"O2 {s['o2_percent']:.1f}%",
            f"Cabin {s['temp_c']:.1f}°C",
        ]
        if s.get("water_l", 0.0) > 0:
            parts.append(f"Water {s['water_l']:.0f} L")
        th = s.get("thermal", {})
        if th:
            il = th.get("internal_loop_c", 0.0)
            parts.append(f"IntLoop {il:.1f}°C")
            ext = th.get("external_loop", {})
            if ext:
                parts.append(f"ExtIn {ext.get('loop_inlet_temp_c',0.0):.1f}°C")
                parts.append(f"ExtOut {ext.get('loop_outlet_temp_c',0.0):.1f}°C")
                rh = ext.get("received_heat_kj", 0.0)
                if rh > 0:
                    parts.append(f"Heat {rh:.1f} kJ")
        # diag summaries
        if s.get("diag", {}).get("level", 0) > 0:
            parts.append(f"SysDiag {s['diag'].get('name','')}={s['diag'].get('message','')}")
        if th.get("diag", {}).get("level", 0) > 0:
            parts.append(f"ThermDiag {th['diag'].get('name','')}={th['diag'].get('message','')}")
            
            
        am = th.get("ammonia", {})
        if am:
            parts.append(f"Ammonia {am.get('tank_temperature_c', 0.0):.1f}°C")
            if am.get("tank_heater_on"):
                parts.append("Heater ON")
                
                
        return " | ".join(parts)
