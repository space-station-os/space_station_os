<template>
  <div class="container">
    <h1>GNC Control Mode</h1>
    <div class="card">
      <div class="row">
        <div>
          <strong>Current Mode:</strong> <span>{{ modeString }}</span>
        </div>
        <div>
          <small>Updated:</small> <span>{{ lastUpdate }}</span>
        </div>
      </div>
      <div class="row">
        <select v-model.number="desiredMode">
          <option :value="0">AUTO</option>
          <option :value="1">CMG</option>
          <option :value="2">THRUSTER</option>
        </select>
        <button @click="setMode" :disabled="busy">Set Mode</button>
      </div>
      <div v-if="toast" class="toast">{{ toast }}</div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";

const MODE_STR = ["AUTO", "CMG", "THRUSTER"];

export default {
  name: "GncControl",
  data() {
    return {
      ros: null,
      mode: null,
      desiredMode: 0,
      lastUpdate: "-",
      busy: false,
      toast: "",
    };
  },
  computed: {
    modeString() {
      return this.mode === null
        ? "---"
        : MODE_STR[this.mode] ?? `UNKNOWN(${this.mode})`;
    },
  },
  methods: {
    connectRos() {
      this.ros = new ROSLIB.Ros({ url: process.env.VUE_APP_ROSBRIDGE_URL });
      this.ros.on("connection", () => {
        const sub = new ROSLIB.Topic({
          ros: this.ros,
          name: "/gnc/mode_actuation",
          messageType: "std_msgs/msg/String", // adjust if bridged differently
        });
        sub.subscribe((msg) => {
          const s = (msg.data || "").toLowerCase();
          this.mode =
            s === "auto" ? 0 : s === "cmg" ? 1 : s === "thruster" ? 2 : null;
          this.lastUpdate = new Date().toLocaleString();
        });
      });
      this.ros.on("error", (e) => {
        this.toast = `ROS error: ${e}`;
      });
      this.ros.on("close", () => {
        this.toast = "ROS connection closed";
      });
    },
    setMode() {
      if (this.busy) return;
      this.busy = true;
      const service = new ROSLIB.Service({
        ros: this.ros,
        name: "/gnc/set_mode_actuation",
        serviceType: "space_station_gnc/srv/SetGncModeActuation",
      });

      const map = ["auto", "cmg", "thruster"];
      const request = new ROSLIB.ServiceRequest({
        mode: map[this.desiredMode] || "auto", // default to "auto" if out of range
      });
      service.callService(
        request,
        (resp) => {
          this.toast = resp.message || (resp.success ? "Accepted" : "Rejected");
          this.busy = false;
        },
        (err) => {
          this.toast = `Call failed: ${err}`;
          this.busy = false;
        }
      );
    },
  },
  mounted() {
    this.connectRos();
  },
};
</script>

<style>
.container {
  max-width: 720px;
  margin: 24px auto;
  padding: 16px;
}
.card {
  border: 1px solid #333;
  border-radius: 8px;
  padding: 16px;
}
.row {
  display: flex;
  gap: 12px;
  align-items: center;
  margin-bottom: 12px;
}
.toast {
  margin-top: 8px;
  font-size: 0.9rem;
}
button {
  padding: 6px 12px;
}
select {
  padding: 6px 10px;
}
</style>
