<template>
  <div id="app">
    <!-- Navigation Bar -->
    <div class="top-bar">
      <img class="mission-icon" src="/assets/mission_patch.png" alt="Patch" />

      <!-- STL Live Status Badge -->
      <div class="status-badge">
        <span :class="'status-dot ' + statusColor"></span>
        <span class="status-label">{{ stlSummary }}</span>
      </div>
    </div>

    <!-- Routed View -->
    <router-view />
  </div>
</template>

<script>
/* global ROSLIB */
export default {
  name: "App",
  data() {
    return {
      ros: null,
      stlStates: {
        "/ars": "PASS",
        "/ogs": "PASS",
        "/water": "PASS",
      },
      currentPath: "/ars",
    };
  },
  computed: {
    stlSummary() {
      return this.stlStates[this.currentPath] || "UNKNOWN";
    },
    statusColor() {
      switch (this.stlSummary) {
        case "PASS":
          return "green";
        case "WARN":
          return "yellow";
        case "CODE_RED":
          return "red";
        default:
          return "gray";
      }
    },
  },
  mounted() {
    this.currentPath = this.$route.path;

    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const stlSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/stl_monitor/status",
      messageType: "std_msgs/String",
    });

    stlSub.subscribe((msg) => {
      try {
        const data = JSON.parse(msg.data);

        this.stlStates["/ars"] = this.computeStl([
          data.collector,
          data.desiccant_moisture,
          data.desiccant_contaminants,
          data.adsorbent,
        ]);
        this.stlStates["/ogs"] = this.computeStl([
          data.ELECTROLYSIS,
          data.OXYGEN_OUTPUT,
        ]);
        this.stlStates["/water"] = this.computeStl([
          data.WHC,
          data.UPA,
          data.FILTER,
          data.IONIZATION,
          data.CATALYTIC,
        ]);
      } catch (e) {
        console.error("Failed to parse STL monitor status:", e);
      }
    });
  },
  watch: {
    $route(to) {
      this.currentPath = to.path;
    },
  },
  methods: {
    computeStl(list) {
      if (list.includes("CODE_RED")) return "CODE_RED";
      if (list.includes("WARN")) return "WARN";
      return "PASS";
    },
  },
};
</script>

<style scoped>
#app {
  font-family: "Inter", sans-serif;
  min-height: 100vh;
  background-color: #0d1117;
  overflow-x: hidden;
}

/* Top bar layout */
.top-bar {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  background: rgba(15, 20, 30, 0.9);
  padding: 10px 20px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  backdrop-filter: blur(4px);
  z-index: 1000;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.6);
}

.mission-icon {
  width: 38px;
  height: 38px;
}

.status-badge {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 0.85rem;
  color: #ccc;
}

.status-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
}
.status-dot.green {
  background-color: #00ff6a;
}
.status-dot.yellow {
  background-color: #ffcc00;
}
.status-dot.red {
  background-color: #ff3b30;
}
.status-dot.gray {
  background-color: #888;
}
</style>
