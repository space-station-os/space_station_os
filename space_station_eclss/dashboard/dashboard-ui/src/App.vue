<template>
  <div id="app">
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
  font-family: "Orbitron", sans-serif;
  min-height: 100vh;
  background-color: #000000;
  color: #ffffff;
  overflow-x: hidden;
}
</style>
