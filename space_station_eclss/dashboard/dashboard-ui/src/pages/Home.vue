<template>
  <div class="home-wrapper">
    <NavBar />

    <div class="hero-section">
      <img src="/assets/mars.webp" alt="Earth from Space" class="hero-image" />
      <h1 class="main-title">Environmental and Life Support Systems (ECLSS)</h1>
      <p class="subtitle">
        Monitor and control the systems that sustain life on the space station.
      </p>

      <select
        class="system-dropdown"
        v-model="selectedSystem"
        @change="goToSystem"
      >
        <option disabled value="">🌐 Select a System</option>
        <option value="ars">🌀 Air Revitalization</option>
        <option value="water">💧 Water Recovery</option>
        <option value="ogs">🧪 Oxygen Generation</option>
      </select>
    </div>

    <div class="telemetry-section">
      <h2 class="section-header">Live Telemetry</h2>
      <div class="telemetry-grid">
        <MetricBox
          label="Carbon Dioxide (CO₂)"
          :value="co2"
          unit="g"
          delta="+1.2%"
          color="#ffd700"
          @click="goTo('/ars')"
        />
        <MetricBox
          label="Oxygen (O₂)"
          :value="o2"
          unit="%"
          delta="-0.8%"
          color="#00ffae"
          @click="goTo('/ogs')"
        />
        <MetricBox
          label="Water (H₂O)"
          :value="h2o"
          unit="mL"
          delta="+2.5%"
          color="#3399ff"
          @click="goTo('/water')"
        />
      </div>
    </div>
  </div>
</template>

<script>
import NavBar from "../components/NavBar.vue";
import MetricBox from "../components/MetricBox.vue";

/* global ROSLIB */
export default {
  name: "HomePage",
  components: { NavBar, MetricBox },
  data() {
    return {
      selectedSystem: "",
      co2: 0,
      o2: 0,
      h2o: 0,
      ros: null,
    };
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    // CO2 from Air Collector
    const co2Sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/collector_air_quality",
      messageType: "demo_nova_sanctum/msg/AirData",
    });
    co2Sub.subscribe((msg) => {
      this.co2 = msg.co2_mass.toFixed(2);
    });

    // O2 from Electrolysis
    const o2Sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/electrolysis_output",
      messageType: "demo_nova_sanctum/msg/Electrolysis",
    });
    o2Sub.subscribe((msg) => {
      this.o2 = (msg.o2 || 0).toFixed(2);
    });

    // H2O from Water Tank
    const waterSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/wpa/tank_status",
      messageType: "demo_nova_sanctum/msg/WaterCrew",
    });
    waterSub.subscribe((msg) => {
      this.h2o = msg.water.toFixed(2);
    });
  },
  methods: {
    goToSystem() {
      if (this.selectedSystem) {
        this.$router.push(`/${this.selectedSystem}`);
      }
    },
    goTo(path) {
      this.$router.push(path);
    },
  },
};
</script>

<style scoped>
.home-wrapper {
  background-color: #0d1117;
  color: white;
  min-height: 100vh;
  padding: 40px 80px;
}

.hero-section {
  text-align: center;
  margin-bottom: 60px;
}

.hero-image {
  width: 100%;
  max-height: 300px;
  object-fit: cover;
  border-radius: 12px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.4);
}

.main-title {
  font-size: 2.8rem;
  font-weight: 700;
  margin: 20px 0 10px;
}

.subtitle {
  font-size: 1.2rem;
  color: #c0c0c0;
  margin-bottom: 30px;
}

.system-dropdown {
  padding: 12px 18px;
  font-size: 1rem;
  border-radius: 8px;
  background-color: #161b22;
  color: white;
  border: 1px solid #30363d;
  outline: none;
  width: 260px;
  text-align: center;
  font-weight: 600;
  cursor: pointer;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.5);
}
.system-dropdown:hover {
  background-color: #1d242e;
}

.telemetry-section {
  margin-top: 60px;
  text-align: center;
}

.section-header {
  font-size: 1.6rem;
  font-weight: 600;
  margin-bottom: 20px;
}

.telemetry-grid {
  display: flex;
  gap: 20px;
  margin-top: 10px;
  justify-content: center;
  flex-wrap: wrap;
}
</style>
