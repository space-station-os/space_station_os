<template>
  <div class="home-wrapper">
    <!-- Navigation -->
    <!-- <NavBar /> -->

    <div class="hero-section">
      <img src="/assets/mars.webp" alt="Earth from Space" class="hero-image" />
      <h1 class="main-title">Environmental and Life Support Systems (ECLSS)</h1>
      <p class="subtitle">
        Monitor and control the systems that sustain life on the space station.
      </p>
    </div>

<<<<<<< HEAD
    <!-- Top Telemetry Summary -->
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
=======
    <!-- NEW ECLSS Flow Diagram Section -->
    <div class="eclss-layout">
      <EclssLoop :co2="co2" :o2="o2" :h2o="h2o" />
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
    </div>

    <!-- NEW ECLSS Flow Diagram Section -->
    <div class="eclss-layout">
      <EclssLoop :co2="co2" :o2="o2" :h2o="h2o" />
    </div>
  </div>
</template>

<script>
// import NavBar from "../components/NavBar.vue";
<<<<<<< HEAD
import MetricBox from "../components/MetricBox.vue";
=======
// import MetricBox from "../components/MetricBox.vue";
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
import EclssLoop from "../components/EclssFlow.vue";

/* global ROSLIB */
export default {
  name: "HomePage",
  components: {
    // NavBar,
<<<<<<< HEAD
    MetricBox,
=======
    // MetricBox,
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
    EclssLoop,
  },
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

    // CO₂ from Air Collector
    const co2Sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/collector_air_quality",
      messageType: "space_station_eclss/msg/AirData",
    });
    co2Sub.subscribe((msg) => {
      this.co2 = msg.co2_mass?.toFixed(2) || 0;
    });

    // O₂ from Electrolysis
    const o2Sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/electrolysis_output",
      messageType: "space_station_eclss/msg/Electrolysis",
    });
    o2Sub.subscribe((msg) => {
      this.o2 = (msg.o2 || 0).toFixed(2);
    });

    // H₂O from Water Tank
    const waterSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/wpa/tank_status",
      messageType: "space_station_eclss/msg/WaterCrew",
    });
    waterSub.subscribe((msg) => {
      this.h2o = msg.water?.toFixed(2) || 0;
    });
  },
  methods: {
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
  padding: 40px 80px 80px;
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

<<<<<<< HEAD
.telemetry-section {
  margin-top: 40px;
  text-align: center;
}

.section-header {
  font-size: 1.6rem;
  font-weight: 600;
  margin-bottom: 20px;
}

.telemetry-grid {
=======
.eclss-layout {
  margin-top: 80px;
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
  display: flex;
  justify-content: center;
}

.eclss-layout {
  margin-top: 80px;
  display: flex;
  justify-content: center;
}
</style>
