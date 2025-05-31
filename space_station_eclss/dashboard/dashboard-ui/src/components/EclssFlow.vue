<template>
  <div class="eclss-grid">
    <!-- Column 1: Crew -->
    <div class="column crew">
      <CrewFigure :co2="co2" :o2="o2" :h2o="h2o" />
    </div>

    <!-- Column 2: Flow Arrows Left Side -->
    <div class="column arrows-left">
      <div class="drawn-arrow large">→</div>
      <div class="drawn-arrow large">→</div>
      <div class="drawn-arrow large">→</div>
    </div>

    <!-- Column 3: Status Cards -->
    <div class="column cards">
      <StatusCard
        title="CO₂ Separator"
        :value="co2"
        unit="ppm"
        @click="goTo('/ars')"
      />
      <StatusCard
        title="Pure H₂O Recycler"
        :value="h2o"
        unit="L"
        @click="goTo('/water')"
      />
      <StatusCard
        title="O₂ Recovery"
        :value="o2"
        unit="ppm"
        @click="goTo('/ogs')"
      />
    </div>

    <!-- Column 4: Right Side Arrows -->
    <div class="column arrows-right">
      <div class="drawn-arrow large">→</div>
      <div class="drawn-arrow large">←</div>
      <div class="drawn-arrow large">↗</div>
    </div>

    <!-- Column 5: Sabatier -->
    <div class="column sabatier">
      <SabatierReactor :co2="co2" :h2="h2" />
    </div>
  </div>
</template>

<script>
import CrewFigure from "./CrewFigure.vue";
import SabatierReactor from "./SabatierReactor.vue";
import StatusCard from "./StatusCard.vue";

export default {
  name: "EclssLoop",
  components: {
    CrewFigure,
    SabatierReactor,
    StatusCard,
  },
  data() {
    return {
      co2: 0,
      o2: 0,
      h2o: 0,
      h2: 0,
      ros: null,
    };
  },
  mounted() {
    const ROSLIB = window.ROSLIB;
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const co2Sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/collector_air_quality",
      messageType: "space_station_eclss/msg/AirData",
    });
    co2Sub.subscribe((msg) => {
      this.co2 = msg.co2_mass?.toFixed(2) || 0;
    });

    const o2Sub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/electrolysis_output",
      messageType: "space_station_eclss/msg/Electrolysis",
    });
    o2Sub.subscribe((msg) => {
      this.o2 = msg.o2?.toFixed(2) || 0;
      this.h2 = msg.h2?.toFixed(2) || 0;
    });

    const h2oSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/wpa/tank_status",
      messageType: "space_station_eclss/msg/WaterCrew",
    });
    h2oSub.subscribe((msg) => {
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
.eclss-grid {
  display: grid;
  grid-template-columns: auto 60px 240px 60px 320px;
  gap: 20px;
  justify-items: center;
  align-items: center;
  padding: 60px 40px;
  font-family: "Orbitron", sans-serif;
}

.column {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 30px;
}

.drawn-arrow {
  font-size: 2.4rem;
  color: #ffffff;
  line-height: 2.5rem;
  font-weight: bold;
}

.drawn-arrow.large {
  font-size: 3.6rem;
  line-height: 3.6rem;
  font-weight: bold;
}
</style>
