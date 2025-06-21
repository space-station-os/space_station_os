<template>
  <div class="eclss-grid">
    <!-- Column 1: Crew -->
    <div class="column crew">
      <CrewFigure :co2="co2_input" :o2="o2" :h2o="h2o" />
    </div>

    <!-- Column 2: Flow Arrows Left Side -->
    <div class="column arrows-left">
      <div class="drawn-arrow large">→</div>
      <div class="drawn-arrow large">→</div>
      <div class="drawn-arrow large">←</div>
    </div>

    <!-- Column 3: Status Cards -->
    <div class="column cards">
      <StatusCard
        title="CO₂ Separator"
        :value="co2_input"
        unit="ppm"
        @click="goTo('/ars')"
      />
      <div class="telemetry-block">
        <span>Input: {{ co2_input }} g</span>
        <span>Output: {{ co2_output }} g</span>
      </div>

      <StatusCard
        title="Pure H₂O Recycler"
        :value="h2o"
        unit="L"
        @click="goTo('/water')"
      />
      <div class="telemetry-block">
        <span>Input: {{ h2o }} L</span>
      </div>
      <StatusCard
        title="O₂ Recovery"
        :value="o2"
        unit="g"
        @click="goTo('/ogs')"
      />
      <div class="telemetry-block">
        <span>Output: {{ o2 }} g</span>
      </div>
    </div>

    <!-- Column 4: Right Side Arrows -->
    <div class="column arrows-right">
      <div class="drawn-arrow large">→</div>
      <div class="drawn-arrow large">←</div>
      <div class="drawn-arrow large">←</div>
    </div>

    <!-- Column 5: Sabatier -->
    <div class="column sabatier">
      <SabatierReactor
        :co2="co2_output"
        :h2="h2"
        :grayWater="gray_water"
        :methane="vented_methane"
      />
    </div>
  </div>
</template>

<script>
import CrewFigure from "./CrewFigure.vue";
import SabatierReactor from "./SabatierReactor.vue";
import StatusCard from "./StatusCard.vue";

export default {
  name: "EclssFlow",
  components: {
    CrewFigure,
    SabatierReactor,
    StatusCard,
  },
  data() {
    return {
      co2_input: 0,
      co2_output: 0,
      o2: 0,
      h2: 0,
      h2o: 0,
      gray_water: 0,
      vented_methane: 0,
      ros: null,
    };
  },
  computed: {
    co2StatusClass() {
      const diff = Math.abs(this.co2_input - this.co2_output);
      return diff > 2.0 ? "error-arrow" : "normal-arrow";
    },
  },
  mounted() {
    const ROSLIB = window.ROSLIB;
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const subscribe = (name, messageType, handler) => {
      const topic = new ROSLIB.Topic({ ros: this.ros, name, messageType });
      topic.subscribe(handler);
    };

    subscribe(
      "/collector_air_quality",
      "space_station_eclss/msg/AirData",
      (msg) => {
        this.co2_input = parseFloat(msg.co2_mass?.toFixed(2)) || 0;
      }
    );

    subscribe("/co2_storage", "std_msgs/msg/Float64", (msg) => {
      this.co2_output = parseFloat(msg.data?.toFixed(2)) || 0;
    });

    subscribe(
      "/wpa/tank_status",
      "space_station_eclss/msg/WaterCrew",
      (msg) => {
        this.h2o = parseFloat(msg.water?.toFixed(2)) || 0;
      }
    );

    subscribe(
      "/electrolysis_output",
      "space_station_eclss/msg/Electrolysis",
      (msg) => {
        this.o2 = parseFloat(msg.o2?.toFixed(2)) || 0;
        this.h2 = parseFloat(msg.h2?.toFixed(2)) || 0;
      }
    );

    subscribe("/grey_water", "std_msgs/msg/Float64", (msg) => {
      this.gray_water = parseFloat(msg.data?.toFixed(2)) || 0;
    });

    subscribe("/vented_methane", "std_msgs/msg/Float64", (msg) => {
      this.vented_methane = parseFloat(msg.data?.toFixed(2)) || 0;
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
  font-size: 3.6rem;
  line-height: 3.6rem;
  font-weight: bold;
  transition: color 0.3s;
}

.normal-arrow {
  color: #00ffe0;
}

.error-arrow {
  color: red;
}

.telemetry-block {
  background: #111;
  border-radius: 12px;
  padding: 8px 12px;
  color: #fff;
  font-size: 0.9rem;
  margin-top: -10px;
  margin-bottom: 20px;
  box-shadow: 0 0 6px #00ffe055;
  text-align: center;
}
</style>
