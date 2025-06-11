<template>
<<<<<<< HEAD
  <div class="layout-4col">
    <!-- Column 1: Human -->
    <div class="col human">
      <CrewFigure :co2="co2" :o2="o2" :h2o="h2o" />
    </div>

    <!-- Column 2: Tanks -->
    <div class="col tanks">
      <VerticalTank
        label="CO₂"
        :value="co2"
        max="1000"
        unit="ppm"
        color="#ffd700"
      />
      <VerticalTank
        label="O₂"
        :value="o2"
        max="1000"
        unit="ppm"
        color="#00ffae"
      />
      <VerticalTank
        label="H₂O"
        :value="h2o"
        max="20"
        unit="L"
        color="#3399ff"
      />
    </div>

    <!-- Column 3: Arrows -->
    <div class="col arrows">
      <div class="arrow-wrapper" style="margin-top: 40px">
        <FlowArrow direction="→ CO₂" :active="true" />
      </div>
      <div class="arrow-wrapper" style="margin-top: 80px">
        <FlowArrow direction="→ H₂" :active="true" />
      </div>
      <div class="arrow-wrapper" style="margin-top: 100px">
        <FlowArrow direction="← H₂O" :active="true" />
      </div>
    </div>

    <!-- Column 4: Sabatier -->
    <div class="col sabatier">
      <SabatierReactor :co2="co2" :h2="h2" />
=======
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
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
    </div>
  </div>
</template>

<script>
import CrewFigure from "./CrewFigure.vue";
<<<<<<< HEAD
import VerticalTank from "./VerticalTank.vue";
import SabatierReactor from "./SabatierReactor.vue";
import FlowArrow from "./FlowArrow.vue";

export default {
  name: "EclssLoop",
  components: {
    CrewFigure,
    VerticalTank,
    SabatierReactor,
    FlowArrow,
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
=======
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
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
  mounted() {
    const ROSLIB = window.ROSLIB;
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

<<<<<<< HEAD
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
=======
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
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
};
</script>

<style scoped>
<<<<<<< HEAD
.layout-4col {
  display: grid;
  grid-template-columns: auto auto auto auto;
  justify-content: center;
  align-items: center;
  gap: 60px;
=======
.eclss-grid {
  display: grid;
  grid-template-columns: auto 60px 240px 60px 320px;
  gap: 20px;
  justify-items: center;
  align-items: center;
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
  padding: 60px 40px;
  font-family: "Orbitron", sans-serif;
}

<<<<<<< HEAD
.arrow-wrapper {
  width: 100%;
  display: flex;
  justify-content: center;
}

.col {
  display: flex;
  flex-direction: column;
  align-items: center;
}

.label {
  margin-top: 16px;
  font-size: 1.2rem;
  color: #90e4ff;
}

.tanks {
  gap: 30px;
}

.arrows {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 24px;
  margin-top: 40px;
  font-size: 1.3rem;
  font-weight: 600;
  height: 340px;
  color: #00ffe0;
}

.sabatier {
  margin-top: 0px;
=======
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
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
}
</style>
