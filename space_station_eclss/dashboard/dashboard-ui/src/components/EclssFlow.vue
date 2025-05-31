<template>
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
    </div>
  </div>
</template>

<script>
import CrewFigure from "./CrewFigure.vue";
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
};
</script>

<style scoped>
.layout-4col {
  display: grid;
  grid-template-columns: auto auto auto auto;
  justify-content: center;
  align-items: center;
  gap: 60px;
  padding: 60px 40px;
  font-family: "Orbitron", sans-serif;
}

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
}
</style>
