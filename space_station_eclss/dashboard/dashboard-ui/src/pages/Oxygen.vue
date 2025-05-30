<template>
  <div :style="backgroundStyle">
    <h1>🧪 Oxygen Generation System</h1>

    <StatusHUD
      :status="systemStatus"
      :mode="'Electrolysis Only'"
      :temperature="temperature"
      :pressure="pressure"
      :stlStatus="stlStatus"
    />

    <div class="ogs-layout">
      <TankCard title="Electrolysis Unit">
        <Tank type="ogs" :oxygen="o2" :hydrogen="h2" :capacity="100" />
      </TankCard>

      <Pipe />

      <TankCard title="Oxygen Outlet">
        <Tank type="ogs" :oxygen="o2" :hydrogen="0" :capacity="100" />
      </TankCard>
    </div>
  </div>
</template>

<script>
/* global ROSLIB */
import Tank from "../components/Tank.vue";
import Pipe from "../components/Pipe.vue";
import TankCard from "../components/TankCard.vue";
import StatusHUD from "../components/StatusHUD.vue";

export default {
  name: "OxygenSystem",
  components: { Tank, Pipe, TankCard, StatusHUD },
  data() {
    return {
      ros: null,
      h2: 0,
      o2: 0,
      temperature: 0,
      pressure: 0,
      systemStatus: "Nominal",
      imgUrl: "/assets/iss_bg.jpg",
      stlStatus: {
        ELECTROLYSIS: "PASS",
        OXYGEN_OUTPUT: "PASS",
      },
    };
  },
  computed: {
    backgroundStyle() {
      return {
        backgroundImage: `url(${this.imgUrl})`,
        backgroundSize: "cover",
        backgroundPosition: "center",
        backgroundAttachment: "fixed",
        minHeight: "100vh",
        paddingTop: "20px",
        color: "white",
      };
    },
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const electrolysisSub = new ROSLIB.Topic({
      ros: this.ros,
      name: "/electrolysis_output",
      messageType: "demo_nova_sanctum/msg/Electrolysis",
    });

    electrolysisSub.subscribe((msg) => {
      this.h2 = msg.h2;
      this.o2 = msg.o2;
      this.temperature = msg.temperature.temperature;
      this.pressure = msg.pressure.fluid_pressure;

      this.systemStatus =
        this.temperature > 80
          ? "OVERHEAT"
          : this.pressure > 16
          ? "HIGH PRESSURE"
          : "Nominal";
    });
  },
};
</script>

<style scoped>
h1 {
  text-align: center;
  font-size: 2.3rem;
  color: #ffffff;
  text-shadow: 1px 1px 4px #000;
  margin-bottom: 20px;
}

.ogs-layout {
  display: flex;
  justify-content: center;
  align-items: flex-end;
  gap: 30px;
  margin-top: 30px;
  flex-wrap: wrap;
}
</style>
