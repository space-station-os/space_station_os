<template>
  <div :style="backgroundStyle">
    <h1>🛰️ ISS Air Revitalization System</h1>

    <StatusHUD
      :status="mainStatus"
      :mode="systemMode"
      :temperature="temperature"
      :pressure="pressure"
      :stlStatus="stlStatus"
    />

    <div class="dashboard-layout">
      <TankCard title="Air Collector">
        <Tank
          type="ars"
          :co2="co2_mass"
          :moisture="moisture_content"
          :contaminants="contaminants"
          :capacity="1000"
        />
      </TankCard>

      <Pipe />

      <TankCard title="Desiccant Bed">
        <Tank
          type="ars"
          :co2="desiccant_co2"
          :moisture="desiccant_moisture"
          :contaminants="desiccant_contaminants"
          :capacity="1000"
        />
      </TankCard>

      <Pipe />

      <TankCard title="Adsorbent Bed">
        <Tank
          type="ars"
          :co2="adsorbent_co2"
          :moisture="adsorbent_moisture"
          :contaminants="adsorbent_contaminants"
          :capacity="1000"
        />
      </TankCard>
    </div>

    <EmergencyPopup
      :show="emergencyActive"
      @dismiss="emergencyActive = false"
    />
  </div>
</template>

<script>
/* global ROSLIB */
import Tank from "../components/Tank.vue";
import Pipe from "../components/Pipe.vue";
import StatusHUD from "../components/StatusHUD.vue";
import EmergencyPopup from "../components/Emergency_popup.vue";
import TankCard from "../components/TankCard.vue";

export default {
  components: { Tank, Pipe, StatusHUD, EmergencyPopup, TankCard },
  name: "ArsSystem",
  data() {
    return {
      ros: null,
      co2_mass: 0,
      moisture_content: 0,
      contaminants: 0,
      desiccant_co2: 0,
      desiccant_moisture: 0,
      desiccant_contaminants: 0,
      adsorbent_co2: 0,
      adsorbent_moisture: 0,
      adsorbent_contaminants: 0,
      imgUrl: "/assets/iss_bg.jpg",
      mainStatus: "Nominal",
      systemMode: "Idle",
      temperature: 72.0,
      pressure: 14.5,
      stlStatus: {
        collector: "PASS",
        desiccant_moisture: "PASS",
        desiccant_contaminants: "PASS",
        adsorbent: "PASS",
      },
      emergencyActive: false,
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
      };
    },
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const subscribeTopic = (topic, cb) => {
      const sub = new ROSLIB.Topic({
        ros: this.ros,
        name: topic,
        messageType: "space_station_eclss/msg/AirData",
      });
      sub.subscribe(cb);
    };

    subscribeTopic("/collector_air_quality", (msg) => {
      this.co2_mass = msg.co2_mass;
      this.moisture_content = msg.moisture_content;
      this.contaminants = msg.contaminants;
    });

    subscribeTopic("/desiccant_air_quality", (msg) => {
      this.desiccant_co2 = msg.co2_mass;
      this.desiccant_moisture = msg.moisture_content;
      this.desiccant_contaminants = msg.contaminants;
    });

    subscribeTopic("/adsorbent_air_quality", (msg) => {
      this.adsorbent_co2 = msg.co2_mass;
      this.adsorbent_moisture = msg.moisture_content;
      this.adsorbent_contaminants = msg.contaminants;
    });

    const stlListener = new ROSLIB.Topic({
      ros: this.ros,
      name: "/stl_monitor/status",
      messageType: "std_msgs/String",
    });

    stlListener.subscribe((msg) => {
      try {
        const parsed = JSON.parse(msg.data);
        this.stlStatus = parsed;
        if (Object.values(parsed).includes("CODE_RED")) {
          this.emergencyActive = true;
        }
      } catch (e) {
        console.error("Failed to parse STL monitor status:", e);
      }
    });
  },
};
</script>

<style scoped>
h1 {
  text-align: center;
  color: #ffffff;
  text-shadow: 1px 1px 4px #000;
  font-size: 2.2rem;
  margin-bottom: 20px;
  font-weight: 600;
}

.dashboard-layout {
  display: flex;
  justify-content: center;
  align-items: flex-end;
  gap: 30px;
  margin-top: 30px;
  flex-wrap: wrap;
}
</style>
