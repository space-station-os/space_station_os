<template>
  <div class="wrs-wrapper">
    <h1>💧 Water Recovery System</h1>

    <!-- STATUS CARDS -->
    <div class="status-grid">
      <StatusCard title="System Health" :value="mainStatus" delta="+5%" />
      <StatusCard
        title="Recovery Rate"
        :value="`${recoveryRate}%`"
        delta="+2%"
      />
      <StatusCard
        title="Tank Level"
        :value="`${finalWater.level}%`"
        delta="-10%"
      />
    </div>

    <!-- TANK LEVEL BAR -->
    <div class="level-bar">
      <label>Water Tank Level</label>
      <div class="bar-wrapper">
        <div class="bar-fill" :style="{ width: `${finalWater.level}%` }"></div>
        <span class="bar-label">{{ finalWater.level.toFixed(1) }}%</span>
      </div>
    </div>

    <!-- PIPELINE VISUALIZATION -->
    <div class="pipeline-layout">
      <TankCard title="WHC">
        <Tank
          type="wrs"
          :water="crewUse.moisture"
          :contaminants="crewUse.contaminants"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Waste Collection">
        <Tank
          type="wrs"
          :water="wasteStatus.level"
          :contaminants="wasteStatus.contaminants"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="UPA">
        <Tank
          type="wrs"
          :water="upaStatus.distillate"
          :contaminants="upaStatus.brine"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Filtration Unit">
        <Tank
          type="wrs"
          :water="filterationStatus.filtered_output"
          :contaminants="filterationStatus.contaminants"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Ionization Bed">
        <Tank
          type="wrs"
          :water="ionStatus.cleaned_output"
          :iodine="ionStatus.iodine"
          :contaminants="ionStatus.contaminants"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Catalytic Chamber">
        <Tank
          type="wrs"
          :water="cleanStatus.pure_water"
          :contaminants="cleanStatus.contaminants"
          :capacity="100"
        />
      </TankCard>
      <Pipe />
      <TankCard title="Product Water Tank">
        <Tank
          type="wrs"
          :water="finalWater.level"
          :iodine="finalWater.iodine"
          :contaminants="finalWater.contaminants"
          :capacity="100"
        />
      </TankCard>
    </div>
  </div>
</template>

<script>
/* global ROSLIB */
import Tank from "../components/Tank.vue";
import Pipe from "../components/Pipe.vue";
import TankCard from "../components/TankCard.vue";
import StatusCard from "../components/StatusCard.vue";

export default {
  name: "WaterSystemsPage",
  components: { Tank, Pipe, TankCard, StatusCard },
  data() {
    return {
      ros: null,
      crewUse: { moisture: 0, contaminants: 0 },
      wasteStatus: { level: 0, contaminants: 0 },
      upaStatus: { distillate: 0, brine: 0 },
      filterationStatus: { filtered_output: 0, contaminants: 0 },
      ionStatus: { cleaned_output: 0, iodine: 0, contaminants: 0 },
      cleanStatus: { pure_water: 0, contaminants: 0 },
      finalWater: { level: 0, iodine: 0, contaminants: 0 },
      mainStatus: "Operational",
      recoveryRate: 95,
    };
  },
  mounted() {
    this.ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    const subscribe = (name, callback) => {
      const topic = new ROSLIB.Topic({
        ros: this.ros,
        name,
        messageType: "demo_nova_sanctum/msg/WaterCrew",
      });
      topic.subscribe(callback);
    };

    subscribe("/whc_controller_water", (msg) => {
      this.crewUse = {
        moisture: msg.water,
        contaminants: msg.contaminants,
      };
    });

    subscribe("/whc/collector_status", (msg) => {
      this.wasteStatus = {
        level: msg.water,
        contaminants: msg.contaminants,
      };
    });

    subscribe("/whc/upa", (msg) => {
      this.upaStatus = {
        distillate: msg.water,
        brine: msg.contaminants,
      };
    });

    subscribe("/wpa/filteration_status", (msg) => {
      this.filterationStatus = {
        filtered_output: msg.water,
        contaminants: msg.contaminants,
      };
    });

    subscribe("/wpa/ionization_status", (msg) => {
      this.ionStatus = {
        cleaned_output: msg.water,
        iodine: msg.iodine_level,
        contaminants: msg.contaminants,
      };
    });

    subscribe("/wpa/catalytic_chamber", (msg) => {
      this.cleanStatus = {
        pure_water: msg.water,
        contaminants: msg.contaminants,
      };
    });

    subscribe("/wpa/tank_status", (msg) => {
      this.finalWater = {
        level: msg.water,
        iodine: msg.iodine_level,
        contaminants: msg.contaminants,
      };
    });
  },
};
</script>

<style scoped>
.wrs-wrapper {
  background-color: #0d1117;
  color: white;
  padding: 40px;
}

h1 {
  text-align: center;
  font-size: 2.3rem;
  margin-bottom: 40px;
}

.status-grid {
  display: flex;
  gap: 20px;
  justify-content: center;
  margin-bottom: 30px;
}

.level-bar {
  max-width: 600px;
  margin: 20px auto 50px;
}

.bar-wrapper {
  background: #333;
  height: 14px;
  border-radius: 6px;
  overflow: hidden;
  position: relative;
  margin-top: 4px;
}

.bar-fill {
  height: 100%;
  background-color: #00aaff;
  transition: width 0.5s ease-in-out;
}

.bar-label {
  position: absolute;
  right: 10px;
  top: -22px;
  font-size: 0.9rem;
  color: #ccc;
}

.pipeline-layout {
  display: flex;
  justify-content: center;
  align-items: flex-end;
  gap: 30px;
  flex-wrap: wrap;
  margin-top: 30px;
}
</style>
