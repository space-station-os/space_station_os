<template>
  <div class="tank air-collector">
    <h2>Air Collector</h2>
    <div class="tank-fill" :style="{ height: fillHeight + '%' }"></div>
    <p>CO₂ Mass: {{ co2.toFixed(2) }} g</p>
    <p>Moisture: {{ moisture.toFixed(2) }}%</p>
    <p>Contaminants: {{ contaminants.toFixed(2) }}%</p>
  </div>
</template>

<script>
export default {
  name: "AirCollector",
  props: {
    co2: Number,
    moisture: Number,
    contaminants: Number,
  },
  computed: {
    fillHeight() {
      const total = this.co2 + this.moisture + this.contaminants;
      return Math.min((total / 1000) * 100, 100); // assume 1000g = full tank
    },
  },
};
</script>

<style scoped>
.tank {
  width: 200px;
  height: 300px;
  border: 2px solid #007bff;
  background-color: #f0f0f0;
  border-radius: 10px;
  padding: 10px;
  position: relative;
  overflow: hidden;
}

.tank-fill {
  position: absolute;
  bottom: 0;
  width: 100%;
  background-color: rgba(0, 123, 255, 0.4);
  transition: height 0.5s ease-in-out;
}
</style>
