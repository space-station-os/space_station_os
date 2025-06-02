<template>
  <div class="tankbar-wrapper">
    <div class="label-row">
      <span class="tank-label">{{ label }}</span>
      <span class="tank-value">{{ value.toFixed(1) }}%</span>
    </div>
    <div class="bar-container">
      <div class="bar-fill" :style="{ width: fillWidth }"></div>
    </div>
  </div>
</template>

<script>
export default {
  name: "TankBar",
  props: {
    label: { type: String, required: true },
    value: { type: Number, required: true },
  },
  computed: {
    fillWidth() {
      return Math.min(this.value, 100) + "%";
    },
  },
};
</script>

<style scoped>
.tankbar-wrapper {
  width: 240px;
  background: #1b1f27;
  padding: 14px;
  border-radius: 8px;
  box-shadow: 0 2px 6px rgba(0, 255, 255, 0.2);
  color: white;
}

.label-row {
  display: flex;
  justify-content: space-between;
  font-size: 0.95rem;
  margin-bottom: 8px;
}

.bar-container {
  width: 100%;
  height: 16px;
  background-color: #30363d;
  border-radius: 6px;
  overflow: hidden;
}

.bar-fill {
  height: 100%;
  background: linear-gradient(to right, #00ffae, #00bfff);
  transition: width 0.6s ease;
}
</style>
