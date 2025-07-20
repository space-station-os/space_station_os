<template>
  <div class="tank-container">
    <div class="tank-label">{{ label }}</div>
    <div class="tank-shell">
      <div
        class="tank-fill"
        :style="{ height: fillHeight + '%', backgroundColor: color }"
      ></div>
    </div>
    <div class="tank-value">{{ value }} {{ unit }}</div>
  </div>
</template>

<script>
export default {
  props: {
    label: String,
    value: Number,
    max: Number,
    unit: String,
    color: String,
  },
  computed: {
    fillHeight() {
      return Math.min(100, (this.value / this.max) * 100);
    },
  },
};
</script>

<style scoped>
.tank-container {
  width: 80px;
  text-align: center;
  margin: 0 10px;
  font-family: "Orbitron", sans-serif;
}
.tank-label {
  font-size: 0.9rem;
  margin-bottom: 6px;
}
.tank-shell {
  height: 150px;
  width: 40px;
  border: 1px solid #2a2e35;
  border-radius: 8px;
  overflow: hidden;
  background: #0d1117;
  margin: 0 auto;
  background-color: #0d1117;
}
.tank-fill {
  width: 100%;
  transition: height 0.5s ease;
}
.tank-value {
  font-size: 0.8rem;
  margin-top: 6px;
  color: #90e4ff;
}
</style>
