<template>
  <div class="sabatier-block">
    <h3>Sabatier Reactor</h3>
    <div class="bars">
      <div class="bar-label">CO₂</div>
      <div class="bar-shell">
        <div class="bar-fill" :style="{ width: co2Fill + '%' }"></div>
      </div>

      <div class="bar-label">H₂</div>
      <div class="bar-shell">
        <div class="bar-fill h2" :style="{ width: h2Fill + '%' }"></div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    co2: Number,
    h2: Number,
  },
  computed: {
    co2Fill() {
      return Math.min(100, this.co2);
    },
    h2Fill() {
      return Math.min(100, this.h2);
    },
  },
};
</script>

<style scoped>
.sabatier-block {
  padding: 20px;
  background: #121b2e;
  border: 2px solid #00ffe0;
  border-radius: 12px;
  width: 250px;
  color: white;
  text-align: center;
  font-family: "Orbitron", sans-serif;
}
.bars {
  margin-top: 10px;
}
.bar-label {
  font-size: 0.9rem;
  margin-top: 10px;
}
.bar-shell {
  width: 100%;
  height: 20px;
  border: 1px solid #00ffe0;
  margin-top: 4px;
  background: #0d1117;
}
.bar-fill {
  height: 100%;
  background: #ff6b6b;
  transition: width 0.5s;
}
.bar-fill.h2 {
  background: #4fdc74;
}
</style>
