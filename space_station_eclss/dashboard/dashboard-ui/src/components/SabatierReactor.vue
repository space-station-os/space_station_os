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
<<<<<<< HEAD
=======

      <div class="bar-label">Gray Water</div>
      <div class="bar-shell">
        <div
          class="bar-fill gray"
          :style="{ width: grayWaterFill + '%' }"
        ></div>
      </div>

      <div class="bar-label">Vented Methane</div>
      <div class="bar-shell">
        <div
          class="bar-fill methane"
          :style="{ width: methaneFill + '%' }"
        ></div>
      </div>
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
    </div>
  </div>
</template>

<script>
export default {
  props: {
    co2: Number,
    h2: Number,
<<<<<<< HEAD
=======
    grayWater: Number,
    methane: Number,
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
  },
  computed: {
    co2Fill() {
      return Math.min(100, this.co2);
    },
    h2Fill() {
      return Math.min(100, this.h2);
    },
<<<<<<< HEAD
=======
    grayWaterFill() {
      return Math.min(100, this.grayWater);
    },
    methaneFill() {
      return Math.min(100, this.methane);
    },
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
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
<<<<<<< HEAD
=======
.bar-fill.gray {
  background: #a0a0a0;
}
.bar-fill.methane {
  background: #ffaa00;
}
>>>>>>> 4ff73ce2306d293b53e29e6e93e5c7a3ee237ace
</style>
