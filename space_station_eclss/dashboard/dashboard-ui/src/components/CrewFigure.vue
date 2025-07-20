<template>
  <div class="crew-figure">
    <svg viewBox="0 0 200 400" width="200" height="400">
      <!-- Outer shape -->
      <path
        d="M100,20
           C130,20 150,40 150,70
           C150,90 140,110 120,110
           C140,120 150,140 150,180
           L150,350
           C150,370 130,380 100,380
           C70,380 50,370 50,350
           L50,180
           C50,140 60,120 80,110
           C60,110 50,90 50,70
           C50,40 70,20 100,20
           Z"
        fill="#111"
        stroke="#00ffe0"
        stroke-width="2"
      />

      <!-- Inner fill: CO2 (red, lower), H2O (blue, middle), O2 (green, upper) -->
      <rect
        :y="calcFillY('co2')"
        x="60"
        width="80"
        :height="fillHeights.co2"
        fill="#ff6b6b"
        opacity="0.6"
      />
      <rect
        :y="calcFillY('h2o')"
        x="60"
        width="80"
        :height="fillHeights.h2o"
        fill="#3399ff"
        opacity="0.5"
      />
      <rect
        :y="calcFillY('o2')"
        x="60"
        width="80"
        :height="fillHeights.o2"
        fill="#4fdc74"
        opacity="0.4"
      />
    </svg>
    <div class="label">Crew Metabolism</div>
  </div>
</template>

<script>
export default {
  props: {
    co2: Number,
    o2: Number,
    h2o: Number,
  },
  computed: {
    fillHeights() {
      return {
        co2: Math.min(80, this.co2 * 0.1),
        o2: Math.min(80, this.o2 * 0.1),
        h2o: Math.min(80, this.h2o * 5),
      };
    },
  },
  methods: {
    calcFillY(type) {
      return 350 - this.fillHeights[type];
    },
  },
};
</script>

<style scoped>
.crew-figure {
  text-align: center;
}
.label {
  margin-top: 10px;
  font-family: "Orbitron", sans-serif;
  font-size: 1rem;
}
</style>
