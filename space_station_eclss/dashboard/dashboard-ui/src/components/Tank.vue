<template>
  <div class="schematic-tank">
    <div class="tank-header">{{ label }}</div>

    <div class="tank-body">
      <!-- OGS -->
      <template v-if="type === 'ogs'">
        <div class="oxygen-layer" :style="{ height: oxygenHeight + '%' }"></div>
        <div
          class="hydrogen-layer"
          :style="{ height: hydrogenHeight + '%' }"
        ></div>
      </template>

      <!-- WRS -->
      <template v-else-if="type === 'wrs'">
        <div
          class="water-layer"
          :style="{
            height: waterHeight + '%',
            backgroundColor: waterColor,
          }"
        ></div>
      </template>

      <!-- ARS -->
      <template v-else>
        <div
          class="contaminants-layer"
          :style="{ height: contaminantsHeight + '%' }"
        ></div>
        <div
          class="moisture-layer"
          :style="{ height: moistureHeight + '%' }"
        ></div>
        <div class="co2-layer" :style="{ height: co2Height + '%' }"></div>
      </template>
    </div>

    <div class="tank-readout">
      <template v-if="type === 'ogs'">
        <p><strong>O₂:</strong> {{ formatValue(oxygen) }} mol</p>
        <p><strong>H₂:</strong> {{ formatValue(hydrogen) }} mol</p>
      </template>

      <template v-else-if="type === 'wrs'">
        <p><strong>H₂O:</strong> {{ formatValue(water) }} mL</p>
        <p><strong>Iodine:</strong> {{ formatValue(iodine) }}%</p>
        <p><strong>Contaminants:</strong> {{ formatValue(contaminants) }}%</p>
      </template>

      <template v-else>
        <p><strong>CO₂:</strong> {{ formatValue(co2) }} g</p>
        <p><strong>Moisture:</strong> {{ formatValue(moisture) }}%</p>
        <p><strong>Contaminants:</strong> {{ formatValue(contaminants) }}%</p>
      </template>
    </div>
  </div>
</template>

<script>
export default {
  name: "eclss-Tank",
  props: {
    label: String,
    type: { type: String, default: "ars" },
    co2: { type: Number, default: 0 },
    moisture: { type: Number, default: 0 },
    contaminants: { type: Number, default: 0 },
    oxygen: { type: Number, default: 0 },
    hydrogen: { type: Number, default: 0 },
    water: { type: Number, default: 0 },
    iodine: { type: Number, default: 0 },
    capacity: { type: Number, required: true },
  },
  computed: {
    totalContent() {
      if (this.type === "ogs") return this.oxygen + this.hydrogen;
      if (this.type === "wrs")
        return this.water + this.iodine + this.contaminants;
      return this.co2 + this.moisture + this.contaminants;
    },
    co2Height() {
      return this.toPercentage(this.co2);
    },
    moistureHeight() {
      return this.toPercentage(this.moisture);
    },
    contaminantsHeight() {
      return this.toPercentage(this.contaminants);
    },
    oxygenHeight() {
      return this.toPercentage(this.oxygen);
    },
    hydrogenHeight() {
      return this.toPercentage(this.hydrogen);
    },
    waterHeight() {
      return this.toPercentage(this.water);
    },
    iodineHeight() {
      return this.toPercentage(this.iodine);
    },
    waterColor() {
      // Contaminants-based interpolation: yellow (dirty) → blue (clean)
      const c = Math.min(Math.max(this.contaminants / 100, 0), 1); // normalize to [0,1]
      const start = [204, 153, 0]; // yellow
      const end = [0, 170, 255]; // clean blue
      const r = Math.round(start[0] * c + end[0] * (1 - c));
      const g = Math.round(start[1] * c + end[1] * (1 - c));
      const b = Math.round(start[2] * c + end[2] * (1 - c));
      return `rgb(${r}, ${g}, ${b})`;
    },
  },
  methods: {
    toPercentage(value) {
      if (this.totalContent === 0 || isNaN(value)) return 0;
      return Math.min((value / this.totalContent) * 100, 100);
    },
    formatValue(val) {
      return val !== undefined ? val.toFixed(2) : "0.00";
    },
  },
};
</script>

<style scoped>
.schematic-tank {
  width: 120px;
  background: #eee;
  border: 4px solid black;
  border-radius: 4px;
  overflow: hidden;
  box-shadow: 0 0 6px rgba(0, 0, 0, 0.3);
  text-align: center;
}

.tank-header {
  background: black;
  color: white;
  font-weight: bold;
  padding: 4px;
  font-size: 0.85rem;
}

.tank-body {
  height: 240px;
  position: relative;
  background: #dcdcdc;
}

.co2-layer,
.moisture-layer,
.contaminants-layer,
.oxygen-layer,
.hydrogen-layer,
.water-layer,
.iodine-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  transition: height 0.5s ease-in-out, background-color 0.5s ease-in-out;
  opacity: 0.75;
}

.co2-layer {
  background-color: #ffd700;
  z-index: 3;
}
.moisture-layer {
  background-color: #00bfff;
  z-index: 2;
}
.contaminants-layer {
  background-color: #32cd32;
  z-index: 1;
}
.oxygen-layer {
  background-color: #ff6347;
  z-index: 2;
}
.hydrogen-layer {
  background-color: #87cefa;
  z-index: 1;
}
.water-layer {
  position: absolute;
  bottom: 0;
  width: 100%;
  opacity: 0.85;
  z-index: 3;
  transition: height 0.5s ease-in-out, background-color 0.5s ease-in-out;
}

.iodine-layer {
  background-color: #ff69b4;
  z-index: 2;
}

.tank-readout {
  padding: 8px;
  font-size: 0.8rem;
  color: #000;
  background-color: rgba(255, 255, 255, 0.85);
}
</style>
