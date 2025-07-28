<template>
  <div class="hud-container">
    <div class="core-status">
      <div class="status-block">
        <label>Status:</label> <span>{{ status }}</span>
      </div>
      <div class="status-block">
        <label>Mode:</label> <span>{{ mode }}</span>
      </div>
      <div class="status-block">
        <label>Temp:</label> <span>{{ temperature }} °C</span>
      </div>
      <div class="status-block">
        <label>Pressure:</label> <span>{{ pressure }} psi</span>
      </div>
    </div>

    <div class="stl-panel">
      <h3>🧪 STL Monitor</h3>
      <div class="stl-grid">
        <div
          v-for="(value, key) in stlStatus"
          :key="key"
          :class="['stl-entry', value.toLowerCase()]"
        >
          <span class="led" :class="value.toLowerCase()"></span>
          {{ key.replaceAll("_", " ").toUpperCase() }}:
          <strong>{{ value }}</strong>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    status: String,
    mode: String,
    temperature: Number,
    pressure: Number,
    stlStatus: Object,
  },
};
</script>

<style scoped>
.hud-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  border: 2px solid #00bcd4;
  background: rgba(0, 0, 0, 0.65);
  border-radius: 10px;
  padding: 16px 24px;
  margin-bottom: 20px;
  font-family: "Courier New", monospace;
  color: #00ffe0;
  box-shadow: 0 0 12px #00bcd4;
}

.core-status {
  display: flex;
  gap: 30px;
  margin-bottom: 14px;
  font-size: 1rem;
}

.status-block label {
  color: #cccccc;
  font-weight: bold;
  margin-right: 5px;
}

.stl-panel {
  width: 100%;
  border-top: 1px solid #00bcd4;
  padding-top: 12px;
}

.stl-panel h3 {
  margin-bottom: 8px;
  font-size: 1rem;
  color: #00ffe0;
  text-shadow: 0 0 4px #007c9c;
}

.stl-grid {
  display: flex;
  flex-wrap: wrap;
  gap: 10px 20px;
  justify-content: center;
}

.stl-entry {
  font-size: 0.85rem;
  display: flex;
  align-items: center;
  gap: 8px;
}

.led {
  width: 12px;
  height: 12px;
  border-radius: 50%;
  display: inline-block;
  box-shadow: 0 0 5px rgba(0, 255, 0, 0.8);
}

.pass .led {
  background-color: #00ff00;
}

.fail .led {
  background-color: #ffaa00;
  animation: blink 1s infinite;
}

.code_red .led {
  background-color: red;
  animation: blink 0.6s infinite;
}

@keyframes blink {
  0%,
  100% {
    opacity: 1;
  }
  50% {
    opacity: 0.3;
  }
}
</style>
