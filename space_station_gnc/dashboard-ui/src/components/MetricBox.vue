<template>
  <div class="metric-box" @click="$emit('click')">
    <p class="metric-label">{{ label }}</p>
    <h2 class="metric-value">{{ value }} {{ unit }}</h2>
    <p
      class="metric-delta"
      :class="{
        positive: isPositive,
        negative: !isPositive,
        pulse: animateDelta,
      }"
    >
      {{ delta }}
    </p>
  </div>
</template>

<script>
export default {
  name: "MetricBox",
  props: {
    label: String,
    value: [String, Number],
    unit: String,
    delta: String,
  },
  data() {
    return {
      animateDelta: false,
    };
  },
  computed: {
    isPositive() {
      return this.delta?.includes("+");
    },
  },
  watch: {
    delta() {
      this.animateDelta = true;
      setTimeout(() => {
        this.animateDelta = false;
      }, 1000);
    },
  },
};
</script>

<style scoped>
.metric-box {
  border: 1px solid #2a2e35;
  background-color: #0d1117;
  font-family: "Orbitron", sans-serif;
  border-radius: 10px;
  padding: 20px;
  width: 200px;
  color: white;
  cursor: pointer;
  user-select: none;
  text-align: center;
  transition: transform 0.2s ease;
}
.metric-box:hover {
  transform: translateY(-2px);
}

.metric-label {
  font-size: 0.9rem;
  color: #8b949e;
  margin-bottom: 6px;
}

.metric-value {
  font-size: 1.8rem;
  font-weight: 600;
  margin-bottom: 6px;
}

.metric-delta {
  font-size: 0.9rem;
  font-weight: 500;
  transition: all 0.3s ease-in-out;
}

.positive {
  color: #3fb950; /* green */
}
.negative {
  color: #f85149; /* red */
}

.pulse {
  animation: pulseDelta 1s ease;
}

@keyframes pulseDelta {
  0% {
    transform: scale(1);
    opacity: 1;
  }
  50% {
    transform: scale(1.15);
    opacity: 0.85;
  }
  100% {
    transform: scale(1);
    opacity: 1;
  }
}
</style>
