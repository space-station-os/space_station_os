<template>
  <div class="status-card">
    <h4 class="card-title">{{ title }}</h4>
    <div class="card-value">
      {{ value }}
    </div>
    <div
      class="card-delta"
      :class="{ positive: isPositive, negative: isNegative }"
    >
      {{ delta }}
    </div>
  </div>
</template>

<script>
export default {
  name: "StatusCard",
  props: {
    title: { type: String, required: true },
    value: { type: String, default: "—" },
    delta: { type: String, default: "" },
  },
  computed: {
    isPositive() {
      return this.delta.trim().startsWith("+");
    },
    isNegative() {
      return this.delta.trim().startsWith("-");
    },
  },
};
</script>

<style scoped>
.status-card {
  background-color: #161b22;
  color: white;
  padding: 18px 24px;
  border-radius: 12px;
  min-width: 180px;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.4);
  transition: all 0.3s ease-in-out;
  display: flex;
  flex-direction: column;
  align-items: flex-start;
}

.card-title {
  font-size: 0.9rem;
  font-weight: 500;
  color: #bbb;
  margin-bottom: 8px;
}

.card-value {
  font-size: 1.6rem;
  font-weight: 600;
  margin-bottom: 6px;
}

.card-delta {
  font-size: 0.85rem;
  font-weight: 500;
}

.card-delta.positive {
  color: #00ff6a;
  animation: pulse-up 0.8s ease-in-out;
}

.card-delta.negative {
  color: #ff4f4f;
  animation: pulse-down 0.8s ease-in-out;
}

@keyframes pulse-up {
  0% {
    transform: scale(1);
    opacity: 0.6;
  }
  50% {
    transform: scale(1.2);
    opacity: 1;
  }
  100% {
    transform: scale(1);
    opacity: 0.6;
  }
}

@keyframes pulse-down {
  0% {
    transform: scale(1);
    opacity: 0.6;
  }
  50% {
    transform: scale(0.9);
    opacity: 1;
  }
  100% {
    transform: scale(1);
    opacity: 0.6;
  }
}
</style>
