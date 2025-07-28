<template>
  <div class="tank-card">
    <div class="card-header">
      <h3>{{ title }}</h3>
      <span v-if="subtitle" class="card-subtitle">{{ subtitle }}</span>
    </div>

    <div class="card-body">
      <slot />
      <!-- insert your <Tank /> component here -->
    </div>

    <div class="card-footer" v-if="note">
      <small>{{ note }}</small>
    </div>
  </div>
</template>

<script>
export default {
  name: "TankCard",
  props: {
    title: { type: String, required: true },
    subtitle: { type: String, default: "" },
    note: { type: String, default: "" },
  },
};
</script>

<style scoped>
.tank-card {
  background-color: #1a1d22;
  color: #ffffff;
  border-radius: 10px;
  border: 1px solid #2c2f36;
  width: 150px;
  padding: 12px;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.4);
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  transition: transform 0.3s ease;
}

.tank-card:hover {
  transform: translateY(-4px);
}

.card-header h3 {
  font-size: 1rem;
  margin: 0;
  font-weight: 600;
}

.card-subtitle {
  font-size: 0.75rem;
  color: #9fa6b2;
}

.card-body {
  margin-top: 10px;
  margin-bottom: 6px;
  display: flex;
  justify-content: center;
}

.card-footer {
  text-align: center;
  font-size: 0.7rem;
  color: #b0b0b0;
}
</style>
