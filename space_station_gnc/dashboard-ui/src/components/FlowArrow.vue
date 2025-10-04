<template>
  <div :class="['arrow', active ? 'active' : 'inactive']">{{ direction }}</div>
</template>

<script>
export default {
  props: {
    direction: String, // "→", "↓", etc.
    active: Boolean,
  },
};
</script>

<style scoped>
.arrow {
  font-size: 1.5rem;
  font-family: monospace;
  transition: color 0.3s, text-shadow 0.3s;
}
.active {
  color: #00ffe0;
  text-shadow: 0 0 6px #00ffe0;
}
.inactive {
  color: red;
  opacity: 0.6;
}
</style>
