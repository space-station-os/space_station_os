<template>
  <div class="emergency-popup" v-if="show">
    <div class="popup-box">
      <h1>🚨 EMERGENCY!</h1>
      <p>
        A subsystem has entered <strong>CODE RED</strong> status. Immediate
        action required.
      </p>
      <button @click="dismiss">Acknowledge</button>
    </div>
  </div>
</template>

<script>
export default {
  props: ["show"],
  emits: ["dismiss"],
  methods: {
    dismiss() {
      this.$emit("dismiss");
    },
  },
};
</script>

<style scoped>
.emergency-popup {
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  background: rgba(255, 0, 0, 0.85);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 9999;
}

.popup-box {
  background: #1a0000;
  padding: 40px;
  border: 3px solid #ff0000;
  border-radius: 10px;
  color: white;
  text-align: center;
  font-family: "Courier New", monospace;
  box-shadow: 0 0 20px #ff0000;
}

.popup-box h1 {
  font-size: 2.5rem;
  color: #ff3333;
  margin-bottom: 20px;
  text-shadow: 0 0 10px #ff0000;
}

.popup-box button {
  background: black;
  color: #ff9999;
  border: 2px solid #ff9999;
  padding: 10px 20px;
  font-size: 1rem;
  border-radius: 5px;
  cursor: pointer;
}

.popup-box button:hover {
  background: #ff3333;
  color: white;
}
</style>
