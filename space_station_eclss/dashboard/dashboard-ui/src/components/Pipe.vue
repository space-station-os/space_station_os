<template>
  <div class="pipe"></div>
</template>
<script>
export default {
  name: "PipeConnection",
};
</script>
<style scoped>
.pipe {
  width: 60px;
  height: 10px;
  background: repeating-linear-gradient(
    135deg,
    #333,
    #333 10px,
    #00bcd4 10px,
    #00bcd4 20px
  );
  border-radius: 10px;
  margin: 0 15px;
  animation: flow 1s linear infinite;
}

@keyframes flow {
  from {
    background-position: 0;
  }
  to {
    background-position: 40px;
  }
}
</style>
