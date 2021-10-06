<template>
  <div>
    <Map />
    <CameraView />
    <Move />
  </div>
</template>

<script>
import CameraView from './CameraView.vue';
import Map from './Map.vue';
import Move from './Move.vue';

export default {
  components: {
    CameraView,
    Map,
    Move,
  },
};
</script>

<style>

</style>
