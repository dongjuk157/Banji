<template>
  <div>
    <Map />
    <CameraView />
    <Move />
    <v-icon
      v-if="camCheck"
      aria-hidden="false"
      x-large
      @click="onScreenshot"
      id="screenshot"
    >
      fas fa-camera
    </v-icon>
  </div>
</template>

<script>
import { mapGetters } from 'vuex';
import CameraView from './CameraView.vue';
import Map from './Map.vue';
import Move from './Move.vue';

export default {
  components: {
    CameraView,
    Map,
    Move,
  },
  methods: {
    onScreenshot() {
      this.$socket.emit('back_screenshot_front');
      // eslint-disable-next-line no-alert
      alert('저장되었습니다');
    },
  },
  computed: {
    ...mapGetters([
      'camCheck',
    ]),
  },
};
</script>

<style>
#screenshot {
  position: absolute;
  bottom: 1rem;
  right: 1rem;
}
</style>
