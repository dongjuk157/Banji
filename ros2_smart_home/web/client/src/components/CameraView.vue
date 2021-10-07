<template>
  <v-col>
    <v-img
      @load="imageLoad"
      @error="imageError"
      :src="imgSrc"
    />
  </v-col>
</template>

<script>
export default {
  data() {
    return {
      imgSrc: null,
    };
  },
  mounted() {
    this.$socket.on('front_robotview_back', (msg) => {
      const src = `data:image/png;base64,${msg}`;
      this.imgSrc = src;
      this.$store.dispatch('receiveCam');
    });
  },
  methods: {
    // getCam() {
    //   this.$socket.emit('back_robotview_front', 'cam');
    // },
    imageLoad() {},
    imageError() {},
  },
};
</script>

<style>

</style>
