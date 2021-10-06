<template>
  <v-col>
    <!-- 버튼눌렀을때만 나오게 되어있는데
    robot이랑 연결되어있을때
    계속 이미지 받는걸로 만들어야됨
     -->
    <!-- <v-btn
      @click="getCam"
    >
      cam update
    </v-btn> -->
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
      // console.log(msg);
      const src = `data:image/png;base64,${msg}`;
      this.imgSrc = src;
      this.$store.dispatch('receiveCam');
    });
  },
  methods: {
    // getCam() {
    //   // console.log('cam');
    //   this.$socket.emit('back_robotview_front', 'cam');
    // },
    imageLoad() {},
    imageError() {},
  },
};
</script>

<style>

</style>
