<template>
  <v-row class="pa-3">
    <!-- 로직 2. 침입자 인지 시점 이미지 뷰어 생성 -->
    <!-- <v-col cols="6">
      <v-img :src="imageSrc" width="480" @load="imageLoad" @error="imageError"></v-img>
    </v-col> -->
    <!-- 로직 4. 수동 컨트롤 버튼 생성 -->
    <!-- <v-col cols="6">
      <p>Manual Controller</p>
      <Move />
    </v-col> -->
    <v-col>
      <v-btn
        @click="getCam"
      >
        cam
      </v-btn>
      <v-img
        :src="imgSrc"
      />
    </v-col>
  </v-row>
</template>

<script>
// import Move from './Move.vue';

export default {
  components: {
    // Move,
  },
  data() {
    return {
      time: new Date().getTime(),
      imgSrc: null,
    };
  },
  mounted() {
    this.$socket.on('front_robotview_back', (msg) => {
      console.log(msg);
      const src = `data:image/png;base64,${msg}`;
      this.imgSrc = src;
    });
  },
  methods: {
    getCam() {
      console.log('cam');
      this.$socket.emit('back_robotview_front', 'cam');
    },
    imageLoad() {
      setTimeout(() => {
        this.time = new Date().getTime();
        // console.log(this.time)
      }, 50);
    },
    imageError() {
      setTimeout(() => {
        this.time = new Date().getTime();
        // console.log(this.time)
      }, 50);
    },
  },
  computed: {
    imageSrc() {
      const fileName = 'cam.jpg';
      const query = '?t=';
      // console.log(fileName + query + this.time)
      return fileName + query + this.time;
    },
  },
};
</script>

<style></style>
