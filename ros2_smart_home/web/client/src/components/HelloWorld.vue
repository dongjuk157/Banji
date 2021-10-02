<template>
  <v-row class="pa-3">
    <!-- 로직 2. 침입자 인지 시점 이미지 뷰어 생성 -->
    <!-- <v-col cols="6">
      <v-img :src="imageSrc" width="480" @load="imageLoad" @error="imageError"></v-img>
    </v-col> -->
    <v-col cols="6">
      <!-- 로직 4. 수동 컨트롤 버튼 생성 -->
      <p>Manual Controller</p>
      <Move />
    </v-col>
  </v-row>
</template>

<script>
import Move from './Move.vue';

export default {
  components: {
    Move,
  },
  data() {
    return {
      time: new Date().getTime(),
    };
  },
  mounted() {
    this.$socket.on('sendAirConditionerToWeb', (message) => {
      console.log('sendAirConditionerToWeb', message);
    });
  },
  methods: {
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
