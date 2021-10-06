<template>
  <b-container>
    <b-row>
      <b-col cols="12" class="btn-main-page" @click="moveAddress">
        로봇 설정
      </b-col>
      <b-col cols="12" class="btn-main-page" @click="moveAddress">
        지도
      </b-col>
      <b-col cols="12" class="btn-main-page" @click="moveAddress">
        스케줄
      </b-col>
      <b-col cols="12" class="btn-main-page" @click="moveAddress">
        가전 제어
      </b-col>
      <b-col cols="12" class="btn-main-page" @click="moveAddress">
        나의 반려동물
      </b-col>
      <b-col cols="12" class="btn-main-page" @click="moveAddress">
        침입자 확인
      </b-col>
    </b-row>
  </b-container>
</template>

<script>
export default {
  name: 'Home',
  components: {
  },
  methods: {
    moveAddress(event) {
      const { innerText } = event.target;
      switch (innerText) {
        case '로봇 설정':
          this.$router.push({ path: '/settings/robot' });
          break;
        case '지도':
          this.$router.push({ path: '/map' });
          break;
        case '스케줄':
          this.$router.push({ path: '/schedule' });
          break;
        case '가전 제어':
          this.$router.push({ path: '/controls' });
          break;
        case '나의 반려동물':
          this.$router.push({ path: '/capture' });
          break;
        case '침입자 확인':
          this.$router.push({ path: '/intruder' });
          break;
        default:
      }
    },
  },
};

</script>

<style>
  .btn-main-page {
    padding:0.5rem 1rem 0.5rem 1rem;
    background-color: grey;
    background-clip: content-box;
    color: white;
    height: 4rem;
    display: flex;
    justify-content: center;
    align-items: center;
    cursor: pointer;
  }
</style>
