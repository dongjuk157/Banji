<template>
  <div>
  </div>
</template>

<script>
// import axios from 'axios';
import HomeVue from '../views/Home.vue';

export default {
  name: 'CallBack',
  created() {
    const code = new URL(window.location.href).searchParams.get('code');
    if (code) {
      this.$store.commit('updateLoginState');
      localStorage.setItem('isLogin', true);
    }
    this.$router.push(HomeVue);
  },
};
</script>

<style>

</style>
