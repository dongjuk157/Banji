<template>
  <div>
  </div>
</template>

<script>
import HomeVue from '../views/Home.vue';
// import axios from 'axios';

export default {
  name: 'CallBack',
  created() {
    const code = new URL(window.location.href).searchParams.get('code');
    if (code) {
      this.$store.commit('updateLoginState', true);
    }
    this.$router.push(HomeVue);
    // axios.post()
  },
};
</script>

<style>

</style>
