<template>
  <div class="home">
    <MainPage v-if="$store.state.isLogin" />
    <LoginPage v-else />
  </div>
</template>

<script>
// @ is an alias to /src
import MainPage from '../components/MainPage.vue';
import LoginPage from '../components/LoginPage.vue';

export default {
  name: 'Home',
  components: {
    MainPage,
    LoginPage,
  },
  created() {
    this.$store.commit('updateLoginState');
  },
};
</script>
