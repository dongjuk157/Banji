<template>
  <div class="about">
    <v-img src="../assets/mantoman.jpg"></v-img>
    <h6 style="margin: 10px auto;">mady by mantoman</h6>
    <p class="made-team">고동건 양준서 이광교 이진영 정대진 천주용</p>
    <v-btn @click="logout">
      Logout
    </v-btn>
  </div>
</template>
<script>
import HomeVue from './Home.vue';

export default {
  methods: {

    Logout() {
      localStorage.removeItem('isLogin');
      this.$router.push(HomeVue);
      window.location.reload();
    },
  },
};
</script>

<style>
  .made-team {
    font-size: 00.5rem;
    color: rgb(240, 240, 229);
  }
</style>
