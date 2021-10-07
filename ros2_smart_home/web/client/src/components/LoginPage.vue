<template>
  <b-container>
    <b-row>
      <b-col class="m-3">
    <img class="dog" alt="dog logo" src="../assets/dog.jpg" />
     </b-col>
    </b-row>
    <b-row>
      <b-col>
        <a :href="kakaoLoginLink" alt="kakao login">
          <img alt="kakao logo" src="../assets/kakao_login_medium_wide.png" />
        </a>
      </b-col>
    </b-row>
  </b-container>
</template>

<script>
export default {
  name: 'LoginPage',
  data: () => ({
    client_id: '5ba6d91bd5b746ea331fa2f4a67e44f7',
    redirect_uri: 'http://localhost:8080/oauth/callback/kakao',
  }),
  computed: {
    kakaoLoginLink() {
      return `https://kauth.kakao.com/oauth/authorize?client_id=${this.client_id}&redirect_uri=${this.redirect_uri}&response_type=code`;
    },
  },
};
</script>

<style>
</style>
