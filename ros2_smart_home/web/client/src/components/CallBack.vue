<template>
  <div>
    <b-spinner></b-spinner>
  </div>
</template>

<script>
export default {
  name: 'CallBack',
  created() {
    const code = new URL(window.location.href).searchParams.get('code');
    if (code) {
      this.$store.commit('updateLoginState');
      localStorage.setItem('isLogin', true);
    }
    this.$router.push({ path: '/' });
    window.location.reload();
  },
};
</script>

<style>

</style>
