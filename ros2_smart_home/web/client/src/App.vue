<template>
  <v-app id="app">
    <v-container v-if="loginCheck">
      <div id="nav">
        <router-link to="/">Home</router-link> |
        <router-link to="/about">반지</router-link>
      </div>
      <!-- <button @click="intruder">test</button> -->
      <div
        id="intruderAlert"
        v-if="alertCheck"
        @click="checkIntruder"
      >
        <v-icon
          color="red"
        >mdi-alert-circle</v-icon>
      </div>
      <router-view/>
    </v-container>
    <v-container v-else-if="kakaoLoginStep">
      <CallBack />
    </v-container>
    <v-container v-else>
      <LoginPage/>
    </v-container>
  </v-app>
</template>

<script>
import { mapGetters } from 'vuex';
import CallBack from './components/CallBack.vue';
import LoginPage from './components/LoginPage.vue';

export default {
  components: {
    LoginPage,
    CallBack,
  },
  created() {
    this.$store.commit('updateLoginState');
  },
  mounted() {
    this.$socket.on('front_alert_back', (message) => {
      // console.log(message);
      // console.log(window.location.pathname.includes('intruder'));
      if (window.location.pathname.includes('intruder')) {
        return;
      }
      this.$store.dispatch('receiveIntruder', message);
      // console.log(message);
    });
  },
  methods: {
    intruder() {
      this.$socket.emit('back_alert_robot', 'test');
    },
    checkIntruder() {
      this.$store.dispatch('viewIntruder');
      this.$router.push({ path: '/intruder' });
    },
  },
  computed: {
    ...mapGetters([
      'alertCheck',
      'loginCheck',
    ]),
    kakaoLoginStep() {
      const code = new URL(window.location.href).searchParams.get('code');
      const step = !!code;
      // console.log(step);
      return (!this.$store.state.isLogin) && step;
    },
  },
};
</script>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}

#nav {
  padding: 30px;
}

#nav a {
  font-weight: bold;
  color: #2c3e50;
}

#nav a.router-link-exact-active {
  color: #42b983;
}

#intruderAlert {
  cursor: pointer;
  position: absolute;
  right: 1rem;
  top: 1rem;
}

</style>
