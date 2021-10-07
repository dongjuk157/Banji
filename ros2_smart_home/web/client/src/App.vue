<template>
  <v-app id="app">
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
  </v-app>
</template>

<script>
import { mapGetters } from 'vuex';

export default {
  mounted() {
    this.$socket.on('front_alert_back', (message) => {
      console.log(message);
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
    ]),
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
