import Vue from 'vue';
import { BootstrapVue, IconsPlugin } from 'bootstrap-vue';
import io from 'socket.io-client';

import App from './App.vue';
import router from './router';
import store from './store';

// Import Bootstrap an BootstrapVue CSS files (order is important)
import 'bootstrap/dist/css/bootstrap.css';
import 'bootstrap-vue/dist/bootstrap-vue.css';
import vuetify from './plugins/vuetify';
import 'roboto-fontface/css/roboto/roboto-fontface.css';
import '@mdi/font/css/materialdesignicons.css';

// Make BootstrapVue available throughout your project
Vue.use(BootstrapVue);

// Optionally install the BootstrapVue icon components plugin
Vue.use(IconsPlugin);

// socket.io
// const socket = io('http://localhost:12001');
// const socket = io('http://192.168.0.11:12001', { transports: ['websocket'] });
const socket = io('https://j5b301.p.ssafy.io:12001', { transport: ['websocket'] });

Vue.prototype.$socket = socket;

Vue.config.productionTip = false;

new Vue({
  router,
  store,
  vuetify,
  render: (h) => h(App),
}).$mount('#app');
