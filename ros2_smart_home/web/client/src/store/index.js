import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default new Vuex.Store({
  state: {
    isAlert: false,
  },
  mutations: {
    alertChange(state, logic) {
      // logic: true or false
      if (logic === true || logic === false) {
        state.isAlert = logic;
      }
    },
  },
  actions: {
    receiveIntruder({ commit }) {
      commit('alertChange', true);
    },
    viewIntruder({ commit }) {
      commit('alertChange', false);
    },
  },
  getters: {
    alertCheck(state) {
      console.log(state.isAlert);
      return state.isAlert;
    },
  },
  modules: {
  },
});
