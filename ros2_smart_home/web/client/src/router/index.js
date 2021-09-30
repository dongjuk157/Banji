import Vue from 'vue';
import VueRouter from 'vue-router';
import Home from '../views/Home.vue';
import RobotStatus from '../components/RobotStatus.vue';

Vue.use(VueRouter);

const routes = [
  {
    path: '/test',
    name: 'Test',
    component: () => import('../components/HelloWorld.vue'),
  },
  {
    path: '/',
    name: 'Home',
    component: Home,
  },
  {
    path: '/about',
    name: 'About',
    // route level code-splitting
    // this generates a separate chunk (about.[hash].js) for this route
    // which is lazy-loaded when the route is visited.
    component: () => import(/* webpackChunkName: "about" */ '../views/About.vue'),
  },
  {
    path: '/schedule',
    name: 'Schedule',
    component: () => import('../components/Scheduler.vue'),
  },
  {
    path: '/notfound',
    name: 'NotFound',
    component: () => import('../components/NotFound.vue'),
  },
  {
    path: '/settings/robot',
    name: 'RobotSetting',
    component: RobotStatus,
  },
  {
    path: '*',
    redirect: '/notfound',
  },
];

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes,
});

export default router;
