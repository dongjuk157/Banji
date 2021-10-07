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
    path: '/oauth/callback/kakao',
    name: 'CallBack',
    component: () => import('../components/CallBack.vue'),
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
    path: '/map',
    name: 'Map',
    component: () => import('../components/CCTV.vue'),
  },
  {
    path: '/controls',
    name: 'Control',
    component: () => import('../components/Control.vue'),
  },
  {
    path: '/capture',
    name: 'Capture',
    component: () => import('../components/ImageViewer.vue'),
  },
  {
    path: '/intruder',
    name: 'Intruder',
    component: () => import('../components/IntruderList.vue'),
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
