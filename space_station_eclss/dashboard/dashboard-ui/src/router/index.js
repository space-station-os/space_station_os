import { createRouter, createWebHistory } from "vue-router";
import HomePage from "../pages/Home.vue";
import Ars from "../pages/Ars.vue";
import Oxygen from "../pages/Oxygen.vue";
import Water from "../pages/Waterrecovery.vue";

const routes = [
  {
    path: "/",
    name: "Home",
    component: HomePage,
  },
  {
    path: "/ars",
    name: "ARS",
    component: Ars,
  },
  {
    path: "/ogs",
    name: "OGS",
    component: Oxygen,
  },
  {
    path: "/water",
    name: "WRS",
    component: Water,
  },
  {
    path: "/:catchAll(.*)",
    redirect: "/",
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
