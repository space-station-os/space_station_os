import { createRouter, createWebHistory } from "vue-router";
import GncControl from "../views/GncControl.vue";

const routes = [
  { path: "/", name: "GncControl", component: GncControl },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
