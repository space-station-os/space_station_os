import { createRouter, createWebHistory } from "vue-router";
import GncControl from "../views/GncControl.vue";

const routes = [{ path: "/", name: "GncControl", component: GncControl }];

export default createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes,
});
