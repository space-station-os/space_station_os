<template>
  <nav class="navbar">
    <div class="navbar-left">
      <span class="logo">🛰️ Space Station Control</span>
    </div>

    <ul class="navbar-links">
      <li><router-link exact to="/">Overview</router-link></li>
      <li><router-link to="/ars">Air</router-link></li>
      <li><router-link to="/wrs">Water</router-link></li>
      <li><router-link to="/ogs">Oxygen</router-link></li>
      <li><router-link to="/crew">Crew</router-link></li>
    </ul>

    <div class="navbar-right">
      <img class="profile-pic" src="/assets/avatar.jpg" alt="Profile" />
    </div>
  </nav>
</template>

<script>
export default {
  name: "NavBar",
};
</script>

<style scoped>
.navbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  background-color: #161b22;
  padding: 16px 32px;
  border-bottom: 1px solid #30363d;
  color: white;
}

.logo {
  font-size: 1.2rem;
  font-weight: 600;
  color: #c9d1d9;
}

.navbar-links {
  list-style: none;
  display: flex;
  gap: 24px;
  padding: 0;
  margin: 0;
}

.navbar-links li a {
  text-decoration: none;
  color: #c9d1d9;
  font-weight: 500;
  transition: color 0.2s ease;
}

.navbar-links li a:hover {
  color: #58a6ff;
}

.navbar-links li a.router-link-exact-active {
  color: #58a6ff;
}

.profile-pic {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  object-fit: cover;
  border: 2px solid #30363d;
}
</style>
