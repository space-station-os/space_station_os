/** @type {import('tailwindcss').Config} */
export default {
  darkMode: "class",
  content: ["./index.html", "./src/**/*.{js,jsx,ts,tsx}"],
  theme: {
    extend: {
      colors: {
        primary: "#1173d4",
        "background-light": "#f6f7f8",
        "background-dark": "#101922",
        "card-light": "#FFFFFF",
        "card-dark": "#161B22",
        "text-light": "#1F2328",
        "text-dark": "#C9D1D9",
        "border-light": "#D0D7DE",
        "border-dark": "#30363D",
      },
      fontFamily: {
        display: ["Manrope", "sans-serif"],
      },
    },
  },
  plugins: [],
};
