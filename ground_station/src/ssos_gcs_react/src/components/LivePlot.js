import React, { useEffect, useState } from "react";

function LivePlot() {
  const [timestamp, setTimestamp] = useState(Date.now());

  useEffect(() => {
    const intervalId = setInterval(() => {
      setTimestamp(Date.now());
    }, 2000);
    return () => clearInterval(intervalId);
  }, []);

  // Append a query param for cache-busting
  const plotUrl = `http://localhost:8000/iss_plot.png?t=${timestamp}`;

  return (
    <div>
      <h2>Space Station Live Plot</h2>
      <img
        src={plotUrl}
        alt="ISS Plot"
        style={{ maxWidth: "50%", border: "0.5px solid #ccc" }}
      />
    </div>
  );
}

export default LivePlot;
