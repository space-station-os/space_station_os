import React, { useEffect, useState } from "react";

function YamcsDataPanel() {
  const [items, setItems] = useState([]);
  const [error, setError] = useState(null);

  useEffect(() => {
    fetchParams();
  }, []);

  async function fetchParams() {
    // Replace with your actual endpoint if you need different query params:
    const url = "http://localhost:8090/api/mdb/ssos_gcs_yamcs/parameters";

    try {
      console.log("Fetching from:", url);
      const response = await fetch(url);
      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`);
      }
      const data = await response.json();
      console.log("Response data:", data);

      // If the endpoint returns an object like { parameters: [ ... ] }, then:
      setItems(data.parameters || []);
    } catch (err) {
      console.error("Fetch error:", err);
      setError(err.message);
    }
  }

  return (
    <div>
      <h2>Space Station Telemetry (YAMCS) </h2>
      {error && <p style={{ color: "red" }}>Error: {error}</p>}

      {items.length === 0 ? (
        <p>No telemetry data found.</p>
      ) : (
        <ul>
          {items.map((param, i) => (
            <li key={i}>{param.name}</li>
          ))}
        </ul>
      )}
    </div>
  );
}

export default YamcsDataPanel;
