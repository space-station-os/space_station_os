import React, { useState } from 'react';

function CommandForm() {
  const [commandName, setCommandName] = useState("");
  const [arg1, setArg1] = useState("");
  const [arg2, setArg2] = useState("");
  const [status, setStatus] = useState(null);

  const sendCommand = async (e) => {
    e.preventDefault();
    try {
      // Adjust instance, processor, and arguments as needed
      const url = `http://localhost:8090/api/processors/myproject/realtime/commands/${commandName}`;
      
      const payload = {
        // If your command requires named arguments
        args: {
          arg1: arg1,
          arg2: arg2
        }
      };

      const response = await fetch(url, {
        method: "POST",
        headers: { 
          "Content-Type": "application/json"
        },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setStatus(`Command sent successfully: ${JSON.stringify(data)}`);
    } catch (err) {
      setStatus(`Error sending command: ${err.message}`);
    }
  };

  return (
    <div>
      <h2>Send User Command</h2>
      <form onSubmit={sendCommand}>
        <label>
          Target:
          <input 
            type="text" 
            value={commandName}
            onChange={(e) => setCommandName(e.target.value)}
          />
        </label>
        <br />
        <label>
          Command:
          <input
            type="text"
            value={arg1}
            onChange={(e) => setArg1(e.target.value)}
          />
        </label>
        <br />
        <label>
          Comment:
          <input
            type="text"
            value={arg2}
            onChange={(e) => setArg2(e.target.value)}
          />
        </label>
        <br />
        <button type="submit">Send Command</button>
      </form>

      {status && (
        <p style={{ color: "blue" }}>{status}</p>
      )}
    </div>
  );
}

export default CommandForm;
