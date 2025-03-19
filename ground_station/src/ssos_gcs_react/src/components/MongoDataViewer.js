import React, { useEffect, useState } from 'react';
import axios from 'axios';
import { LineChart, Line, XAxis, YAxis, Tooltip, CartesianGrid, Legend } from 'recharts';

function MongoDataViewer() {
  const [data, setData] = useState([]);

  useEffect(() => {
    const fetchData = () => {
      axios.get('http://localhost:5000/api/mongo/latest')
        .then(response => {
          setData(response.data);
        })
        .catch(error => console.error('Error fetching data:', error));
    };

    fetchData();
    const intervalId = setInterval(fetchData, 2000);

    return () => clearInterval(intervalId);
  }, []);

  return (
    <div className="m-4 p-4 border rounded shadow">
      <h2 className="text-xl font-bold mb-4">MongoDB Live Data Visualization</h2>
      <LineChart width={600} height={250} data={data}>
        <CartesianGrid strokeDasharray="3 3" />
        <XAxis dataKey="timestamp" tickFormatter={(time) => new Date(time * 1000).toLocaleTimeString()} />
        <YAxis />
        <Tooltip labelFormatter={(time) => new Date(time * 1000).toLocaleTimeString()} />
        <Legend />
        <Line type="monotone" dataKey="angular_velocity.x" name="Angular Vel X" stroke="#8884d8" />
        <Line type="monotone" dataKey="angular_velocity.y" name="Angular Vel Y" stroke="#82ca9d" />
        <Line type="monotone" dataKey="angular_velocity.z" name="Angular Vel Z" stroke="#ffc658" />
      </LineChart>
    </div>
  );
}

export default MongoDataViewer;