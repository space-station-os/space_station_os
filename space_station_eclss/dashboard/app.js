const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'  // Ensure rosbridge_websocket is running on this port
});

ros.on('connection', function () {
  console.log('✅ Connected to rosbridge websocket server');
});

ros.on('error', function (error) {
  console.error('❌ Error connecting to rosbridge:', error);
});

ros.on('close', function () {
  console.warn('⚠️ Connection to rosbridge closed.');
});

const airDataListener = new ROSLIB.Topic({
  ros: ros,
  name: '/collector_air_quality',
  messageType: 'demo_nova_sanctum/msg/AirData'
});

airDataListener.subscribe(function (message) {
  const totalMass = message.co2_mass + message.moisture_content + message.contaminants;
  updateTank(totalMass);
});

function updateTank(value) {
  const maxTankCapacity = 1000.0; // g
  const fillPercent = Math.min((value / maxTankCapacity) * 100, 100);

  const fillElem = document.getElementById('tank-fill');
  const labelElem = document.getElementById('tank-label');

  fillElem.style.height = `${fillPercent}%`;
  labelElem.textContent = `${value.toFixed(2)} g`;
  listener.subscribe(function(message) {
    console.log("CO₂ mass:", message.co2_mass); 
    
  });
  
}
