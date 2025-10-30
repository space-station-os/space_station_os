import ROSLIB from "roslib";

let ros = null;
let reconnectTimer = null;
let connected = false;

/**
 * Connect to ROSBridge WebSocket
 * @param {string} url - Default ws://localhost:9090
 * @returns {ROSLIB.Ros}
 */
export function connectToRosBridge(url = "ws://localhost:9090") {
  if (ros && connected) return ros;

  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    connected = true;
    console.log("✅ Connected to ROS 2 via rosbridge:", url);
    if (reconnectTimer) {
      clearTimeout(reconnectTimer);
      reconnectTimer = null;
    }
  });

  ros.on("error", (err) => {
    console.error("❌ ROSBridge error:", err);
    connected = false;
  });

  ros.on("close", () => {
    console.warn("⚠️ ROSBridge connection closed — retrying in 3 s");
    connected = false;
    if (!reconnectTimer) {
      reconnectTimer = setTimeout(() => connectToRosBridge(url), 3000);
    }
  });

  return ros;
}

/**
 * Call a ROS 2 service via rosbridge
 * @param {string} serviceName - e.g. "/gnc/set_mode"
 * @param {object} request - request object fields
 * @param {string} [serviceType="std_srvs/srv/Trigger"] - ROS service type
 * @returns {Promise<object>}
 */
export function callService(serviceName, request = {}, serviceType = "std_srvs/srv/Trigger") {
  const rosInstance = connectToRosBridge();
  const service = new ROSLIB.Service({
    ros: rosInstance,
    name: serviceName,
    serviceType,
  });

  const req = new ROSLIB.ServiceRequest(request);

  return new Promise((resolve, reject) => {
    service.callService(
      req,
      (res) => {
        console.log(`🛰️ [Service] ${serviceName} →`, res);
        resolve(res);
      },
      (err) => {
        console.error(`❌ [Service] ${serviceName} failed:`, err);
        reject(err);
      }
    );
  });
}

/**
 * Subscribe to a topic and receive messages continuously
 * @param {string} topicName - e.g. "/gnc/control_mode"
 * @param {string} messageType - e.g. "std_msgs/String"
 * @param {(msg: any) => void} callback - message handler
 * @returns {ROSLIB.Topic}
 */
export function subscribeTopic(topicName, messageType, callback) {
  const rosInstance = connectToRosBridge();
  const topic = new ROSLIB.Topic({
    ros: rosInstance,
    name: topicName,
    messageType,
    throttle_rate: 200, // limit callback rate (ms)
  });

  topic.subscribe((msg) => {
    console.debug(`📡 [Topic] ${topicName} →`, msg);
    callback(msg);
  });

  return topic;
}

/**
 * Publish a one-off message to a topic
 * @param {string} topicName
 * @param {string} messageType
 * @param {object} data
 */
export function publishTopic(topicName, messageType, data) {
  const rosInstance = connectToRosBridge();
  const topic = new ROSLIB.Topic({
    ros: rosInstance,
    name: topicName,
    messageType,
  });

  const msg = new ROSLIB.Message(data);
  topic.publish(msg);
  console.log(`🚀 [Publish] ${topicName} ←`, data);
}

/**
 * Get connection state
 * @returns {boolean}
 */
export function isConnected() {
  return connected;
}
