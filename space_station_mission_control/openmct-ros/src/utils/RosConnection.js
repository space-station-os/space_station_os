import ROSLIB from 'roslib';

export default class RosConnection {
    constructor(url) {
        this.url = url;
        this.rosConnectionPromise = null;
    }

    getConnection() {
        if (!this.rosConnectionPromise) {
            this.rosConnectionPromise = this.#connectToRos();
        }

        return this.rosConnectionPromise;
    }

    #connectToRos() {
        console.debug(`🤖🤖🤖🤖 Connecting to ROS at ${this.url} 🤖🤖🤖`);
        const ros = new ROSLIB.Ros();

        return new Promise((resolve, reject) => {
            ros.connect(this.url);
            ros.on('connection', () => {
                console.debug(`🤖🤖🤖🤖 Connected to ROS 🤖🤖🤖🤖`);
                resolve(ros);
            });
            ros.on('error', (error) => {
                console.error(`🚨 Error connecting to ROS. Ensure rosbridge_server is running on ${this.url} 🚨`, error);
                reject(error);
            });
        });
    }
}
