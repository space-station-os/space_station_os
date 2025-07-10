# ROS Plugin for Open MCT
This project provides a plugin for connecting [Open MCT](https://github.com/nasa/openmct) to [ROS 2](https://docs.ros.org/en/humble/index.html) (and maybe ROS 1).

<img width="1626" alt="Open MCT with ROS" src="https://github.com/traclabs/openmct-ros/assets/9853862/643a0aaa-38de-4369-887f-a14ff3cce382">

# Requirements
This plugin requires rosbridge_suite > [1.3.2](https://github.com/RobotWebTools/rosbridge_suite/releases/tag/1.3.2)

# Installation
Get the latest [Node JS](https://nodejs.org/en/download)

Then run:
```
git clone git@github.com:traclabs/openmct-ros.git
cd openmct-ros
npm install
npm run build:example
npm start
```

This should build the example and run the Open MCT development server. After the server has started, launch a web browser pointing to http://localhost:9097/ 

This will start the Open MCT web application connected to a running `rosbridge_server` on `ws://localhost:9090`.
Note you can change the hostname/IP of the `rosbridge_server` in `example/index.js`.

# Options

#### `rosBridgeEndpoint`

- **Type:** `string`
- **Description:** The WebSocket endpoint for the `rosbridge_server`. This is the URL where the Open MCT plugin will connect to receive ROS data.
- **Example:** `"ws://localhost:9090"`

#### `unsubscribeFromTopicsOnStop`

- **Type:** `boolean`
- **Description:** Determines whether the plugin should automatically unsubscribe from ROS topics when the Open MCT client wants to unsubscribe.
- **Default:** `false`

#### `telemetryDataToKeepPerTopic`

- **Type:** `number`
- **Description:** Specifies the maximum number of telemetry data points to retain in memory for each topic. This helps in managing memory usage and performance by limiting the amount of data stored.
- **Default:** `10000`
- **Example:** Setting this to `5000` would limit the storage to the last 5000 data points per topic.

#### `flattenArraysToSize`

- **Type:** `number`
- **Description:** When receiving array data from a ROS topic, this option limits the array size by only keeping the specified number of elements.
- **Default:** `10`
- **Example:** If a topic publishes an array with 100 elements and this option is set to `10`, only the first 10 elements of each array will be kept.

#### `namespace`

- **Type:** `string`
- **Description:** Specifies a namespace for the ROS topics to be used within Open MCT. This allows for organizing and grouping topics under a specific namespace.
- **Default:** `"ros"`

# Testing plugin

## Prerequisites
* Download [Docker](https://www.docker.com/) in some fashion
* Ensure you've followed the guide to allow your [X11 server](http://wiki.ros.org/docker/Tutorials/GUI) to allow for connection.
    * Note on MacOS, ensure 'Allow connections from network clients' has been set in its preferences

## Running Test
1. Launch `openmct-ros-dev-server`, `rosbridge_server`, `turtlesim`, and [turtlesim_controller](https://github.com/DominikN/ros2_docker_examples) by running:
```
docker compose up
```
2. Wait a moment for everything to initialize. You should see the `turtlesim` window appear and circling.
3. Refresh your browser pointing to http://localhost:9091/ - it should automatically connect and display a list of ROS topics.
