
# Space Station Power Generation Demo

## Command and published output
```
$ cd space_station_os_dev/src/space_station_power_demo
$ colcon build
$ source install/setup.bash
$ ros2 run space_station_power_demo space_station_power_demo
```
Then the ROS node "space_station_power_demo" publishes two messages, generated power [W] and battery level [Wh] (or state of charge, SoC). They can be shown by rqt_plot.
If you plot them, graphs are like this:
![image](https://github.com/user-attachments/assets/6fdb9c3b-9d36-4d80-9cfc-9ae2ba2378f5)

## Simulation Overview


## Parameters
Space station parameters:
- ss_altitude: altitude of space station orbit.
- ss_raan: RAAN of space station orbit.
- ss_inclination: inclination of space station orbit.
- ss_init_euler_angle: atitude of space station as Euler angle.
- ss_init_w_vec: angular velocity of space station.

Simulation parameters:
- simu_timestep: timestep of simulation [s]
- simu_speed_rate: rate of simulation

These parameters can be set by like
```
ros2 run space_station_power_demo space_station_power_demo --ros-args -p simu_timestep:="2.0"
```
