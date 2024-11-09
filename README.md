# Space Station OS source  

## Prerequisites  
- Ubuntu 22.04
- ROS 2 Humble (desktop install)

### Common procedures for preparing Space Station OS  
- Install Ubuntu 22.04
- Install ROS 2 Humble (desktop install)
- Clone Space Station OS source  
  ...
- Compile Space Station OS source  
  ...

## Demo1  
Demo1 focuses on space station GNC features.  

In Demo 1a, we estimate what happened and how the system and people reacted in the Nauka incident that occurred on the ISS in July 2021.  
https://www.nasaspaceflight.com/2021/07/nauka-docking/  
In Demo 1b, we prepared for an imaginary difficult case.  

Through these cases, we will study the importance of determining FDIR (Fault Detection, Isolation and Recovery).

### Procedures to run Demo 1a/1b  
- Terminal 1:  
for Demo 1a  
$ ros2 run space_station_gnc demo1a_nauka_incident_estimatate  
for Demo 1b  
$ ros2 run space_station_gnc demo1b_crisis_mainengine  
Terminal 1 handles scenario description and user input.  

- Terminal 2:  
(common for Demo 1a/1b)  
$ ros2 launch space_station_gnc launch_gnc.py  
Terminal 2 handles space station propulsion system and attitude control dynamics, where the user (you) do not need to touch thereafter.  

- Terminal 3:  
(common for Demo 1a/1b)  
$ ros2 run rviz2 rviz2  
This will start up RVIZ, a common tool for ROS 2.  
->Choose "Fixed Frame" change from "map" to "world"    
->"Add" -> "RobotModel"  
->Choose "Description Topic" change (empty) to "/robot_description"  
This will result loading ISS model to rviz (while you need to zoom out.)  

Then you go back to Terminal 1 and follow the scenario in the console as the attitude of ISS is displayed in RVIZ2.  

## TODOs  
...

