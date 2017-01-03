# Master in Computer Vision - UE4 - Robotics Project

![N|Solid](https://media.licdn.com/mpr/mpr/shrink_200_200/AAEAAQAAAAAAAAIEAAAAJGUxMGNmYWYxLTUxMGYtNDU5Zi05ZDE2LTZjNDZkMzgwYjE2Mw.png)

**Group2 :**
 ```html
<KATRINI  Chrysanthi
```
```html
<CHARALAMPAKI Eirini
 ```
 ```html
<CASPANI Bastien
 ``` 

## The project scenario

> The scenario of our project is that Tbot the turtlebot is arriving on a foreign environement and need to move automosly and identify tools.
So in practice the robot will you slam and detect 3D geometric forms as sphere a cube
...
 
## Installation and requiered packages
Please copy the content of the folder packages into  your workspace.
Please copy the content of the folder worlds (file from gazebo) to **opt/ros/indigo/share/turtlebot_gazebo/world/**
Please copy the content of the folder rviz (config file forRVIZ) to **opt/ros/indigo/share/turtlebot_rviz_launcher/rviz/**

## Build the project 

Our project is design to use **catkin** compiler.
So when you are on the root of your catkin workspace  (catkin_ws)
run **catkin_make**  then **rospack profiles**

## Run Launch files 
The main launch file is **bce_7.launch** 
To start it use the commande  **roslaunch my_pcl_tutorial bce_7.launch**
To Run the simulated detection use the commande ** rosrun sim_detect sim_detect_node**

## Link of videos from the project
On Gazebo:
https://www.youtube.com/watch?v=Wah2mYxG0Ig&t=267s  
On the Turtlebot:
https://www.youtube.com/watch?v=WO2-XAnb5Ks
https://www.youtube.com/watch?v=YnviLrt-0LM


## References used

1. http://learn.turtlebot.com/. [Online] http://learn.turtlebot.com/.
2. http://wg-perception.github.io. [Online] http://wg-perception.github.io/tabletop/.
3. http://wiki.ros.org. [Online] http://wiki.ros.org/tabletop_object_detector.
4. http://wg-perception.github.io/object_recognition_core/index.html
5.  http://www.pointclouds.org/
6.  http://pcl.ros.org
7. https://github.com/ros-perception/perception_pcl
8. http://wiki.ros.org/ROS/Tutorials


 Ms CV- 2016 - University of Burgundy
 
