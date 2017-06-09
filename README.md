# UAV_demo
demo repo for running launch files

Steps to run uav flight tests

On basestation computer

1. Start Vicon camera server and run vicon tracker
2. Run motion capture: ```roslaunch uav_demo base_station.launch```
3. Run mission window: ```roslaunch uav_demo odroid.launch```

On uav computer
from host ssh login:```ssh <uav_name>```
1. Run controller node: ```roslaunch uav_demo moca.launch```


Common issues: Make sure ROS_MASTER_URI is set to base station and also ROS_IP for the basestation
- gain tuning ```rosrun rqt_reconfigure rqt_reconfigure```
- plot ```rosrun rqt_plot rqt_plot```
- Visualization ```rosrun rviz rviz```

Missions:

- Take off
- Land
- Point to ping
- Hover
- etc.
