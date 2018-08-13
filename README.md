# colab-gsoc2018-ArsalanAkhter
Currently, this repo has four packages to control and interact with the amazon robot.
- basic teleop
- amazon robot teleop
- amazon robot odom demo
- NavigationOmpl

Follow these simple steps to run the practice:
1. Launch Gazebo with roslaunch
```
$ roslaunch /opt/jderobot/share/jderobot/launch/amazon-warehouse.launch  
```

2. Execute the practice's component indicating the configuration file for the map:
```
$  python2 NavigationOmpl.py taxiMap.conf
```
