---
title: 'ROS launch file writing tutorial'
date: 2024-11-6
permalink: /posts/2012/08/blog-post-4/
tags:
  - ROS launch
  - category1
  - category2
---

# Writing ROSlaunch file is inevitable always.I am here to briefly desrible some important elements in ROS launch file.

# Writing ROSlaunch file is inevitable always.I am here to briefly desrible some important elements in ROS launch file.

I post an example of moveit roslaunch file here:

```
<launch>

  <!-- This file makes it easy to include the settings for sensor managers -->  

  <!-- Params for the octomap monitor -->
  <!--  <param name="octomap_frame" type="string" value="some frame in which the robot moves" /> -->
  <param name="octomap_resolution" type="double" value="0.025" />
  <param name="max_range" type="double" value="5.0" />

  <!-- Load the robot specific sensor manager; this sets the moveit_sensor_manager ROS parameter -->
  <arg name="moveit_sensor_manager" default="ur5e" />
  <include file="$(find ur5e_moveit_config)/launch/$(arg moveit_sensor_manager)_moveit_sensor_manager.launch.xml" />
  <rosparam command="load" file="$(find panda_moveit_config)/config/sensors_kinect_pointcloud.yaml" />
</launch>
```

```
<launch>
  <node name="my_robot_node" pkg="my_robot_package" type="my_node" output="screen" />
</launch>

```

1.` <launch>` is the root element. define whole launch file

2. `<param>` used to publish simple parameters to ROS parameter server, like here we publish the value of "octomap_resolution" with type="double" value="0.025" to the ROS parameter server. Then the main programs can automatically subsrible the  octomap_resolution value after starting.

3. `<rosparam> `this can be also seen as publishing parameters to the ROS parameter server. But it publish parameters inside yaml file, this enable writing complex and bunch of parameters inside yaml file. like `sensors_kinect_pointcloud.yaml`

4. `<arg>` is to define a argument which can be used to share at current ROS launch file.

5. `<node>` is to embed and rosrun a ROS .py file or .cpp file that contains a node.  <node name="node_name" pkg="package_name" type="executable_name" output="screen" />. where the name="node_name" is the node name that can define randomly. pkg="package_name" is the package name, tips for finding ros package is `rospack list`. <type> is the cpp or .py file's name.

6. `<include>` is used to include other .launch file in the current .launch file. 

