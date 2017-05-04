# skimap_ros
This package contains the **SkiMap Mapping Framework** described here:

> DE GREGORIO, Daniele; DI STEFANO, Luigi. **SkiMap: An Efficient Mapping Framework for Robot Navigation**. In: *Robotics and Automation (ICRA), 2017 IEEE International Conference on*. [**PDF**](https://arxiv.org/abs/1704.05832)

Watch the Video:

<a href="https://www.youtube.com/watch?v=MverWmFAgkg" target="_blank"><img src="https://img.youtube.com/vi/MverWmFAgkg/0.jpg" 
alt="paper video" width="240" height="180" border="10" /></a>



The framework si wrapped in a ROS package to maximize portability but, since it is an *Header-Only* library,
can be easily used elsewhere. This package contains also an implementation of **SlamDunk** algorithm described
in the last section used to track camera 6-DOF pose.

This package is equipped with a sample BAG file: 
[tiago_lar.bag](https://mega.nz/#!QJA2mS6Z!bv67y8nTiQGWT6f5tL05SegwUYxaEAvUuwpLcLc6bSc)
. This bag was collected by means of a mobile robot with an head-mounting
RGB-D camera. So it provides not only RGB-D frames but also a whole TF tree, including Odometry informations. 
For this reason this bag can be used as source for **SkiMap** mapping using Odometry to track Camera pose or as source 
for **SlamDunk+SkiMap** duet where Camera is tracked by Slam System.

### Dependencies

* OpenMP
* Eigen3
* OpenCV (2.4)
* Boost

### LICENSE
You may use, distribute and modify this code under the terms of the GNU GPLv3 license.
If you use *SkiMap* in an academic work, please cite:
```
@inproceedings{degregorio2017skimap,
  title={SkiMap: An Efficient Mapping Framework for Robot Navigation},
  author={De Gregorio, Daniele and Di Stefano, Luigi},
  booktitle={Robotics and Automation (ICRA), 2017 IEEE International Conference on},
  year={2017}
}
```

## SkiMap live mapping: *skimap_live.launch*

If the Camera 6-DOF pose is available (e.g. the camera is in a eye-to-hand configuration..) we can use **SkiMap**
as a classical Mapping Framework. Furthermore, as described in the paper, if the Global Reference Frames of the Camera Poses 
lies on ground then SkiMap is able to perform also a 2D Map of the environment simultaneously with the 3D Map. 
The attached BAG is an useful example to understand the operation of this node because it is collected with a mobile robot 
with an head-mounted RGB-D camera so the Global Reference frame is certainly on the Ground (Odometry is the Global RF). To
launch the example you have to run two separate commands:


```
1) roslaunch skimap_ros slamdunk_tracker.launch
2) rosbag play tiago_lar.bag --clock
```

If you want to run *skimap_live.launch* in your real context you have to change the Camera Topics parameters and the TF parameters
in the launch file:


```
<!-- Camera topic for RGB and Depth images -->
<param name="camera_rgb_topic" value="/$(arg camera)/rgb/image_raw"/>
<param name="camera_depth_topic" value="/$(arg camera)/depth/image_raw"/>


<!-- Global and Camera TF Names -->
<param name="base_frame_name" value="odom"/>
<param name="camera_frame_name" value="xtion_rgb_optical_frame"/>
```

Remember to set as *base_frame_name* the name of the TF representing the World Frame, or the Fixed Frame, in which the
*camera_frame_name* is represented. Should be noted that **SkiMap** is not responsible to produce these frames but it uses
them to build the map, so the accuracy of the reconstruction depends on the accuracy of them.

## SkiMap Service: *skimap_map_service.launch*

A less intrusive manner to use SkiMap is the Map Service launching *skimap_map_service.launch*. This service use *SkimapIntegrationService.srv* as interface to integrate new measurements in the global map. To understand how to create a client for this service you can take a look at *SkiMapServiceClient.hpp* (or you can just use it!). To instantiate our off-the-shelf client just do:

```
skimap_ros::SkimapServiceClient* skimap_service_client = new skimap_ros::SkimapServiceClient(&nh, "/skimap_map_service/integration_service");
```
Remember that the string */skimap_map_service/integration_service* is the name of the service, in this case is the default name used also in the *skimap_map_service.launch* node. Once this client is ready you can just call the *integratePoints* method passing a vector filled with the points representing sensor measurements (in the sensor Reference Frame) and the *geometry_msgs/Pose* representing the 6-DOF pose of the sensor:

```
typedef skimap_ros::SkimapServiceClient::ColorPoint CPoint;
std::vector<CPoint> points;
//Fill vector with your points
geometry_msgs::Pose sensor_pose;
//Fill sensor_pose with your camera pose
skimap_service_client->integratePoints(points, sensor_pose);
```

This example Client is built with an asynchronous queue so the *integratePoints* method just appends your data to the queue, the real communication with SkiMap Map Service will be made in another thread. This technique is useful when your node requires real-time performance.

### SkiMap with ORBSLAM2:

This Video:
<a href="https://www.youtube.com/watch?v=MverWmFAgkg" target="_blank"><img src="https://img.youtube.com/vi/W3nm2LXmgqE/0.jpg" 
alt="paper video" width="240" height="180" border="10" /></a>

shows an integration of the SkiMap (using the abovementioned SkiMap Map Service) with the popular [ORBSLAM2](https://github.com/raulmur/ORB_SLAM2) framework.


## SlamDunk and SkiMap: *slamdunk_tracker.launch*

*Important*: in order to activate this feature enable the following build option:

```
option(BUILD_SLAMDUNK "Build Slamdunk Library" OFF)
```


In this node is implemented a Wrapper of the popular **SlamDunk** localization and mapping algorithm, described here:

> FIORAIO, Nicola; DI STEFANO, Luigi. **SlamDunk: affordable real-time RGB-D SLAM**. In: *Workshop at the European Conference on Computer Vision. Springer International Publishing*, 2014. p. 401-414. [**PDF**](http://ai2-s2-pdfs.s3.amazonaws.com/7e9e/191c127144b61d5d5cabac37bbbc27fe7697.pdf)

This is a real-time solution to RGB-D SLAM problem. It enables metric loop closure seamlessly and preserves local consistency by means
of relative bundle adjustment principles. In the first paper *degregorio2017skimap* we described a combination of both systems.
Also in this use case we can exploit the attached bag file: in this case however we will benefit only from the RGB-D Frames
provided in the Bag file because the **SlamDunk** algorithm will provide us a consistent 6-DOF Camera Pose to be used as source 
to build the map of the environment. For proper use of **SkiMap** also here is better to build the Global Reference Frame in
such a way as it lies on the ground; for this reason the camera must get a shot of the ground plane in the very first frame, 
as occurs in the example bag. To launch the complete example you have to run two separate commands:

```
1) roslaunch skimap_ros slamdunk_tracker.launch
2) rosbag play tiago_lar.bag --clock
```

Also here , to adapt this node to your real context, pay attention to the two Camera Topics in the launch file:

```
<!-- Camera topic for RGB and Depth images -->
<param name="camera_rgb_topic" value="/$(arg camera)/rgb/image_raw"/>
<param name="camera_depth_topic" value="/$(arg camera)/depth/image_raw"/>
```

Here you can also disable *SkiMap*:

```
<param name="mapping" value="true"/>
```

Using only the *camera tracking* feature offered by *SlamDunk*. The latter will publish computed camera 6-DOF Pose in a TF tree 
with these two names:

```
<param name="base_frame_name" value="slam_map"/>
<param name="camera_frame_name" value="camera"/>
```
Where *base_frame_name* will be the Global Reference Frame of the Slam system (e.g. usually centered in the first camera pose).


### LICENSE
If you use *SlamDunk* in an academic work, please cite:

```
@inproceedings{fioraio2014slamdunk,
  title={SlamDunk: affordable real-time RGB-D SLAM},
  author={Fioraio, Nicola and Di Stefano, Luigi},
  booktitle={Workshop at the European Conference on Computer Vision},
  pages={401--414},
  year={2014},
  organization={Springer}
}
```


