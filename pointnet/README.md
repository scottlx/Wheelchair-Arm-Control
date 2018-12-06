# Point Cloud Recognition Part

Basiclly, do the things below:
0. make sure your rostopic are publishing point cloud data
1. fetch the point cloud data from rostopic and decoding
2. do some preprocess to the point cloud data and transform it to numpy array
3. feed the numpy array point cloud data to pretrained pointnet
4. the user specify one class out of 12 to concerning on
5. the program return the estimated location of the object belong to the class,and the std


## Requirements
[Ubuntu 18.04.1 LTS](http://releases.ubuntu.com/18.04)  
[ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)  
[gazebo_ros_pkgs](http://wiki.ros.org/gazebo_ros_pkgs)
[python 2.7](https://www.python.org/download/releases/2.7/)
[tensorflow 1.0](https://www.tensorflow.org/api_guides/python/upgrade)



## file explained
```
subscriber.py
```
fetching point cloud data from rostopic

```
get_pointcloud.py
```
a wrapper for subscriber, it will reshape the numpy array from subscriber,
change it to [batch,points,row]

```
inference.py
```
tensorflow running here, it will get numpy array point cloud from get_pointcloud,
and then doing infernce, when you call evaluate() it will return location and std
of the object you are interested in

```
class_names.txt tf_utils.py
```
these are just utils files

```
recognize.py
```
example usage, please see details in the file
you can customize offset for kinect camera, currently only support transition

## mesh

all the files below are in global coordination

```
coke_object.obj
```
this is object (for example coke can) points picked out from scene

```
scene.obj
```
the remain points(except coke can)

```
scene.stl
```
stl file of scene.ojb converted by meshlab
