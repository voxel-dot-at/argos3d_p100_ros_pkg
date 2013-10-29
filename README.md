argos_p100_ros_pkg
===================
### ROS package for Bluetechnix Argos P100 ToF camera. ###

# Sumary #

This package explaint how to configurate your system and ROS to use the ToF camera Argos P100.
The package include an example that allows to visualize images using the rviz viewer include in ROS.
It demostrates how to use the camera within ROS and the different parameter configurations of the argos
as well as his possibilities.

## First step: Get Ros ##

The argos_p100_ros_kg works with ROS version groovy and hydro. You can user catkin workspaces or the previous
rosbuild to configurate, compile and get ready ROS.

We will point in the above lines how to get ros_hydro and catkin workspace from the tutorials of the ROS web site.

In Ubuntu:
Follow the ROS installation tutorial: 
>http://wiki.ros.org/hydro/Installation/Ubuntu.

Use catkin workspaces:
>http://wiki.ros.org/catkin 
>
>http://wiki.ros.org/catkin_or_rosbuild
>
>http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To configurate a catkin workspace in your ROS instalation, follow this ROS 
>tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## Known Problems ##

Be sure your libboost library version is >= 1.49.
Previous versions as 1.46 generate error while compiling argos3d_p100-ros-pkg.

# 1. Configuration #

### Setup P100 USB rules and native libraries ###

We should make something similar to what is done on it. 
The first point is to add the PMDSDK to our system. (The PMDSDK is copyrighted software and it is distributed with your Argos3D P100)

#### 1.1 Usb Device rules ####

First browse to the directory of your operating system inside the software files you get with your camera. Install the driver for using the ToF camera:

In Ubuntu/linux
<pre><code>cd driver
sudo cp 10-pmd-ubuntu.rules /usr/udev/rules.d/
</code></pre>
copy the file from the driver folder to **/usr/udev/rules.d/**
This will set the right permissions to allow not sudo users to use the camera.

#### 1.2 Libraries setup ####

Add PMDSDK to your system (use the right version for your system 32 or 64 bits).

1. Add the dynamically linked shared object library libpmdaccess2. Use one of the following options:

- You have different ways to add it to the **LD_LIBRARY_PATH**.
- You can simply copy the libpmdaccess2.so file to **/usr/local/lib** as root (or the defauld search library path of your system)
- Add a .conf file (like pmd_camera_L64.conf) with the path of your library in **/etc/ld.so.conf.d**
<pre><code>#PMD camera support
/.../PMDSDK/bin
/.../PMDSDK/include
</code></pre>
*replace the dots with the absolute path to the PMDSDK directory*

2. Add includes and dynamic plugin libraries

- Move the PMDSDK folder to a directory where the operating system can find it (/usr/include)
- or set an environment variable $PMDDIR to the path where PMDSDK is.

After adding libraries update the links and cache of the share libraries with:
<pre><code>ldconfig
</code></pre>

# 2. Installation #

####  2.1 Install dependencies #### 

Make sure you have de following dependencies already installed:
<pre><code> apt-get install ros-hydro-pcl-ros ros-hydro-pcl-conversions ros-hydro-perception-pcl 
</code></pre>

####  2.2 Install the package ####

Clone from repository: https://github.com/voxel-dot-at/argos3d_p100_ros_pkg.git
to your /src folder in your catkin workspace.
Now compile it with:
<pre><code>cd catkin_ws
source devel/setup.bash ## initialize search path to include local workspace
catkin_make
</code></pre>

# 3. Usage #

#### 3.1 Start the ROS core ####

<pre><code>roscore &
</code></pre>

#### 3.2 Start capturing ####

<pre><code>rosrun argos3d_p100 argos3d_p100_node 
</code></pre>

*Use --help parameter to display parameter initialization usage*

#### 3.3 Visualization in rviz ####

<pre><code>rosrun rviz rviz 
</code></pre>

*After the rviz windows comes up, set the following options*

1. In the "Display" on the left, set in the "Global Option" the fixed_frame as **/tf_argos3d**
2. At the bottom in "Display" click on **add** 
3. In the "Create visualization" opened, select the "By topic" tab and select the **/depth_non_filtered** topic.

Add a Pointcloud2 topic to visualize the depthclouds. Three different set of points are published with following topic names:
> - **/depth_non_filtered :** raw data from the pmd camera
> - **/depth_filtered : after** applying statistical outlier detection from pcl

#### 3.4 Using filters and parameters configuration ####

To use the filter and change camera parameters, use dynamic_reconfigure from ros. To use it do (after launching argos3d_p100_ros_pkg)

<pre><code>rosrun rqt_reconfigure rqt_reconfigure 
</code></pre>

*Select /argos3d_p100 to view the options available for modifications.*

Following camera parameters and filtering methods can be accessed using the dynamic reconfigure

* **Integration_Time :** Modifies the integration time of the sensor.
* **Modulation_Frequency :** Modifies the modulation frequency of the sensor.
* **Bilateral_Filter :** Turns bilateral filtering on or off.
* **Amplitude_Filter_On :** Indicates if the amplitude filteration to be used or not
* **Amplitude_Threshold :** Image pixels with lesser aplitude values will be filtered out. Amplitude_Filter_On status should be true to apply this filter.
