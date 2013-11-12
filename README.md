argos_p100_ros_pkg
===================
### ROS package for Bluetechnix Argos P100 ToF camera. ###

# Summary #

This package explains how to configure your system and ROS to use the Argos P100 ToF camera.
The package includes an example allowing you to visualize images using the rviz viewer included in ROS.
It demostrates how to use the camera within ROS and the different parameter configurations of the Argos
as well as its capabilities.

## First step: Get ROS ##

The argos_p100_ros_kg works with ROS versions groovy and hydro. 
You can use catkin workspaces or the previous rosbuild to configure, compile and get ready ROS.

We will point in the above lines how to get ros_hydro and catkin workspace ready 
from the tutorials of the ROS web site.

In Ubuntu follow the ROS installation tutorial: 
>http://wiki.ros.org/hydro/Installation/Ubuntu.

Why to use catkin workspaces:
>http://wiki.ros.org/catkin 
>
>http://wiki.ros.org/catkin_or_rosbuild
>
>http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To configure a catkin workspace in your ROS installation, follow this: 
>ROS tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## Known Problems ##

Be sure your libboost library version is >= 1.49.
Previous versions as 1.46 generate errors while compiling argos3d_p100-ros-pkg.

# 1. Configuration #

## Setup P100 USB rules and native libraries ##

The first step is to add the PMDSDK to our system. 
(The PMDSDK is copyrighted software and it is distributed with your Argos3D P100)

#### 1.1 Usb Device rules ####

After unziping the software distributed with your argos_p100 camera, browse to the directory 
of your operating system version. 

Install the driver for using the ToF camera:

In Ubuntu/linux copy the file from the driver folder to **/usr/udev/rules.d/**
<pre><code>cd driver
sudo cp 10-pmd-ubuntu.rules /usr/udev/rules.d/
</code></pre>
This will set the right permissions to allow not sudo users to use the camera.

#### 1.2 Libraries setup ####

Add PMDSDK to your system (use the right version for your system 32 or 64 bits).

1. Add the dynamically linked shared object library libpmdaccess2. Use one of the following options:

- You have different ways to add it to the **LD_LIBRARY_PATH**.
- You can simply copy the libpmdaccess2.so file to **/usr/local/lib** as root (or the defauld search library path of your system)
- Add a .conf file (like pmd_camera_L64.conf) with the path of your library in **/etc/ld.so.conf.d**. The file must contaning the following lines:

<pre><code>#PMD camera support
/.../PMDSDK/bin
/.../PMDSDK/include
</code></pre>

*replace the dots with the absolute path to the PMDSDK directory*

2. Add includes and dynamic plugin libraries

- Move the PMDSDK folder to a directory where the operating system can find it (/usr/include)
- or set an environment variable called $PMDDIR to the path where there is the PMDSDK.

After adding libraries, update the links and cache of the share libraries with:

<pre><code>ldconfig
</code></pre>

# 2. Installation #

#### 2.1 Install dependencies ####

Make sure you have the following dependencies already installed:
<pre><code> apt-get install ros-hydro-pcl-ros ros-hydro-pcl-conversions ros-hydro-perception-pcl 
</code></pre>

####  2.2 Install the package ####

Clone from repository: https://github.com/voxel-dot-at/argos3d_p100_ros_pkg.git
to your src/ folder in your catkin workspace.
and compile it with:
<pre><code>cd catkin_ws
source devel/setup.bash ## initialize search path to include local workspace
cd src/
git clone https://github.com/voxel-dot-at/argos3d_p100_ros_pkg.git
cd ..
catkin_make
</code></pre>

# 3. Usage #

#### * Watch our demo video:  ####

> http://youtu.be/CWwY1LJMYmk


#### 3.1 Start the ROS core ####

<pre><code>roscore &
</code></pre>

#### 3.2 Start capturing ####

<pre><code>cd catkin_ws
source devel/setup.bash
rosrun argos3d_p100 argos3d_p100_node 
</code></pre>

*You can change the camera parameters through command option when initializing the node*

*Use --help parameter to display parameter initialization usage*

<pre><code>
 Using help for argos3d_p100_ros_pkg
 You can set default configuration values for the camera with the following options: 

 Usage:
 rosrun argos3d_p100 argos3d_p100_node 
	-it *Integration_Time* 
	  Integration time(in msec) for the sensor 
	  (min: 100 | max: 2700 | default: 1500) 
	-mf  *Modulation_Frequency* 
	  Set the modulation frequency(Hz) of the sensor 
	  (min: 5000000 | max: 30000000 | default: 30000000) 
	-bf *Bilateral_Filter* 
	  Turns bilateral filtering on or off 
	  (ON: if set | OFF: default) 
	-af *Amplitude_Filter_On* 
	  Whether to apply amplitude filter or not. Image pixels with amplitude values less than the threshold will be filtered out 
	  (ON: if set | OFF: odefault) 
	-at *Amplitude_Threshold* 
	  What should be the amplitude filter threshold. Image pixels with lesser aplitude values will be filtered out. Amplitude Filter Status should be true to use this filter 
	  (min: 0 | max: 2500 | default: 0) 

 Example:
 rosrun argos3d_p100 argos3d_p100_node -it 1500 -mf 30000000 
</code></pre>

#### 3.3 Visualization in rviz ####

<pre><code>rosrun rviz rviz 
</code></pre>

*After the rviz window comes up, set the following options*

Add a Pointcloud2 topic to visualize the depth clouds. 
Two different point sets are published with following topic names:
> - **/depth_non_filtered :** raw data from the pmd camera
> - **/depth_filtered : after** applying statistical outlier detection from pcl

To do this please perform the following steps:

1. In the "Display" panel on the left, open the first group of settings called "Global Options", set the option "Fixed frame" to **/tf_argos3d**
2. At the bottom in "Display" click on the button **add** to open a dialog titled "Create visualization"
3. In the dialog opened after, choose the tab "By topic" and select the **/depth_non_filtered** topic.

#### 3.4 Using filters and parameters configuration ####

To apply point filters and change camera parameters, use dynamic_reconfigure from ros. 
To use, it run the configuration interface (after launching argos3d_p100_ros_pkg)

<pre><code>rosrun rqt_reconfigure rqt_reconfigure 
</code></pre>

*Select /argos3d_p100 to view the options available for modifications.*

Following camera parameters and filtering methods can be accessed using the dynamic reconfigure

* **Integration_Time :** Modifies the integration time of the sensor.
* **Modulation_Frequency :** Modifies the modulation frequency of the sensor.
* **Bilateral_Filter :** Turns bilateral filtering on or off.
* **Amplitude_Filter_On :** Use  the amplitude filter or not
* **Amplitude_Threshold :** Image pixels with smaller amplitude values will be filtered out. Amplitude_Filter_On status needs to be true to apply this filter value.
