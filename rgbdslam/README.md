# RGBDSLAM<i>v2</i> 
... is a state-of-the-art SLAM system for RGB-D cameras, e.g., the Microsoft
Kinect or the Asus Xtion Pro Live.  You can use it to create 3D point clouds or
OctoMaps. 

RGBDSLAMv2 is based on the open source projects, ROS, OpenCV, OpenGL, PCL,
OctoMap, SiftGPU, g2o, and more - Thanks!

A journal article with a system description and performance evaluation 
can be found in the following publication:

"3D Mapping with an RGB-D Camera",<br/>
*F. Endres, J. Hess, J. Sturm, D. Cremers, W. Burgard*,<br/>
IEEE Transactions on Robotics, 2014.

Even more information can be found in my [PhD thesis](http://www2.informatik.uni-freiburg.de/~endres/files/publications/felix-endres-phd-thesis.pdf)

Additional information can be found here:<br/>
* www.informatik.uni-freiburg.de/~endres
* http://www.ros.org/wiki/rgbdslam
* http://answers.ros.org/questions/tags:rgbdslam

<img src="http://raw.githubusercontent.com/felixendres/rgbdslam_v2/hydro/media/rgbdslamv2_fr2desk.jpg" alt="RGBDSLAM on the RGB-D Benchmark Dataset" width="600">

# Prerequisites  ################################################################
- Ubuntu 16.04 
- [ROS kinetic](http://wiki.ros.org/kinetic/)
- Amd64 processor (there are known problems with ARM, mostly related to qt and opengl)
Other versions may work, but are not tested. Please report success if you use other versions.

# Installation ################################################################
This is a canonical way, feel free to adapt if you known what you are doing.

1. Put RGBDSLAMv2 in a catkin workspace: See [the catkin tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 
  for details. Use git to clone this repository into your workspace's "src/" directory. Or download RGBDSLAMv2 as an [archive](http://codeload.github.com/felixendres/rgbdslam_v2/zip/kinetic) and extract it to "src/".

2. Download my [g2o fork|https://github.com/felixendres/g2o], put it in some other directory. 
   Build and install. Export the environment variable `$G2O_DIR` to the installation directory to
   let rgbdslam_v2 know where to find it (see Installation from Scratch for an example).

2. Use rosdep (i.e. "rosdep install rgbdslam") to install missing 
  dependencies. For details see http://wiki.ros.org/ROS/Tutorials/rosdep

4. To build RGBDSLAMv2 go to your catkin workspace and execute "catkin_make". 
   If you get an error about the missing siftgpu library, execute "catkin_make" again.


## Installation from Scratch  #####################################################
There is now an install.sh script, which can be executed (`bash install.sh`).
It installs everything required below ~/Code (you can change the location in the script).

The script is short and not complicated, so you can also use it as a manual.

If you want to use the install script, it is sufficient to 
[download it directly](https://raw.githubusercontent.com/felixendres/rgbdslam_v2/kinetic/install.sh). 
There is no need to clone this repository then, as the script will do that for you.

If you have a multi-core machine with 4GB RAM or more, you can speed up the compilation
by increasing the two occurences of "-j2" to, e.g., "-j4".

# Installation done! What's next?
See the sections below for more details on the usage. 
But to get you started quickly here's the most important pointers:

-   If you want to use RGBDSLAMv2 with an RGB-D camera you may have
    to install openni (sudo apt-get install ros-kinetic-openni-launch)
    or something similar

-   Check out the launch files in "launch/" for examples and specific 
    use cases. roslaunch rgbdslam openni+rgbdslam.launch is a good starting 
    point for live mapping.

-   You probably need to adapt the parameters for the input topics depending
    on your camera driver node.

-   Check out the README in "test/" for running, testing and evaluating
    RGBDSLAMv2 on Juergen Sturm's RGB-D SLAM Dataset and Benchmark:
    http://vision.in.tum.de/data/datasets/rgbd-dataset
    You need cython for the evaluation scripts (sudo apt-get install cython).

-   If you want to use SURF or SIFT, you will need to build OpenCV from source,
    including the non-free module (this does not include SIFTGPU, which is
    included, but needs to be enabled in CMakeLists.txt). In the CMakeLists.txt 
    of RGBDSLAMv2 you can set the build directory of OpenCV and enable the
    non-free functionality.  Note that SIFT and SURF are not the best choice.
    Due to new (software) features in RGBDSLAMv2, ORB outperforms both.

<img src="http://raw.githubusercontent.com/felixendres/rgbdslam_v2/hydro/media/rgbdslamv2_empty.jpg" alt="RGBDSLAM right after startup" width="600">

## IMPORTANT NOTE ################################################################
This software is an update of the ROS Fuerte version of RGBDSLAM. However
many things have changed, so some of the DOCUMENTATION BELOW MAY BE OUTDATED.
Please report problems with the documentation. Thanks.

# Configuration ##############################################################

There are several example launch-files that set the parameters of RGB-D SLAM
for certain use cases. For a definitive list of all settings and their default
settings have a look at their quite readable definition in
src/parameter_server.cpp or (with the current settings instead of the default)
in the GUI Menu Settings->View Current Settings.

The various use-case launch-files might not work correctly yet, as they are not
regularly tested. You should get them running if you fiddle with the topics
("rostopic list" and "rosnode info" will help you. "rqt_graph" is great too).



# Usage ##############################################################

Most people seem to want the registered point cloud. It is by default sent out
on /rgbdslam/batch_clouds when you command RGB-D SLAM to do so (see below). The
clouds sent are actually the same as before, but the according transformation -
by default from /map to /openni_camera - is sent out on /tf.

The octoMap library is compiled into the rgbdslam node. This allows to create
the octomap directly. In the GUI this can be done by selecting "Save Octomap"
from the "Data" Menu. Online octomapping is possible, but not recommended.

<img src="http://raw.githubusercontent.com/felixendres/rgbdslam_v2/hydro/media/rgbdslamv2_fr2desk_octomap.jpg" width="600" alt="OctoMap created from the RGB-D Benchmark sequence fr2/desk">

## Usage with GUI #################################################################

To start RGBDSLAMv2 launch, e.g.,
$ roslaunch rgbdslam openni+rgbdslam.launch

Alternatively you can start the openni nodes and RGBDSLAMv2 separately, e.g.:
roslaunch openni_camera openni_node.launch 
roslaunch rgbdslam rgbdslam.launch

To capture models either press space to start recording a continuous stream or
press enter to record a single frame. To reduce data redundancy, sequential
frames from (almost) the same position are not included in the final model.

Parameters
RGBDSLAMv2 is customizable by parameters. These should be set in the launch
file. Parameters can be changed during operation from the GUI, however, 
changes from the GUI may have no effect for many parameters.

Visualization
The 3D visualization shows the globally optimized model (you might have
to click into it to update the view after optimization). Neighbouring
points can be triangulated except at missing values and depth jumps. With
the shortcut "t", triangulation can be toggled. Since raw points render
slightly faster the parameter "cloud_display_type" controls whether 
triangulation is computed at all - at the time the cloud is received.
The parameter "gl_point_size" may be useful to most users.


## Usage without GUI ##############################################################

The RosUI is an alternative to the Grapical_UI to run the rgbdslam headless,
for example on the PR2.  rgbdslam can then be used via service-calls.
The possible calls are:
 * /rgbdslam/ros_ui   {reset,  quick_save,  send_all,  delete_frame,  optimize,  reload_config, save_trajectory}
 * /rgbdslam/ros_ui_b {pause, record} {true, false}
 * /rgbdslam/ros_ui_f {set_max} {float}
 * /rgbdslam/ros_ui_s {save_octomap,  save_cloud,  save_g2o_graph,  save_trajectory,  save_features,  save_individual} {filename}

To start the rgbdslam headless use the headless.launch: 
$ roslaunch rgbdslam headless.launch

Capture single frames via: 
$ rosservice call /rgbdslam/ros_ui frame

Capture a stream of data: 
$ rosservice call /rgbdslam/ros_ui_b pause false

Send point clouds with computed transformations (e.g., to rviz or octomap_server): 
$ rosservice call /rgbdslam/ros_ui send_all

Save data using one of the following:

All pointclouds in one file quicksave.pcd in rgbdslam/bin-directory: 
$ rosservice call /rgbdslam/ros_ui_s save_cloud

Every pointcloud in its own file in rgbdslam/bin-directory: 
$ rosservice call /rgbdslam/ros_ui save_individual

/rgbdslam/ros_ui: 
	
 * reset	''resets the graph, delets all nodes (refreshes only when capturing new images)''
 * frame	''capture one frame from the sensor''
 * optimize	''trigger graph optimizer''
 * reload_config	''reloads the paramters from the ROS paramter server''
 * quick_save ''saves all pointclouds in one file quicksave.pcd in rgbdslam/bin-directory''
 * send_all	''sends all pointclouds to /rgbdslam/transformed_cloud (can be visualized with rviz)''
 * delete_frame	''delete the last frame from the graph (refreshes only when capturing new images)''
	
/rgbdslam/ros_ui_b: 

 * pause <flag>	''pauses or resumes the capturing of images'' 
 * record <flag>  ''pauses or stops the recording of bag-files, can be found in the rgbdslam/bin-directory''
	
/rgbdslam/ros_ui_f: 

 * set_max <float>	''filters out all datapoints further away than this value (in cm, only for saving to files)''

/rgbdslam/ros_ui_s: 

 * save_features <filname> ''saves the feature locations and descriptors in a yaml file with the given filename''
 * save_cloud	<filename> ''saves the cloud to the given filename (should end with .ply or .pcd)''
 * save_individual <fileprefix>	''saves every scan in its own file (appending a suffix to the given prefix)''
 * save_octomap	<filename> ''saves the cloud to the given filename''
 * save_trajectory	<fileprefix> ''saves the sensor trajectory to the file <fileprefix>_estimate.txt''



# Further Help ##################################################################
The compilation may take a lot of memory, particularly if the environment variable
$ROS_PARALLEL_JOBS is set.

If you are located in Germany and get errors loading the saved ply files
into meshlab, try switching to U.S. locale or replace the decimal point with a
comma in your .ply file 

If you have questions regarding installation or usage of RGBDSLAM please refer
to http://answers.ros.org/questions/?tags=RGBDSLAM For further questions,
suggestions, corrections of this README or to submit patches, please contact
Felix Endres (endres@informatik.uni-freiburg.de).

Apart from this manual, code documentation can be created using rosdoc
("rosrun rosdoc rosdoc rgbdslam"), which will create a "doc" folder in your
current directory.



## GICP and SIFTGPU ###############################################################

If there are problems related to the compilation or linking of GICP or SIFTGPU,
you can deactivate these features at the top of CMakeLists.txt. You might get
even faster GPU features setting the parameter "siftgpu_with_cuda" but you will
need to install the proprietary CUDA drivers that may require a NVidia GPU 
(see http://www.nvidia.com/object/cuda_gpus.html).  For installing the
development drivers and the CUDA SDK you can use the following tutorial:
http://sublimated.wordpress.com/2011/03/25/installing-cuda-4-0-rc-on-ubuntu-10-10-64-bit/
or for ubuntu 10.04: http://ubuntuforums.org/showthread.php?t=1625433 (tested
on Ubuntu 10.04 x64) To use SiftGPU you should install "libdevil-dev".
  
  Additional compiling information can be changed in
  external/siftgpu/linux/makefile.
  
  If you get an error that the siftgpu library is not found, execute "make" manually
  in the directory external/siftgpu/ and rerun catkin_make.

  GICP Generalized ICP can be (de)activated for refining the registration. For
  more information see http://stanford.edu/~avsegal/generalized_icp.html



# License Information ############################################################

This software is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.

RGBDSLAM is licenced under GPL v.3. See the accompanying file "COPYING".

