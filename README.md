# 代码说明

本代码共包含4份源码，皆为ros package：

* **bundlefusion_ros** bundlefusion在ros上的移植，为本项目的服务器端
* **fastfusion_ros** fastfusion在ros上的移植，为本项目的在线端
* **rgbdslam** rgbdslam-v2，为本项目的离线重建与渲染端
* **topic_demo** 自定义的ros消息类型，包含相机位姿与RGB图像和深度图像



***配置方法***

1. 需首先安装`ROS`和`ros_astra_camera`

   1.1 `ROS`安装方法：http://wiki.ros.org/ROS/Installation

   1.2 `ros_astra_camera`安装方法：https://github.com/orbbec/ros_astra_camera

2. 将以上四个文件复制到步骤1.2中创建的`~/catkin_ws/src`目录下，编译整个workspace

    ```bash
catkin_make
source devel/setup.bash
    ```

    若步骤2 catkin_make过程出错，可以按照提示安装缺乏的依赖项，并查阅以下内容：
    
    * [ros_astra_camera](https://github.com/orbbec/ros_astra_camera)
    * [rgbdslam_v2](https://github.com/felixendres/rgbdslam_v2)
    * [BundleFusion_Ubuntu_V0](https://github.com/nonlinear1/BundleFusion_Ubuntu_V0) and [BundleFusion](https://github.com/niessner/BundleFusion)
    * [fastfusion](https://github.com/tum-vision/fastfusion)

3. `bundlefusion_ros`中，`src/mLibExternal`目录包含一些依赖的库，文件较大，请直接在：http://kaldir.vc.in.tum.de/mLib/mLibExternal.zip 下载并解压到`src/mLibExternal`目录使用。
4. 请将`bundlefusion_ros/src`下的`zParametersDefault.txt`和`zParametersDefault.txt`两个文件复制到catkin workspace下的`devel/lib/bundlefusion_ros`目录下，并在该目录下新建名为`sacns`的文件夹以存放服务器端重建的三维模型



***相机连接方法***

1. 主机与开发板之间通过以太网或局域网建立连接，并在`~/.bashrc`增加对应的内容

   ```bash
   # 主机端 ip：192.168.2.51
   # ~/.bashrc 中加入以下两行
   export ROS_HOSTNAME=192.168.2.51
   export ROS_MASTER_URI=http://192.168.2.53:11311
   
   # 开发板端 ip：192.168.2.53
   # ~/.bashrc 中加入以下两行
   export ROS_HOSTNAME=192.168.2.53
   export ROS_MASTER_URI=http://192.168.2.53:11311
   ```

2. 将Astra Pro连接开发板，并在开发板上运行`ros_astra_camera`

   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   roslaunch astra_camera astrapro.launch
   ```




***离线渲染版系统运行方法***

3. 在开发板上运行fastfusion_ros

   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   rosun fastfusion_ros onlinefusion ~/catkin_ws/src/fastfusion_ros/associate.txt --intrinsics ~/catkin_ws/src/fastfusion_ros/intrinsics.txt --imagescale 1000 --thread-fusion
   ```

   按`s`键，等待服务器端发送计算得到的相机位姿

4. 在服务器端运行bundlefusion_ros

   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   cd ~/catkin_ws/devel/lib/bundlefusion_ros
   rosrun bundlefusion_ros FriedLiver
   ```

完成上述步骤后用户需要操控Astra Pro相机，拍摄需要重建的环境，并尽可能保证环境内不要出现移动的人或物。服务器端会从开发板上订阅采集到的彩色图像和深度图像，以及相机内参，并将恢复出的相机位姿发送回开发板，供开发板离线融合并渲染，用户可以实时采集图像。此外服务器端也同时进行了消耗计算资源更大的三维重建，但并没有可视化界面，按数字键`9`可将重建结果保存到`~/catkin_ws/devel/lib/bundlefusion/scans`下，使用`meshlab`等软件可以查看该三维模型。

***在线渲染版系统运行方法***

3. 在主机上运行rgbdslam

   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   roslaunch rgbdslam rgbdslam_orbbec.launch
   ```

   完成上述步骤后用户需要操控Astra Pro相机，拍摄需要重建的环境，并尽可能保证环境内不要出现移动的人或物。此时主机会从开发板上订阅采集到的彩色图像和深度图像，以及相机内参，并直接在主机端实时的估计相机位姿，并将三维重建结果实时的展示在主机端供用户观察，并提供了GUI界面与用户进行交互。
