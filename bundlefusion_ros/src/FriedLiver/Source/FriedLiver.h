#include <string>
#include <iostream>
#include "GlobalAppState.h"
#include "mLib.h"
#include "GlobalBundlingState.h"
#include "DualGPU.h"
#include "ConditionManager.h"
#include "RGBDSensor.h"
//#include "BinaryDumpReader.h"
#include "PrimeSenseSensor.h"
//#include "KinectSensor.h"
//#include "KinectOneSensor.h"
//#include "StructureSensor.h"  don't support linux for Uplink
#include "SensorDataReader.h"
#include "CUDAImageManager.h"//from here to change 
#include "OnlineBundler.h"
#include "DepthSensing/DepthSensing.h"
#include <ros/ros.h>  //所有的ROS c++程序都要使用这个头文件
#include <topic_demo/pose.h> //gps.h使gps.msg文件编译生成的.h文件

// extern ros::Publisher pub;

/*
#ifdef KINECT
#pragma comment(lib, "Kinect10.lib")
#endif

#ifdef KINECT_ONE
#pragma comment(lib, "Kinect20.lib")
#endif

#ifdef OPEN_NI
#pragma comment(lib, "OpenNI2.lib")
#endif

#ifdef INTEL_SENSOR
#ifdef _DEBUG
#pragma comment(lib, "DSAPI.dbg.lib")
#else
#pragma comment(lib, "DSAPI.lib")
#endif
#endif

#ifdef REAL_SENSE
#ifdef _DEBUG
#pragma comment(lib, "libpxc_d.lib")
#pragma comment(lib, "libpxcutils_d.lib")
#else
#pragma comment(lib, "libpxc.lib")
#pragma comment(lib, "libpxcutils.lib")
#endif
#endif

#ifdef STRUCTURE_SENSOR
#pragma comment (lib, "Ws2_32.lib")
#pragma comment(lib, "gdiplus.lib")
#endif
*/
/*
#include "RGBDSensor.h"
#include "BinaryDumpReader.h"
//TODO add other sensors here
#include "PrimeSenseSensor.h"
#include "KinectSensor.h"
#include "KinectOneSensor.h"
#include "StructureSensor.h"
#include "SensorDataReader.h"


#include "GlobalBundlingState.h"
#include "TimingLog.h"

#include "SiftGPU/MatrixConversion.h"
#include "SiftGPU/CUDATimer.h"
#include "SiftGPU/SIFTMatchFilter.h"
#include "CUDAImageManager.h"

#include "ConditionManager.h"
#include "DualGPU.h"
#include "OnlineBundler.h"
#include "DepthSensing/DepthSensing.h"
*/

