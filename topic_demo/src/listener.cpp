
#include <iostream>
#include <ros/ros.h>
//包含自定义msg产生的头文件
#include <topic_demo/pose.h>
//ROS标准msg头文件
#include <std_msgs/Float32.h>

void gpsCallback(const topic_demo::pose::ConstPtr &msg)//topic_demo::gps::ConstPtr为gps.msg文件编译生成的头文件中定义的数据类型，是一个常指针。
{   
    ROS_INFO("Listener: TIME %d depthname: %s",msg->time, msg->depthname);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");//初始化，且节点名为listener
  ros::NodeHandle n; //节点句柄为n
  ros::Subscriber sub = n.subscribe("pose_info", 100, gpsCallback); //gpsCallback为回调函数，即当Subscriber接收到信息后，会调用回调函数进行处理。gps_info为接收到的msg，1为缓冲队列的长度。
  ros::spin(); //ros::spin()用于调用所有可触发的回调函数
  return 0;
}
