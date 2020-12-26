//代码段1  主要讲解ros::init
#include <ros/ros.h>  //所有的ROS c++程序都要使用这个头文件
#include <topic_demo/pose.h> //gps.h使gps.msg文件编译生成的.h文件

int main(int argc,char** argv){  
ros::init(argc,argv,"talker");//初始化 ROS 

//代码段2 
ros::NodeHandle nh; //定义节点的句柄，实例化节点。
topic_demo::pose msg ;//定义msg变量，类型为topic_demo命名空间下的gps
msg.time=1;
msg.translation1=0.0033839;
msg.translation2=-0.00175466;
msg.translation3= 0.000273935;
msg.q1=-0.0006228626557156834;
msg.q2=0.00017081929270482853;
msg.q3=0.00032601358150340554;
msg.q4=0.9999997499999687;
msg.depthname = "a.depth.png";
msg.rgbname = "a.rgb.png";


//创建publisher
ros::Publisher pub = nh.advertise<topic_demo::pose>("pose_info", 100); //定义ros::Publisher类型的变量 pub为nh.advertise<topic_demo::gps>("gps_info", 1)，nh.advertise<topic_demo::gps>("gps_info", 1);为模板函数，指的是在函数进行定义时，不指定其应该传入的具体参数类型，而是以一个模板传入，使用此函数的时候，再定义函数的参数类型。代码下面有详细讲解


//定义发布的频率 
ros::Rate loop_rate(1.0); //循环发送的频率为1，即一秒钟1次
while (ros::ok())
  {
    //msg.time += 1;
    ROS_INFO("Talker: t = %d, depthname = %s ",  msg.time ,msg.depthname);//将当前的信息打印处理，类似于printf。
    //以1Hz的频率发布msg
    pub.publish(msg);
    //根据前面定义的频率, sleep 1s
    loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
  }

  return 0;
}


