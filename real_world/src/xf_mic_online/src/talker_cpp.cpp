#include <iostream>
// #include <fstream>
#include <string>
// #include <locale>
// #include <codecvt>
// #include <ctime>
#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/String.h>
// #include <joint.h>
// #include <std_msgs/Int8.h>
// #include <std_msgs/Int32.h>
#include <std_msgs/Header.h>
#include <sys/stat.h>
#include <sstream>
#include <unistd.h>
#include <pwd.h>
#include <cstdlib>
#include <boost/bind.hpp>
#include <signal.h>

bool ros_stop = false;
ros::Publisher chatter_pub;

// void Callback(const std_msgs::Header::ConstPtr& data)
// {
//   ros::Time current = ros::Time::now() ;
//   ros::Duration respl = current- data->stamp;
//   ROS_INFO("sum is %f.%f",respl.toSec(),respl.toNSec());
// }

void sigint_handler(int sig)
{
    if(sig ==SIGINT )
    {
    ros::Time current;
    current = ros::Time::now() ;
    std_msgs::Header msg;
    msg.stamp = current;
    msg.frame_id = " ";
    msg.seq   = 0;
    chatter_pub.publish(msg);
    ros_stop = true;
    ros::shutdown();
    }
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "timer");
	ros::NodeHandle ndHandle("~");
	ros::NodeHandle n;
    chatter_pub = n.advertise<std_msgs::Header>("switch", 1);
    ros::Rate loop_rate(10);
    ros::Time last_time = ros::Time::now();
    // double time_difference[1000];
    std_msgs::Header msg;
    int index = 0;
    ros::Time current;
    // ros::Duration respl;
    // fstream  file;
    signal(SIGINT,sigint_handler);
    while (!ros::isShuttingDown() && !ros_stop)
    {
        current = ros::Time::now();
        msg.stamp = current;
        msg.frame_id = " ";
        msg.seq   = 1;
        chatter_pub.publish(msg) ; 
        // respl = current- last_time;
        // time_difference[index] = respl.toNSec();
        last_time = current;
        index +=1;
        // ros::spinOnce();
        loop_rate.sleep();
        if (ros_stop)
        {
            break;
        }
        ros::spinOnce();
    }
    std::cout << "Program STOP" << std::endl ; 
    // file.open("file.txt", ios::out );
    // for (int i=0;i<index;i++)
    // {
    //     file << time_difference[i];
    //     file << ","; 

    // }
    // file.close();
	return 0;
}
