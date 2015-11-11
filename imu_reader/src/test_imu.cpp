#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Quaternion.h>
#include <math.h>



const double ticks = 360;

double orient = 0.0;

void imuCallback(const sensor_msgs::Imu &msg)
{   
    orient = msg.orientation.x;
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "test_imu");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe("imu/data_raw", 1000, imuCallback);    

     // Control @ 10 Hz
    double control_frequency = 10.0;
    ros::Rate loop_rate(control_frequency);

    while(ros::ok())
    {

        std::cout << orient;


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
