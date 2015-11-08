#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "ras_arduino_msgs/ADConverter.h"
#include "geometry_msgs/Twist.h"
#include "localization/Distance_message.h"
#include <sstream>
#include <math.h>
#include <cmath>



const double ticks = 360;
const double max_lin_vel = 0.20;
const double max_angular_vel = 0.30;
const double control_frequency = 10;
const double wheel_radius_ = 0.050;
const double base_ = 0.208;
const double LOOP_RATE = 10;
const double alpha = 0.05;
const double PI = 3.14;
const double DIST_THRESHOLD = 0.10;
const double ALIGN_DIST_THRESHOLD = 0.10;
const double MIN_DIST_FRONT = 0.90;
double theta = 0.0;
const double alpha1 = 0.4;


class WallFollowingControllerNode
{

public:

    ros::NodeHandle n;
    ros::Publisher twist_pub_;
    // ros::Subscriber distance_sub_;
    ros::Subscriber dist_sub_adc;
    ros::Subscriber dist_sub_ir;
    ros::Subscriber encoders_sub_;

    WallFollowingControllerNode()
    {
        n = ros::NodeHandle("~");
        twist_pub_ = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
        dist_sub_adc = n.subscribe("/arduino/adc", 1, &WallFollowingControllerNode::distance_sensor_function, this);
        dist_sub_ir = n.subscribe("/ir_measurements", 1, &WallFollowingControllerNode::dist_sensor_values, this);
        encoders_sub_ = n.subscribe("arduino/encoders", 1, &WallFollowingControllerNode::encoders_feedback, this);
        wall_following = false;
        align_wall = false;
        turn_flag = true;
    }

    ~WallFollowingControllerNode()
    {
        //delete motor_controller_;
    }


    void distance_sensor_function(const ras_arduino_msgs::ADConverter& distance_sensor_msg){
        front_left_adc = distance_sensor_msg.ch1; //lower left
        back_left_adc = distance_sensor_msg.ch2; //upper left
        front_right_adc = distance_sensor_msg.ch3; //upper right
        back_right_adc = distance_sensor_msg.ch7; //lower right 
        front_adc_2 = distance_sensor_msg.ch8;  
        front_adc_1 = distance_sensor_msg.ch8; //front    
    }


     void dist_sensor_values(const localization::Distance_message& dist_msg){
         front_left = dist_msg.d1;
         back_left = dist_msg.d2;
         front_right = dist_msg.d3;
         back_right = dist_msg.d4;
         front_wall_1 = dist_msg.d5;
         front_wall_2 = dist_msg.d6;
     }


    void encoders_feedback(const ras_arduino_msgs::Encoders& msg){
        delta_enc1 = msg.delta_encoder1;
        delta_enc2 = msg.delta_encoder2;

        estimated_w_left = (delta_enc1*2*PI* control_frequency)/ticks;
        estimated_w_right = (delta_enc2*2*PI* control_frequency)/ticks;
        estimated_linear_vel = (estimated_w_right + estimated_w_left) * wheel_radius_ / 2 ;
        estimated_angular_vel = (estimated_w_left - estimated_w_right) * wheel_radius_ / base_;

    }


    void updateDistances(){



    }


    void run(){
        twist_msg.angular.z = 0.90;
        twist_msg.linear.x = 0.0;
        ROS_INFO("%f,%f",twist_msg.angular.z, twist_msg.linear.x);

        twist_pub_.publish(twist_msg);

    
    }


    void stopRobo(){
        linear_vel = 0.0;
        angular_vel = 0.0;
    }


    // bool turnRobo(){
    //     int turn_direction;
    //     turn_direction = left_dist > right_dist ? 1 : -1;
    //     linear_vel = 0.0;
    //     angular_vel = turn_direction * 0.10;

    //     // use erics code to integrate odometry here ..

    //     current_time = ros::Time::now();
    //     double dt = (double) current_time.toSec() - last_time.toSec();
    //     double theta_diff = dt * estimated_angular_vel;
    //     theta = theta + theta_diff;
    //     last_time = current_time;
    //     if (abs(theta) >= PI/2){
    //         theta = 0.0;
    //         angular_vel = 0.0;
    //         linear_vel = 0.0;
    //         // align_wall = true;
    //         return true;
    //     }

    //     return false;

    // }



private:
    geometry_msgs::Twist twist_msg;
    double front_left;
    double back_left;
    double front_right;
    double back_right;
    double left_dist;
    double right_dist;
    double left_right_diff;
    double front_left_adc;
    double back_left_adc;
    double front_right_adc;
    double back_right_adc;
    double front_adc_1;
    double front_adc_2;
    double delta_enc1;
    double delta_enc2;
    double estimated_w_right;
    double estimated_w_left;
    bool turn_flag;
    bool wall_following;
    bool align_wall;
    bool stop_flag;
    double linear_vel;
    double angular_vel;
    double left_distance;
    double right_distance;
    double estimated_angular_vel;
    double estimated_linear_vel;
    ros::Time current_time;
    ros::Time last_time;
    double delta_left;
    double delta_right;
    double front_wall_1;
    double front_wall_2;
    double front;

};


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "wall_following_controller");
    WallFollowingControllerNode Wall_Following_Controller_Node;
     // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
    // while (ros::ok())
    while(Wall_Following_Controller_Node.n.ok())
    {
    Wall_Following_Controller_Node.run();

    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}
