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
#include <cmath>        // std::abs

 
class WallFollowingController
{
public:

    ros::NodeHandle n;
    ros::Publisher twist_pub_test_;
    ros::Subscriber dist_sub_test_;
    ros::Subscriber adc_sub_test_;


    WallFollowingController()
    {

	ros::init(argc, argv, "wall_following_controller");
        n = ros::NodeHandle("~");

        front_left_adc = 0.0; //lower left
        back_left_adc = 0.0; //upper left
        front_right_adc = 0.0; //upper right
        back_right_adc = 0.0; //lower right
        front_adc = 0.0; //front
    
        twist_pub_test_ = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
        adc_sub_test_ = n.subscribe<ras_arduino_msgs::ADConverter>("/arduino/adc", 1, &WallFollowingControllerNodeTest::ddist_sensor_function_test, this);
	//dist_sub_ = n.subscribe<localization::Distance_message>("/ir_measurements", 10, &WallFollowingControllerNodeTest::dist_measure_test, this);

    }

    ~WallFollowingController()
    {
        //delete motor_controller_;
    }


void ddist_sensor_function_test(const ras_arduino_msgs::ADConverter distance_sensor_msg)
{

    front_left_adc = distance_sensor_msg.ch1; //lower left
    back_left_adc = distance_sensor_msg.ch2; //upper left
    
    front_right_adc = distance_sensor_msg.ch3; //upper right
    back_right_adc = distance_sensor_msg.ch7; //lower right
    
    front_adc = distance_sensor_msg.ch8; //front
    std::cout << "test:"<<front_adc<<", "<< distance_sensor_msg.ch8<<"\n";
}


//void dist_measure_test(const localization::Distance_message& distance_msg){
//	front_left_dist =  distance_msg.d1;
//	back_left_dist = distance_msg.d2;
//	front_right_dist = distance_msg.d3;
//	back_right_dist = distance_msg.d4;
//	front_dist_1= distance_msg.d5;
//	front_dist_2= distance_msg.d6;

//}



void run()
    {


   // double left_dist = (front_left_dist + back_left_dist) / 2;
   // double right_dist = (front_right_dist + back_right_dist) / 2;
 
    double dist_lr_wall = (front_right_adc  - front_left_adc);
    // std::cout << front_adc;

    while(1)
    {

	twist_msg.linear.x = 0;
	twist_msg.angular.z = 0;
    //std::cout << front_adc<<"\n";

        if( front_adc <= 100) //initial value to be calibrated, it nothing infront is detected go forward and follow the right wall
        {                                                                                    // and same distance is kept fixed
            twist_msg.linear.x=0.2;
            twist_msg.angular.z = 0.0;
        }else if(std::abs(dist_lr_wall) >= 100 & dist_lr_wall >= 0){
		  twist_msg.linear.x =0.0;
		  twist_msg.angular.z =0.2;
	   }else if(std::abs(dist_lr_wall) >= 100 & dist_lr_wall <= 0){
          twist_msg.linear.x =0.0;
          twist_msg.angular.z =-0.2;
       }
    twist_pub_test_.publish(twist_msg);

    }

 }


private:

    geometry_msgs::Twist twist_msg;
    double front_left_adc;
    double back_left_adc;
    double front_right_adc;
    double back_right_adc;
    double front_adc;
    double lin_vel;
    double angular_vel;
    double theta;

//    double front_left_dist;
//    double back_left_dist;
//    double front_right_dist;
//    double back_right_dist;
//    double front_dist_1;
//    double front_dist_2;



};




int main(int argc, char **argv)
{
    
    
    WallFollowingController  wfc;
    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
	// while (ros::ok())
    while(Wall_Following_Controller_Node_Test.n.ok())
    {
    Wall_Following_Controller_Node_Test.twist_function();

    ros::spinOnce();
    loop_rate.sleep();
    }

    wfc.run();

    return 0;
}
