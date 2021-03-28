// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// include math 
#include <math.h>

using namespace std;
#define PI 3.14159265
turtlesim::Pose pose;
geometry_msgs::Twist vel_msg;
geometry_msgs::Point goal_point;
//Define a data structure to 3D
struct XYZ{
  float x;
  float y;
  float z;
  float angle;
};
//Declare a variable.Its name is pos_err with XYZ data type
struct XYZ pos_err;

// declare call back function(call back the pose of robot)
void pos_cb(const turtlesim::Pose::ConstPtr& msg)
{
  pose = *msg;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tutorial_1");
  ros::NodeHandle n;

  // declare publisher & subscriber
  ros::Subscriber pos_sub = n.subscribe<turtlesim::Pose>("turtle1/pose", 10, pos_cb);
  ros::Publisher turtlesim_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
  //input your desired position
  ROS_INFO("Please input (x,y). x>0,y>0");
  cout<<"desired_X:";
  cin>>goal_point.x;
  cout<<"desired_Y:";
  cin>>goal_point.y;
  // setting frequency as 10 Hz
  ros::Rate loop_rate(10);

  int count = 0;
  int rotate_flag=0;
  int margin = 0.5;
  float p_gain_d = 0.5;
  float p_gain_c = 0.05;
  float last_pos_err = 0;
  while (ros::ok()){

    static float d_linear_x=0.0;
    static float d_angular_z=0.0;
    float m_d = 0.0 ;
    printf("\ncount : %d\n",count);
    printf("goal x : %f \t y : %f\n",goal_point.x,goal_point.y);
    printf("pose x : %f \t y : %f\n",pose.x,pose.y);

    // Calculate position error(feedback term)
    pos_err.x = goal_point.x - pose.x;
    pos_err.y = goal_point.y - pose.y;
    pos_err.angle = atan2(pos_err.y,pos_err.x);
    printf("pos_err.angle :%f\n",pos_err.angle);
    printf("pose.theta : %f\n",pose.theta);
    pos_err.angle = (pos_err.angle - pose.theta)* 180 / PI;
    while(abs(pos_err.angle)>=180){
      if(pos_err.angle>0){
        pos_err.angle -=360;
      }else{
        pos_err.angle +=360;
      }
    }

    // prevent change direction rapidly
    if(last_pos_err*pos_err.angle<0){
      if(rotate_flag>3){
        rotate_flag = 0;
      }else{
        pos_err.angle = last_pos_err;
        rotate_flag+=1;        
      }
    }
    if(pose.x!=0)last_pos_err = pos_err.angle;
    
    //Your error-driven controller design
    if(abs(pos_err.x) < margin && abs(pos_err.y) < margin){
      d_linear_x = 0.0;
      d_angular_z = 0.0;      
    }else{
      m_d = abs(pos_err.x)+abs(pos_err.y);
      d_linear_x=m_d*p_gain_d;
      d_angular_z=pos_err.angle*p_gain_c;      

    }

    vel_msg.linear.x = d_linear_x; //positive-->turn left
    vel_msg.angular.z = d_angular_z; //positive-->counterclockwise
    turtlesim_pub.publish(vel_msg);

    count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



