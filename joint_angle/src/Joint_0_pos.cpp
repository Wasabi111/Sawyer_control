#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sstream"
using namespace std;

//define Global msg
double angles[7];
double curr_pos[7];
void joint_statesCallback (const sensor_msgs::JointState& joint);
void moveToTarget(void);

//Publisher Defination 7 joints on Sawyer
ros::Publisher chatter_0_pub, chatter_1_pub, chatter_2_pub, chatter_3_pub, chatter_4_pub, chatter_5_pub, chatter_6_pub;
ros::Subscriber angle_0_sub;

void joint_statesCallback (const sensor_msgs::JointState& joint)
{
    for (int i = 0; i < 7; i++)
    {
      curr_pos[i] = joint.position[i+1];
    }
}

void moveToTarget()
{
//Publish msg using array
  std_msgs::Float64 msg;
  msg.data = angles[0];
  chatter_0_pub.publish(msg);
  msg.data = angles[1];
  chatter_1_pub.publish(msg);
  msg.data = angles[2];
  chatter_2_pub.publish(msg);
  msg.data = angles[3];
  chatter_3_pub.publish(msg);
  msg.data = angles[4];
  chatter_4_pub.publish(msg);
  msg.data = angles[5];
  chatter_5_pub.publish(msg);
  msg.data = angles[6];
  chatter_6_pub.publish(msg);
}

bool reach()
{
  for (int i = 0; i < 7; i++)
  {
    if(abs(curr_pos[i]-angles[i]) > 0.001)
    {
      ROS_INFO("diff of %d: %lf", i, abs(curr_pos[i]-angles[i]));
      return false;
    }
  }
  ROS_INFO("reach!");
  return true;
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Joint_0_pos");
  ros::NodeHandle n;
  chatter_0_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j0_controller/command", 1000);
  chatter_1_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j1_controller/command", 1000);
  chatter_2_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j2_controller/command", 1000);
  chatter_3_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j3_controller/command", 1000);
  chatter_4_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j4_controller/command", 1000);
  chatter_5_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j5_controller/command", 1000);
  chatter_6_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j6_controller/command", 1000);
  angle_0_sub = n.subscribe("/robot/joint_states",1000, joint_statesCallback);

//Input by user
  while (ros::ok())
{
  printf("Please input Joint positions: ");
  for(int i=0;i<7;i++)
 	 scanf("%lf", &angles[i]);

  ros::Time start = ros::Time::now();
  while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
  {
    moveToTarget();
    ros::spinOnce();
  }
}
  return 0;
}
