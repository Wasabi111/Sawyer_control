#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "sstream"
#include "vector"
using namespace std;
//define global msg
double angles [7];
double cur_pos[7];
//define functions
void joint_stateCallback (const sensor_msgs::JointState& joint);
void movetotarget(void);
void inputdata(void);
void replaypose(int);
void cleardata(void);
int c;
vector<vector<double> > g1;
int q=0;
//define ros publisher and subscriber on sawyer
ros::Publisher joint_0_pub, joint_1_pub, joint_2_pub, joint_3_pub, joint_4_pub, joint_5_pub, joint_6_pub;
ros::Subscriber angle_0_sub;

void joint_stateCallback (const sensor_msgs::JointState& joint)
{
    for (int i=0;i<7;i++)
        cur_pos[i] = joint.position[i+1];
}
void movetotarget()
{
    //Publish msg using array
    std_msgs::Float64 msg;
    msg.data = angles[0];
    joint_0_pub.publish(msg);
    msg.data = angles[1];
    joint_1_pub.publish(msg);
    msg.data = angles[2];
    joint_2_pub.publish(msg);
    msg.data = angles[3];
    joint_3_pub.publish(msg);
    msg.data = angles[4];
    joint_4_pub.publish(msg);
    msg.data = angles[5];
    joint_5_pub.publish(msg);
    msg.data = angles[6];
    joint_6_pub.publish(msg);
}
void inputdata()
{
    //input data to angles array
    ROS_INFO("Input data? 1/0");
    scanf("%d", &c);
    if (c == 1)
    {
    ROS_INFO("Please input angle data for 7 joints: ");
        for(int i=0;i<7;i++)
        {
         scanf("%lf", &angles[i]);
         g1.push_back(std::vector<double>());
         g1[q].push_back(angles[i]);
        }
        q++;
    }
    else 
    return;
}
void replaypose(int i)
{
    for (int j=0;j<7;j++)
    {
    angles[j] = g1[i][j];
    }
        //ROS_INFO("%lf ", g1[i]);
}
void cleardata()
{
    g1.resize(1);
    g1[1].resize(1);
}
bool reach()
{
    for (int i=0;i<7;i++)
    {
        if (abs(angles[i]-cur_pos[i])>0.01)
        {
            ROS_INFO("Joint %d still have %lf to asked position", i, abs(angles[i]-cur_pos[i]));
            return false;
        }
    }
    ROS_INFO("Reach!");
    return true;
}
int main(int argc, char **argv)
{
    int j;
    int p;
    ros::init(argc, argv, "Joint_pos");
    ros::NodeHandle n;
    joint_0_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j0_controller/command", 1000);
    joint_1_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j1_controller/command", 1000);
    joint_2_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j2_controller/command", 1000);
    joint_3_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j3_controller/command", 1000);
    joint_4_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j4_controller/command", 1000);
    joint_5_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j5_controller/command", 1000);
    joint_6_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j6_controller/command", 1000);
    angle_0_sub = n.subscribe("/robot/joint_states",1000, joint_stateCallback);
//User input joint angle
    while (ros::ok())
    {
        ROS_INFO("Choose mode: 1 for input another pose. 2 for replay the poses.  3 for clean all poses. 4 for exit the program.");
        scanf("%d", &j);
        switch (j) {
            case 1: {   inputdata();
                        ros::Time start = ros::Time::now();
                        while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
                        {
                            movetotarget();
                            ros::spinOnce();
                        }
                        break;}
            case 2: {   
                        int k;
                        k = g1.size();
                        if (k>1)
                        {
                            for (int i=0;i<k/7;i++)
                            {
                            replaypose(i);
                            ros::Time start = ros::Time::now();
                            while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
                            {
                                movetotarget();
                                ros::spinOnce();
                            }
                            }
                        }
                        else 
                        {
                            ROS_INFO("Not enough poses to replay.");
                        }
                        break;}
            case 3: {   cleardata();
                        break;}
            case 4: {   break;}
        }   
        if (j==4) 
            break;
    }
    return 0;
}


