#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "intera_core_msgs/IODeviceStatus.h"
#include "intera_core_msgs/IOComponentCommand.h"
#include "intera_core_msgs/JointCommand.h"
#include "sstream"
#include "vector"
using namespace std;
//define global msg
double angles [7];
double cur_pos[7];
//define functions
void joint_stateCallback (const sensor_msgs::JointState& joint);
void movetotarget(void);
void recorddata(void);
void replaypose(int);
void cleardata(void);
void cuff_1_callback (const intera_core_msgs::IODeviceStatus& button);
int c;
vector<vector<double> > g1;
int q = 0;
int low_button_status [2] = {0,0};
int up_button = 0;//record mode
int low_button = 0;//record now
//define ros publisher and subscriber on sawyer
ros::Publisher joint_0_pub;
ros::Subscriber angle_0_sub, cuff_0_sub;
ros::Time posetime;

void joint_stateCallback (const sensor_msgs::JointState& joint)
{
    for (int i=0;i<7;i++)
        cur_pos[i] = joint.position[i+1];
}
void cuff_1_callback (const intera_core_msgs::IODeviceStatus& button)
{
    ROS_INFO("Enter call back");
    ROS_INFO("The upper status %d", button.signals[1].data[1]-48);
    low_button = button.signals[0].data[1]-48;
    low_button_status[1] = low_button;
    up_button = button.signals[1].data[1]-48;
}
void movetotarget()
{
    //Publish msg using array
    intera_core_msgs::JointCommand msg;
    msg.mode = 1;
    msg.names.push_back("right_j0");
    msg.names.push_back("right_j1");
    msg.names.push_back("right_j2");
    msg.names.push_back("right_j3");
    msg.names.push_back("right_j4");
    msg.names.push_back("right_j5");
    msg.names.push_back("right_j6");
    for (int i=0;i<7;i++)
        msg.position.push_back(angles[i]);
    joint_0_pub.publish(msg);
}
void recorddata()
{
    for(int i=0;i<7;i++)
    {
        g1.push_back(std::vector<double>());
        g1[q].push_back(cur_pos[i]);
    }
    ros::Time posetime ros::Time::now());
    q++;
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
    ROS_INFO("System running");
    joint_0_pub = n.advertise<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
    cuff_0_sub = n.subscribe("/io/robot/cuff/state",1000, cuff_1_callback);
    angle_0_sub = n.subscribe("/robot/joint_states",1000, joint_stateCallback);
    //record poses
    ROS_INFO("short press upper button to enter record mode");
    while (ros::ok())
    {   
        //short press upper (white) button to enter record mode
        if (up_button == 1)
        {
            ROS_INFO("Enter recording mode, please move the arm safely");
            //Long press upper button again to stop record mode
            while (up_button != 2 && ros::ok())
            {
                //save the botton status to see when to record
                if ((low_button == 1) && (low_button_status[0] == 0))
                {
                    recorddata();
                    ROS_INFO("pose recorded");
                }
                else if (low_button == 2)
                {
                    ROS_INFO("stop recording");
                    break;
                }
                low_button_status[0] = low_button_status[1];
                ros::spinOnce();
            }
        }
        if (up_button == 2)
        {
            ROS_INFO("Exit record mode");
            break;
        }
        ros::spinOnce();
    }
    //replay poses
    while (ros::ok())
    {
         ROS_INFO("Choose mode:  1 for replay the poses.  2 for clean all poses. 3 for exit the program.");
         scanf("%d", &j);
         switch (j) {
        //     case 1: {   inputdata();
        //                 ros::Time start = ros::Time::now();
        //                 while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
        //                 {
        //                     movetotarget();
        //                     ros::spinOnce();
        //                 }
        //                 break;}
            case 1: 
            {   
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
                break;
            }
            case 2: {   cleardata();
                        break;}
            case 3: {   break;}
        }   
        if (j==3) 
            break;
    }
    return 0;
}


