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
struct Joint_status
{
    double pose [7];
    ros::Time timestamp;
};
double angles [7];
double cur_pos[7];
//define functions
void joint_stateCallback (const sensor_msgs::JointState& joint);
void movetotarget(void);
void recorddata(void);
void replaypose(int, int, int);
void cleardata(void);
void cuff_1_callback (const intera_core_msgs::IODeviceStatus& button);
int c;
//vector<vector<double> > g1;
int q = 0;
int mode;
int low_button_status [2] = {0,0};
int up_button = 0;//record mode
int low_button = 0;//record now
//define ros publisher and subscriber on sawyer
ros::Publisher joint_0_pub, joint_1_pub, joint_2_pub, joint_3_pub, joint_4_pub, joint_5_pub, joint_6_pub;
ros::Subscriber angle_0_sub, cuff_0_sub;
vector<struct Joint_status> PP1;

void joint_stateCallback (const sensor_msgs::JointState& joint)
{
    for (int i=0;i<7;i++)
    {
        cur_pos[i] = joint.position[i+1];
        // ROS_INFO("Current position are: %lf",cur_pos[i]);
    }

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
    if (mode==2)
    {
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
    if (mode==1)
    {
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
}
void recorddata()
{
    // for(int i=0;i<7;i++)
    // {
    //     g1.push_back(std::vector<double>());
    //     g1[q].push_back(cur_pos[i]);
    // }
    struct Joint_status t;
    for(int i=0;i<7;i++)
    {
        t.pose[i] = cur_pos[i];
        t.timestamp = ros::Time::now();
        ROS_INFO("POS is %lf",t.pose[i]);
    }
    PP1.push_back(t);
    // PP1.push_back(Joint_status());
    // for (int i=0;i<7;i++)
    // {
    //     PP1[q].pose[i] = cur_pos[i];
    //     ros::Time posetime = ros::Time::now();
    //     PP1[q].timestamp = posetime;
    // }
    q++;
}
void replaypose(int i, int p, int n)
{   //The released time interval should be 20 ms. Then the poses are published in (t[i+1]-t[i]) in sec/ 20 ms segments. 
    if (i==0)
    {   
        for (int j=0;j<7;j++)
        {
            angles[j] = PP1[q-1].pose[j]+(PP1[i].pose[j]-PP1[q-1].pose[j])/n*(p+1);
        }
    }
    else
    {
        for (int j=0;j<7;j++)
        {
            angles[j] = PP1[i-1].pose[j]+(PP1[i].pose[j]-PP1[i-1].pose[j])/n*(p+1);
        }
    }
    //ROS_INFO("%lf ", g1[i]);
}
void cleardata()
{
    PP1.resize(0);
    q = 0;
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
    ROS_INFO("Choose mode: 1 for simulator, 2 for real robot");
    scanf("%d",&mode);
    if (mode==2)
    {
        joint_0_pub = n.advertise<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1000);
        cuff_0_sub = n.subscribe("/io/robot/cuff/state",1000, cuff_1_callback);
        angle_0_sub = n.subscribe("/robot/joint_states",1000, joint_stateCallback);
    }
    if (mode==1)
    {
        joint_0_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j0_controller/command", 1000);
        joint_1_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j1_controller/command", 1000);
        joint_2_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j2_controller/command", 1000);
        joint_3_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j3_controller/command", 1000);
        joint_4_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j4_controller/command", 1000);
        joint_5_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j5_controller/command", 1000);
        joint_6_pub = n.advertise<std_msgs::Float64>("/robot/right_joint_position_controller/joints/right_j6_controller/command", 1000);
        angle_0_sub = n.subscribe("/robot/joint_states",1000, joint_stateCallback);
        cuff_0_sub = n.subscribe("/io/robot/cuff/state",1000, cuff_1_callback);
    }
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
                    ROS_INFO("POS is %lf, %lf ,%lf, %lf, %lf, %lf, %lf",PP1[q-1].pose[0],PP1[q-1].pose[1],PP1[q-1].pose[2],PP1[q-1].pose[3],PP1[q-1].pose[4],PP1[q-1].pose[5],PP1[q-1].pose[6]);
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
                if (q>0)
                {
                    //inital position give 2s to move with 20ms interval
                    int i = 0;
                    int nn = 100;
                    for (p=0;p<100;p++)
                    {
                        replaypose(i,p,nn);
                        ros::Time start = ros::Time::now();
                        while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
                        {
                            movetotarget();
                            ros::spinOnce();
                        }
                    }
                    for (int i=1;i<q;i++)
                    {
                        nn = int((PP1[i].timestamp-PP1[i-1].timestamp).toSec()/0.02);
                        for (p=0;p<nn;p++)
                        {
                            replaypose(i,p,nn);
                            ros::Time start = ros::Time::now();
                            while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
                            {
                                movetotarget();
                                ros::spinOnce();
                            }
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


