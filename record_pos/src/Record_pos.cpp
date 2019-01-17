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
struct Pose_array
{
    double pose [7];
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
void splinefit(int);
void splinereplay(int, int, int);
int c;
int q = 0;
int mode;
int low_button_status [2] = {0,0};
int up_button = 0;//record mode
int low_button = 0;//record now
//define ros publisher and subscriber on sawyer
ros::Publisher joint_0_pub, joint_1_pub, joint_2_pub, joint_3_pub, joint_4_pub, joint_5_pub, joint_6_pub;
ros::Subscriber angle_0_sub, cuff_0_sub; 
//the vector to save poses
vector<struct Joint_status> PP1;
vector<vector<double> > coe;

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
    //Real robot mode
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
    //Simulator mode but a a program to receive button signal.
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
    //Record poses and time stamps
    struct Joint_status t;
    for(int i=0;i<7;i++)
    {
        t.pose[i] = cur_pos[i];
        t.timestamp = ros::Time::now();
        ROS_INFO("POS is %lf",t.pose[i]);
    }
    PP1.push_back(t);
}
void replaypose(int i, int p, int n)
{   
    //Replay pose with average speed. Will be good used with only two poses.
    //The released time interval should be 20 ms. Then the poses are published in (t[i+1]-t[i]) in sec/ 20 ms segments. 
    if (i==0)
    {   
        for (int j=0;j<7;j++)
        {
            angles[j] = PP1.back().pose[j]+(PP1[i].pose[j]-PP1.back().pose[j])/n*p;
        }
    }
    else
    {
        for (int j=0;j<7;j++)
        {
            angles[j] = PP1[i-1].pose[j]+(PP1[i].pose[j]-PP1[i-1].pose[j])/n*p;
        }
    }
}
void splinereplay(int i, int p, int n)
{
    double t;//time
    //We could choose time position control or acceleration fix 3758 row in Servo86.cpp by Chi-Ju
    t = (PP1[i].timestamp).toSec()+p*0.02;
    for (int j=0;j<7;j++)
    {
        angles[j] = coe[4*j][i]+coe[4*j+1][n-i-1]*t+coe[4*j+2][n-i]*t*t+coe[4*j+3][n-i-1]*t*t*t;
    }
}
void splinefit(int n)
{
    //according to cubic_splines.pdf 
    //For the fit Sj(x) = aj+bj(x-xj)+cj(x-xj)^2+dj(x-xj)^3
    vector<double> h,alpha,l,u,z,a,b,c,d,x;
    double h_i,alpha_i,l_i,u_i,z_i,a_i,b_i,c_i,d_i;
    for (int j=0;j<7;j++)
    {
        //step 1
        for (int i=0;i<n+1;i++)
        {
            a.push_back(PP1[i].pose[j]);
            x.push_back((PP1[i].timestamp).toSec());
        }
        for (int i=0;i<n;i++)
        {
            h_i = (PP1[i+1].timestamp-PP1[i].timestamp).toSec();
            h.push_back(h_i);
        }
        //step 2
        for (int i=1;i<n;i++)
        {
            alpha_i = 3/h[i]*(a[i+1]-a[i])-3/h[i-1]*(a[i]-a[i-1]);
            alpha.push_back(alpha_i);
        }
        //step 3
        l.push_back(1);
        u.push_back(0);
        z.push_back(0);
        //step 4
        for (int i=1;i<n;i++)
        {
            l_i = 2*(x[i+1]-x[i-1])-h[i-1]*u[i-1];
            l.push_back(l_i);
            u_i = h[i]/l[i];
            u.push_back(u_i);
            z_i = (alpha[i-1]-h[i-1]*z[i-1])/l[i];
            z.push_back(z_i);
        }
        //step 5
        l.push_back(1);
        z.push_back(0);
        c.push_back(0);//b,c,d is a reverse vector of one joint in BB,CC,DD;But a is in the right order
        //step 6
        for (int i=n-1;i>=0;i--)
        {
            c_i = z[i]-u[i]*c[n-(i+1)];
            c.push_back(c_i);
            b_i = (a[i+1]-a[i])/h[i]-h[i]*(c[n-(i+1)]+2*c[n-i])/3;
            b.push_back(b_i);
            d_i = (c[n-(i+1)]-c[n-i])/(3*h[i]);
            d.push_back(d_i);
        }
        //Save all the coefficients
        coe.push_back(a);
        coe.push_back(b);
        coe.push_back(c);
        coe.push_back(d);
        //But remember a is a0-an b-d are bn-b0 cn-c0 dn-d0, in other words reversly
        //clear vectors
        l.clear();
        u.clear();
        z.clear();
        h.clear();
        a.clear();
        b.clear();
        c.clear();
        d.clear();
        alpha.clear();
    }    
}
void cleardata()
{
    PP1.clear();
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
    int nn;
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
                    ROS_INFO("POS is %lf, %lf ,%lf, %lf, %lf, %lf, %lf",PP1.back().pose[0],PP1.back().pose[1],PP1.back().pose[2],PP1.back().pose[3],PP1.back().pose[4],PP1.back().pose[5],PP1.back().pose[6]);
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
        ROS_INFO("Choose mode:  1 for replay the poses with average speed.  2 for replay the poses with cubic spline interpolation. 3 for clean all poses. 3 for exit the program.");
        scanf("%d", &j);
        switch (j) 
        {
            case 1: 
            {   
                if (PP1.size()>0)
                {
                    //inital position give 2s to move with 20ms interval.
                    int i = 0;
                    int nn = 100;
                    for (p=1;p<=nn;p++)
                    {
                        replaypose(i,p,nn);
                        ros::Time start = ros::Time::now();
                        while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
                        {
                            movetotarget();
                            ros::spinOnce();
                        }
                    }
                    for (int i=1;i<PP1.size();i++)
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
            case 2:
            {
                //When only have 1 or 2 poses use average speed mode
                if (PP1.size() == 1)
                {
                    for (p=1;p<=nn;p++)
                    {
                        replaypose(1,p,nn);
                        ros::Time start = ros::Time::now();
                        while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
                        {
                            movetotarget();
                            ros::spinOnce();
                        }
                    }
                }
                if (PP1.size() == 2)
                {
                    nn = int((PP1[2].timestamp-PP1[1].timestamp).toSec()/0.02);
                    for (p=0;p<nn;p++)
                    {
                        replaypose(2,p,nn);
                        ros::Time start = ros::Time::now();
                        while(ros::ok() && !reach() && (ros::Time::now() - start).toSec() < 5)
                        {
                            movetotarget();
                            ros::spinOnce();
                        }
                    }    
                }
                if (PP1.size() > 2)
                {
                    //Get cubic spline coefficients first
                    splinefit(PP1.size()-1);
                    for (int i=0;i<PP1.size();i++)
                    {
                        nn = int((PP1[i+1].timestamp-PP1[i].timestamp).toSec()/0.02);
                        for (p=0;p<nn;p++)
                        {
                            splinereplay(i,p,PP1.size());
                        }
                    }
                }
            }
            case 3: {   cleardata();
                        break;}
            case 4: {   break;}
        }   
        if (j==3) 
            break;
    }
    return 0;
}


