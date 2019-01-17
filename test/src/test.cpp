#include "sstream"
#include "vector"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

using namespace std;

int main(int argc, char **argv)
{
    //step 1
    ROS_INFO("ok");
    double t[4]={0,1,2,3};
    double y[4]={1,exp(1),exp(2),exp(3)};
    vector<double> h,alpha,l,u,z,a,b,c,d,x;
    ROS_INFO("ok");
    double h_i,alpha_i,l_i,u_i,z_i,a_i,b_i,c_i,d_i;
    int n=3;
    int i;
    ROS_INFO("ok");
    for (int i=0;i<n+1;i++)
    {
        a.push_back(y[i]);
        x.push_back(t[i]);
        h_i = 1;
        h.push_back(h_i);
    }
    ROS_INFO("h %lf, %lf",h[0],h[1]);
    //step 2
    for (int i=1;i<n;i++)
    {
        alpha_i = 3/h[i]*(a[i+1]-a[i])-3/h[i-1]*(a[i]-a[i-1]);
        alpha.push_back(alpha_i);
    }

    ROS_INFO("alpha %lf",alpha[0]);
    //step 3
    l.push_back(1);
    u.push_back(0);
    z.push_back(0);
    //step 4

    ROS_INFO("ok");
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

    ROS_INFO("z %lf, %lf, %lf",z[0],z[1],z[2]);

    ROS_INFO("u %lf, %lf",u[0],u[1]);

    ROS_INFO("l %lf, %lf",l[0],l[1]);
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

    ROS_INFO("b %lf, %lf",b[1],b[0]);
    ROS_INFO("c %lf, %lf",c[2],c[1]);

    ROS_INFO("d %lf, %lf",d[1],d[0]);
    ROS_INFO("sizes %d, %d, %d, %d",a.size(),b.size(),c.size(),d.size());
    //Save all the coefficients
    //But remember a is a0-an b-d are bn-b0 cn-c0 dn-d0, in other words reversly
    // //clear vectors
    for (int i=0;i<n;i++)
    {
        ROS_INFO("The coefficients are %lf, %lf, %lf, %lf",a[i],b[n-i-1],c[n-i],d[n-i-1]);
    }

    ROS_INFO("ok");
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