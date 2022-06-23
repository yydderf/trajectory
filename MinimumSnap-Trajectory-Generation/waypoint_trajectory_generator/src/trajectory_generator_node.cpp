#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _way_pts_sub;
ros::Publisher _wp_traj_pub;

// for planning
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
Vector3d _startPos  = Vector3d::Zero();
Vector3d _startVel  = Vector3d::Zero();
Vector3d _startAcc  = Vector3d::Zero();
Vector3d _endVel    = Vector3d::Zero();
Vector3d _endAcc    = Vector3d::Zero();

// declare
void visWayPointTraj(MatrixXd polyCoeff, VectorXd time);
void visWayPointPath(MatrixXd path);
Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);
VectorXd timeAllocation(MatrixXd Path);
void trajGeneration(Eigen::MatrixXd path);
void rcvWaypointsCallBack(const nav_msgs::Path &wp);

//Get the path points 
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);
        ROS_INFO("waypoint%d: (%f, %f, %f)", k+1, pt(0), pt(1), pt(2));
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);  //add the original point
    waypoints.row(0) = _startPos;

    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    //Trajectory generation: use minimum jerk/snap trajectory generation method
    //waypoints is the result of path planning (Manual in this project)
    trajGeneration(waypoints);
}

void trajGeneration(Eigen::MatrixXd path)
{
    ros::Time time_start = ros::Time::now();

    TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;
    
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;

    // use "trapezoidal velocity" time allocation
    _polyTime  = timeAllocation(path);

    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);

    visWayPointTraj( _polyCoeff, _polyTime);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel", _Vel, 1.0);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("planning/dev_order", _dev_order, 3);  // the order of derivative, _dev_order = 3->minimum jerk, _dev_order = 4->minimum snap
    nh.param("planning/min_order", _min_order, 3);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    //_poly_numID is the maximum order of polynomial
    _poly_num1D = 2 * _dev_order;

    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );

    _wp_traj_pub = nh.advertise<nav_msgs::Path>("trajectory", 1);


    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{
    nav_msgs::Path path_msg;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    Vector3d pos;
    geometry_msgs::PoseStamped pt;

    for(int i = 0; i < time.size(); i++ )   // go through each segment
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.pose.position.x = pos(0);
          cur(1) = pt.pose.position.y = pos(1);
          cur(2) = pt.pose.position.z = pos(2);
          std::cout << pos(0) << " " << pos(1) << " " << pos(2) << std::endl;
          path_msg.poses.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }
    ROS_WARN("Trajectory length is %f m", traj_len);
    _wp_traj_pub.publish(path_msg);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);

    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        double t1 = _Vel / _Acc;
        double t2 = x2 / _Vel;
        time(i) = 2 * t1 + t2;
    }
    // cout << time << endl;

    return time;
}
