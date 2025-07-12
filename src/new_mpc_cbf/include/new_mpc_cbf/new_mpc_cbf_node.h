#ifndef NEW_MPC_CBF_INCLUDE_NEW_MPC_CBF_NODE_H
#define NEW_MPC_CBF_INCLUDE_NEW_MPC_CBF_NODE_H
#endif

/*  this is for casadi   */
#include <iostream>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <casadi/casadi.hpp>

// basic ros and it's msgs
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

// tf2
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
// this is needed！！
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// eigen for mtimes in tau to thrust forces
#include <Eigen/Core>


#define N_OB 6
#define N_U 6

namespace new_mpc_cbf
{
    class NewMpcCbfNode
    {
    public:
        NewMpcCbfNode(ros::NodeHandle &nh, double tc);
        void run(void);

    private:
        ros::NodeHandle nh_;
        std::vector<double> bluerov2_states;
        std::vector<double> bluerov2_input;
        std::vector<double> target_state_value;
        double cpu_time;
        /***************************************************/
        /*important params which may need to be changed*/
        int horizon_ = 6;
        double tc_ = 0.01;
        double k_alpha_1 = 960;
        double k_alpha_2 = 80;
        std::vector<double> lbu;
        std::vector<double> ubu;

        casadi::Function mpc_cbf_solver;

        // position and angle
        geometry_msgs::Twist std_eta;

        // topic & msgs
        std_msgs::Float64 solve_time;
        ros::Publisher solve_time_pub;
        geometry_msgs::Wrench control_input_std;
        ros::Publisher control_std_pub;
        geometry_msgs::Vector3 linear_control;
        geometry_msgs::Vector3 angular_control;
        ros::Publisher linear_control_pub;
        ros::Publisher angular_control_pub;
        ros::Publisher eta_pub;
        geometry_msgs::Vector3 cylinder_h;
        geometry_msgs::Vector3 sphere_h;
        ros::Publisher cylinder_pub;
        ros::Publisher sphere_pub;
        ros::Subscriber br2_state_sub;

        visualization_msgs::Marker marker;
        ros::Publisher freq_marker_pub;
        // obstacles in std frame!
        double obstacles_std[N_OB][8] =
            {{1, 30.5,14,-95, 0, 0, 0, 2.6+0.3},
            {1, 33, 11, -95, 0, 0, 0, 1.2+0.3},
             {0, 24, 13, -95, 0, 0, 0, 1+0.3},
             {0, 28, 10, -95, 0, 0, 1.57, 1+0.3},
             {0, 30.9, 8, -95, 0, 0, 1.57, 1+0.3},
             {1, 25, 8, -95, 0, 0, 1, 1.2+0.3}};
        double obstacles[N_OB][8] =
            {{1, 30.5,-14,95, 0, 0, 0, 2.9},
            {1, 33, -11, 95, 0, 0, 0, 1.5},
             {0, 24, -13, 95, 0, 0, 0, 1.3},
             {0, 28, -10, 95, 0, 0, 1.57, 1.3},
             {0, 30.9, -8, 95, 0, 0, 1.57, 1.3},
             {1, 25, -8, 95, 0, 0, 1, 1.2}};
        // double obstacles_std[1][8] =
        //     { {1, 30.5,15,-95, 0, 0, 0, 2.6+0.6}};
        // double obstacles[1][8] =
        //     {  {1, 30.5,-15,95, 0, 0, 0, 3.2}};
        enum EnumStates
        {
            x = 0,
            y = 1,
            z = 2,
            phi = 3,
            theta = 4,
            psi = 5,
            u = 6,
            v = 7,
            w = 8,
            p = 9,
            q = 10,
            r = 11,
        };

        void pose_gt_cb(const nav_msgs::Odometry &msg);
        void control_pub(void);
    };

};
