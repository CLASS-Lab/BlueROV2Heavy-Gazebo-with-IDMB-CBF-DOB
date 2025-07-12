#ifndef NEW_MPC_CBF_DOB_INCLUDE_NEW_MPC_CBF_DOB_NODE_H
#define NEW_MPC_CBF_DOB_INCLUDE_NEW_MPC_CBF_DOB_NODE_H
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
#include <tf2_eigen/tf2_eigen.h>

// eigen for mtimes in tau to thrust forces
#include <Eigen/Core>

#define N_OB 6
#define N_U 6
#define N_X 12
namespace new_mpc_cbf_dob
{
    class NewMpcCbfDobNode
    {
    public:
        NewMpcCbfDobNode(ros::NodeHandle &nh, double tc);
        void run(void);

    private:
        ros::NodeHandle nh_;
        std::vector<double> bluerov2_states;
        std::vector<double> bluerov2_input;
        // last output
        std::vector<double> last_tau_out;
        std::vector<double> target_state_value;
        // double d_hat[N_U]={0,0,0,0,0,0};
        std::vector<double> d_hat;
        // DOB states
        // double z_hat[N_U]={0,0,0,0,0,0};
        std::vector<double> z_hat;
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
        // 0 is cylinder, 1 is sphere
        double obstacles_std[N_OB][8] =
            {{1, 33, 11, -95, 0, 0, 0, 1.2 + 0.6},
             {0, 24, 13, -95, 0, 0, 0, 1 + 0.6},
             {1, 30.5, 14, -95, 0, 0, 0, 2.6 + 0.6},
             {0, 28, 10, -95, 0, 0, 0, 1 + 0.6},
             {0, 30.9, 8, -95, 0, 0, 0, 1 + 0.6},
             {1, 25, 8, -95, 0, 0, 0, 1.2 + 0.6}};
        double obstacles[N_OB][8] =
            {{1, 33, -11, 95, 0, 0, 0, 1.8},
             {0, 24, -13, 95, 0, 0, 0, 1.6},
             {1, 30.5, -14, 95, 0, 0, 0, 3.2},
             {0, 28, -10, 95, 0, 0, 0, 1.6},
             {0, 30.9, -8, 95, 0, 0, 0, 1.6},
             {1, 25, -8, 95, 0, 0, 0, 1.8}};
        /***************************************************/
        //  this is for DOB
        ros::Time last_time;
        // factor for l_mat
        double l_factor = 0.4;
        double alpha = 0.45;
        double omega_1 = 5;
        double beta = 200;
        double v1 = 0.4;
        double p2;

        geometry_msgs::Twist d_hat_msg;
        ros::Publisher d_hat_pub;
        enum EnumMParams
        {
            mass = 0,
            Ix = 1,
            Iy = 2,
            Iz = 3,
            Xu = 4,
            Yv = 5,
            Zw = 6,
            Kp = 7,
            Mq = 8,
            Nr = 9,
        };
        double M_params[10] = {13.5,
                               0.26, 0.23, 0.37,
                               13.7, 0.0, 33.0,
                               0.0, 0.8, 0.0};
        double p_data[N_U] = {0., 0., 0., 0., 0., 0.};
        double m = 13.5, g = 9.82, W = m * g;
        double rho = 1000, volume = 0.0134, B = rho * g * volume, zb = 0.01,
               // Quadratic damping coefficients
            Xuu = 141.0, Yvv = 217.0, Zww = 190.0, Kpp = 1.19, Mqq = 0.47, Nrr = 1.5,
               // Added mass coefficients (Ca coefs)
            Xud = 6.36, Yvd = 7.12, Zwd = 18.68, Kpd = 0.189, Mqd = 0.135, Nrd = 0.222;
        double M_diag_data[N_U] = {
            M_params[EnumMParams::mass] + Xud,
            M_params[EnumMParams::mass] + Yvd,
            M_params[EnumMParams::mass] + Zwd,
            M_params[EnumMParams::Ix] + Kpd,
            M_params[EnumMParams::Iy] + Mqd,
            M_params[EnumMParams::Iz] + Nrd};
        Eigen::DiagonalMatrix<double, N_U> M_mat;

        Eigen::VectorXd dob_state_equation(Eigen::VectorXd z_hat);
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
        void dob_euler(double dt);
        void pose_gt_cb(const nav_msgs::Odometry &msg);
        void control_pub(void);
    };

};
