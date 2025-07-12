#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <deque>
#include <numeric>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#define NUM_FISH 2
ros::Time last_time;

double speed_msg[NUM_FISH][3];
double fish_pos[NUM_FISH][3];
// it's the speed of the bluerov2 relative to the fish,v_blue-v_fish
double relative_speed[3] = {0, 0, 0};
double partial_hx[3] = {0, 0, 0};
// it's just linear pos and vel of the bluerov2
double bluerov2_states[6] = {0, 0, 0, 0, 0, 0};

geometry_msgs::TransformStamped fish_tf;
std::vector<double> factor(NUM_FISH, 0.0);

// 定义一个队列来存储最近的速度值
std::vector<std::deque<double>> speed_x_history(NUM_FISH);
std::vector<std::deque<double>> speed_y_history(NUM_FISH);
std::vector<std::deque<double>> speed_z_history(NUM_FISH);
const size_t window_size = 10; // moving average window size

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  // std::cout <<"here3"<<std::endl;
  // std::cout << "modelStatesCallback" << std::endl;
  static tf2_ros::TransformBroadcaster br;
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  int n = 0;
  // it's the transform of the fish in std frame
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == "fish1" || msg->name[i] == "fish2")
    {
      // std::cout << "find fish: " << msg->name[i] << std::endl;
      fish_tf.header.stamp = ros::Time::now();
      fish_tf.header.frame_id = "world";
      if (msg->name[i] == "fish2")
        fish_tf.child_frame_id = "fish2";
      else
        fish_tf.child_frame_id = "fish1";
      fish_tf.transform.translation.x = msg->pose[i].position.x;
      fish_tf.transform.translation.y = msg->pose[i].position.y;
      fish_tf.transform.translation.z = msg->pose[i].position.z;

      fish_tf.transform.rotation = msg->pose[i].orientation;


      // std::cout <<"no prob" <<std::endl;
      double speed_x = (msg->pose[i].position.x - fish_pos[n][0]) / dt;
      double speed_y = (msg->pose[i].position.y - fish_pos[n][1]) / dt;
      double speed_z = (msg->pose[i].position.z - fish_pos[n][2]) / dt;

      fish_pos[n][0] = msg->pose[i].position.x;
      fish_pos[n][1] = msg->pose[i].position.y;
      fish_pos[n][2] = msg->pose[i].position.z;

      // add speed into query
      speed_x_history[n].push_back(speed_x);
      speed_y_history[n].push_back(speed_y);
      speed_z_history[n].push_back(speed_z);

      // if window is full and new comes in, we remove the oldest one
      if (speed_x_history[n].size() > window_size)
        speed_x_history[n].pop_front();
      if (speed_y_history[n].size() > window_size)
        speed_y_history[n].pop_front();
      if (speed_z_history[n].size() > window_size)
        speed_z_history[n].pop_front();

      // we don't calculate here
      // double avg_speed_x = std::accumulate(speed_x_history.begin(), speed_x_history.end(), 0.0) / speed_x_history.size();
      // double avg_speed_y = std::accumulate(speed_y_history.begin(), speed_y_history.end(), 0.0) / speed_y_history.size();
      // double avg_speed_z = std::accumulate(speed_z_history.begin(), speed_z_history.end(), 0.0) / speed_z_history.size();
      // std::cout <<"here4"<<std::endl;

      // ROS_INFO("Average Speed: x=%.2f, y=%.2f, z=%.2f", avg_speed_x, avg_speed_y, avg_speed_z);
      speed_msg[n][0] = std::accumulate(speed_x_history[n].begin(), speed_x_history[n].end(), 0.0) / speed_x_history[n].size();
      speed_msg[n][1] = std::accumulate(speed_y_history[n].begin(), speed_y_history[n].end(), 0.0) / speed_y_history[n].size();
      speed_msg[n][2] = std::accumulate(speed_z_history[n].begin(), speed_z_history[n].end(), 0.0) / speed_z_history[n].size();
      n++;
    

      br.sendTransform(fish_tf);

    }
  }
  last_time = current_time;
}

void bluerov2SpeedCallback(const nav_msgs::Odometry &msg)
{
  // this also gives the sts in std frame
  bluerov2_states[0] = msg.pose.pose.position.x;
  bluerov2_states[1] = msg.pose.pose.position.y;
  bluerov2_states[2] = msg.pose.pose.position.z;
  bluerov2_states[3] = msg.twist.twist.linear.x;
  bluerov2_states[4] = msg.twist.twist.linear.y;
  bluerov2_states[5] = msg.twist.twist.linear.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fish_tf_broadcaster");
  ros::NodeHandle nh;
  // we don't need to publish the speed, if want to debug, just uncomment this
  // ros::Publisher speed_pub = nh.advertise<geometry_msgs::Vector3>("speed", 10);

  ros::Publisher factor_pub = nh.advertise<std_msgs::Float64MultiArray>("factor", 10);
  std_msgs::Float64MultiArray factor_msg;

  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
  ros::Subscriber bluerov2_speed_sub = nh.subscribe("/bluerov2/pose_gt", 10, bluerov2SpeedCallback);

  last_time = ros::Time::now();

  ros::Rate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    for (int j = 0; j < NUM_FISH; j++)
    {
      relative_speed[0] = bluerov2_states[3] - speed_msg[j][0];
      relative_speed[1] = bluerov2_states[4] - speed_msg[j][1];
      relative_speed[2] = bluerov2_states[5] - speed_msg[j][2];

      partial_hx[0] = 2 * (bluerov2_states[0] - fish_pos[j][0]);
      partial_hx[1] = 2 * (bluerov2_states[1] - fish_pos[j][1]);
      partial_hx[2] = 2 * (bluerov2_states[2] - fish_pos[j][2]);

      factor[j] = partial_hx[0] * relative_speed[0] + partial_hx[1] * relative_speed[1] + partial_hx[2] * relative_speed[2];
      if (factor[j] >= 0)
        factor[j] = 0;
      else
        factor[j] = -factor[j] / (std::sqrt(partial_hx[0] * partial_hx[0] +
                                            partial_hx[1] * partial_hx[1] +
                                            partial_hx[2] * partial_hx[2]));
    }
    // speed_pub.publish(speed_msg);
    // std::cout <<"here1"<<std::endl;
    factor_msg.data = factor;
    // std::cout << "factor:       ";
    // for (const auto &value : factor)
    // {
    //   std::cout << value << " ";
    // }
    // std::cout << std::endl;
    factor_pub.publish(factor_msg);
    rate.sleep();
  }
  return 0;
}
