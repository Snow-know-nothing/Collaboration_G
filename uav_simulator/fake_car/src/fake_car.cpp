#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "fake_car/IDodom.h"

#define PI 3.14159265359
using namespace std;

ros::Publisher car_odom_pub_, traj_pub_, predicted_traj_pub_,odom_broadcast;
ros::Timer broadcast_timer_;
int car_id;
double HEIGHT = 0.0;

class moving_obstacle
{
private:

  ros::Time t_last_update_{ros::Time(0)};
public:

  Eigen::Vector2d pos_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d vel_{Eigen::Vector2d::Zero()};
  double yaw_{0};
  double yaw_rate{0};
  
  double des_clearance_;

  moving_obstacle(){};
  ~moving_obstacle(){};

  void set_position(Eigen::Vector2d pos)
  {
    pos_ = pos;
  }

  double get_yaw() { return yaw_; }
  
  void dyn_update(const double delta_t, const double v, const double w) 
  {
    pos_ += vel_ * delta_t;
    yaw_ += yaw_rate * delta_t;
    if(yaw_>PI)
    {
      yaw_ = yaw_-2*PI;
    }
    else if(yaw_<-PI)
    {
      yaw_ = 2*PI+yaw_;
    }
    vel_ = v * Eigen::Vector2d(cos(yaw_), sin(yaw_));
    yaw_rate = w;
  }

  std::pair<Eigen::Vector2d, Eigen::Vector2d> update(const double vel, const double yaw_rate)
  {
    ros::Time t_now = ros::Time::now();
    if (t_last_update_ == ros::Time(0))
    {
      t_last_update_ = t_now;
    }

    double delta_t = (t_now - t_last_update_).toSec();
    dyn_update(delta_t, vel, yaw_rate);

    t_last_update_ = t_now;
    return std::pair<Eigen::Vector2d, Eigen::Vector2d>(pos_, vel_);
  }

  std::pair<Eigen::Vector2d, Eigen::Vector2d> predict(const double acc, const double dir, double predict_t) const
  {
    constexpr double STEP = 0.1;
    double yaw = yaw_;
    Eigen::Vector2d pos = pos_;
    Eigen::Vector2d vel = vel_;

    for (double t = STEP; t <= predict_t; t += STEP)
    {
      //dyn_update(STEP, acc, dir, yaw, pos, vel);
    }

    return std::pair<Eigen::Vector2d, Eigen::Vector2d>(pos, vel);
  }
};

moving_obstacle car_;

// #      ^                ^
// #    +1|              +4|
// # <-+0      ->     <-+3      ->
// #      |                |
// #      V                V

void joy_sub_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  ros::Time t_now = ros::Time::now();

  double acc1 = msg->axes[1] * 2;
  double dir1 = msg->axes[0] / 3;
  double acc2 = msg->axes[4] * 2;
  double dir2 = msg->axes[3] / 3;
  if (acc1 < 0)
    dir1 = -dir1;
  if (acc2 < 0)
    dir2 = -dir2;

  auto pv1 = car_.update(acc1, dir1);

  //constexpr double HEIGHT = 1.0;

  // publish odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = t_now;
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.position.z = HEIGHT;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;

  Eigen::Quaterniond q1(Eigen::AngleAxisd(car_.get_yaw(), Eigen::Vector3d::UnitZ()));
  odom_msg.pose.pose.position.x = pv1.first(0);
  odom_msg.pose.pose.position.y = pv1.first(1);
  odom_msg.twist.twist.linear.x = pv1.second(0);
  odom_msg.twist.twist.linear.y = pv1.second(1);
  odom_msg.pose.pose.orientation.w = q1.w();
  odom_msg.pose.pose.orientation.z = q1.z();
  car_odom_pub_.publish(odom_msg);

  // publish predicted trajectory

}

void key_sub_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
  double v = msg->linear.x;
  double w = msg->angular.z;

  auto pv1 = car_.update(v,w);
}

void pubOdom()
{	
  ros::Time t_now = ros::Time::now();
  //constexpr double HEIGHT = 1.0;
  // publish odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = t_now;
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.position.z = HEIGHT;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;

  Eigen::Quaterniond q1(Eigen::AngleAxisd(car_.yaw_, Eigen::Vector3d::UnitZ()));
  odom_msg.pose.pose.position.x = car_.pos_(0);
  odom_msg.pose.pose.position.y = car_.pos_(1);
  odom_msg.twist.twist.linear.x = car_.vel_(0);
  odom_msg.twist.twist.linear.y = car_.vel_(1);
  odom_msg.pose.pose.orientation.w = q1.w();
  odom_msg.pose.pose.orientation.z = q1.z();

  car_odom_pub_.publish(odom_msg);
}

void odom_broadcastCallback(const ros::TimerEvent &e)
{
  ros::Time t_now = ros::Time::now();
  fake_car::IDodom idodom;
  idodom.id = car_id;
  idodom.odom.header.stamp = t_now;
  idodom.odom.header.frame_id = "world";
  idodom.odom.pose.pose.position.z = HEIGHT;
  idodom.odom.twist.twist.linear.z = 0.0;
  idodom.odom.pose.pose.orientation.x = 0.0;
  idodom.odom.pose.pose.orientation.y = 0.0;

  Eigen::Quaterniond q1(Eigen::AngleAxisd(car_.yaw_, Eigen::Vector3d::UnitZ()));
  idodom.odom.pose.pose.position.x = car_.pos_(0);
  idodom.odom.pose.pose.position.y = car_.pos_(1);
  idodom.odom.twist.twist.linear.x = car_.vel_(0);
  idodom.odom.twist.twist.linear.y = car_.vel_(1);
  idodom.odom.pose.pose.orientation.w = q1.w();
  idodom.odom.pose.pose.orientation.z = q1.z();

  odom_broadcast.publish(idodom);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_car");
  ros::NodeHandle nh("~");

  double init_x,init_y;
  // nh.getParam("car_init_pos", init_pos);

  nh.param("init_x", init_x,  0.0);
  nh.param("init_y", init_y,  0.0);
  nh.param("id", car_id,  -1);
  //nh.getParam("height", HEIGHT);
 
  broadcast_timer_ = nh.createTimer(ros::Duration(0.08), &odom_broadcastCallback);

  car_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_car", 10);
  odom_broadcast = nh.advertise<fake_car::IDodom>("/broadcast_odom", 10);

  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joy_sub_cb);
  ros::Subscriber key_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, key_sub_cb);

  car_.set_position(Eigen::Vector2d(init_x, init_y));

  while (ros::ok())
  {
   pubOdom();        
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return 0;
}
