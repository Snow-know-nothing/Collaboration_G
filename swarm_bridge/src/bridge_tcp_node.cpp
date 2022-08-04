#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
//#include <custom_msgs/cloud_xyz.h>
#include <unistd.h>
#include "swarm_bridge/reliable_bridge.hpp"
#include <sensor_msgs/PointCloud2.h>

double delta_time_;
bool topic_send_way_; //false single, true together
std::vector<ros::Publisher> imu_pub_;
std::vector<ros::Publisher> other_camera_detect_msg_pub_; 
std::vector<ros::Publisher> map_pub_; 
ros::Publisher all_other_imu_pub_;
ros::Publisher all_other_camera_detect_msg_pub_;
ros::Publisher all_other_map_pub_;


std::vector<int> id_list_;
std::vector<std::string> ip_list_;
ros::Subscriber imu_sub_, camera_detect_xyz_sub_,map_sub_;


int self_id_;
int self_id_in_bridge_;
int device_num_;
int ground_station_num_;
double broadcast_freq_;
bool is_groundstation_;
std::unique_ptr<ReliableBridge> bridge;


inline int remap_ground_station_id(int id)
{
  return id + device_num_;
}

inline int remap_frome_frameid_to_id(std::string frame_id)
{
  //frame_id formate is device_ + num;
  if(frame_id.length() < 8)
  {
    ROS_ERROR("Frame ID ERROE(1)");
    return -1;
  }
  std::string str_num = frame_id.substr(7);
  int id = std::atoi(str_num.c_str());
  return id;
}

template <typename T>
int send_to_all_device_except_me(std::string topic, T &msg)
{
  int err_code = 0; 
  for (int i = 0; i < device_num_; ++i)// Only send to all devices.
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(i,topic,msg);
    if(err_code< 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

template <typename T>
int send_to_all_groundstation_except_me(std::string topic, T &msg)
{
  int err_code = 0;
  for (int i = 0; i < ground_station_num_; ++i)// Only send to all groundstations.
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(ind,topic,msg);
    if(err_code < 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

void register_callbak_to_all_groundstation(std::string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < ground_station_num_; ++i)
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(ind,topic_name,callback);
  }
}

void register_callbak_to_all_devices(std::string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < device_num_; ++i)
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(i,topic_name,callback);
  }
}
 
void delta_time_callback(const std_msgs::Float64ConstPtr& delta_time_msg)
{
    delta_time_ = delta_time_msg -> data;
}

// Here is callback from local topic.
void imu_sub_tcp_callback(const sensor_msgs::ImuConstPtr &msg)
{
  sensor_msgs::Imu imu_msg = *msg;
  double t = imu_msg.header.stamp.toSec();
  imu_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  imu_msg.header.frame_id = std::string("device_") + std::to_string(self_id_);
  std::cout << "pub imu!"<<std::endl;
  // other_odoms_pub_.publish(imu_msg); // send to myself
  send_to_all_device_except_me("/imu",imu_msg);// Only send to devices.
  send_to_all_groundstation_except_me("/imu",imu_msg);// Only send to ground stations.
}

void map_sub_tcp_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  sensor_msgs::PointCloud2 map_msg = *msg;
  double t = map_msg.header.stamp.toSec();
  map_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  map_msg.header.frame_id = std::string("device_") + std::to_string(self_id_);

  // other_odoms_pub_.publish(map_msg); // send to myself
  send_to_all_device_except_me("/map",map_msg);// Only send to devices.
  send_to_all_groundstation_except_me("/map",map_msg);// Only send to ground stations.
}

// void camera_dedtect_xyz_sub_tcp_callback(const custom_msgs::cloud_xyzConstPtr &msg)
// {
//   custom_msgs::cloud_xyz xyz_msg = *msg;
//   double t = xyz_msg.header.stamp.toSec();
//   xyz_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
//   xyz_msg.header.frame_id = std::string("device_") + std::to_string(self_id_);
//   send_to_all_device_except_me("/camera_detect_xyz",xyz_msg);// Only send to devices.
//   send_to_all_groundstation_except_me("/camera_detect_xyz",xyz_msg);// Only send to ground stations.
// }

// Here is callback when the brodge received the data from others.
void imu_bridge_callback(int ID, ros::SerializedMessage& m)
{

  //这里的ID和下面的recv_device_id是一样的，抓要是由于每个线程监视一个ip，每个ip和设备是绑定的。
  sensor_msgs::Imu imu_msg;
  ros::serialization::deserializeMessage(m,imu_msg);
  imu_msg.header.stamp = ros::Time().fromSec(imu_msg.header.stamp.toSec() + delta_time_);
  int recv_device_id = remap_frome_frameid_to_id(imu_msg.header.frame_id);
  std::cout << "check if imu ID equal to recv_device_id" << std::endl;
  std::cout << "ID is " << ID << std::endl;
  std::cout << "recv_device_id " << recv_device_id << std::endl;
  if(recv_device_id < 0 || recv_device_id >= device_num_)
  {
    ROS_ERROR("Frame ID ERROE(2)");
    return;
  }
  if(topic_send_way_)
  {
    all_other_imu_pub_.publish(imu_msg);
    return;
  }
  imu_pub_[recv_device_id].publish(imu_msg);
}


void map_bridge_callback(int ID, ros::SerializedMessage& m)
{

  //这里的ID和下面的recv_device_id是一样的，抓要是由于每个线程监视一个ip，每个ip和设备是绑定的。
  sensor_msgs::PointCloud2 map_msg;
  ros::serialization::deserializeMessage(m,map_msg);
  map_msg.header.stamp = ros::Time().fromSec(map_msg.header.stamp.toSec() + delta_time_);
  int recv_device_id = remap_frome_frameid_to_id(map_msg.header.frame_id);
  std::cout << "check if map ID equal to recv_device_id" << std::endl;
  std::cout << "ID is " << ID << std::endl;
  std::cout << "recv_device_id " << recv_device_id << std::endl;
  if(recv_device_id < 0 || recv_device_id >= device_num_)
  {
    ROS_ERROR("Frame ID ERROE(2)");
    return;
  }
  if(topic_send_way_)
  {
    all_other_map_pub_.publish(map_msg);
    return;
  }
  map_pub_[recv_device_id].publish(map_msg);
}

// void camera_detect_xyz_bridge_callback(int ID, ros::SerializedMessage& m)
// {
//   custom_msgs::cloud_xyz xyz_msg;
//   ros::serialization::deserializeMessage(m,xyz_msg);
//   xyz_msg.header.stamp = ros::Time().fromSec(xyz_msg.header.stamp.toSec() + delta_time_);
//   int recv_device_id = remap_frome_frameid_to_id(xyz_msg.header.frame_id);
//   if(recv_device_id < 0 || recv_device_id >= device_num_)
//   {
//     ROS_ERROR("Frame ID ERROE(3)");
//     return;
//   }
//   if(topic_send_way_)
//   {
//     all_other_camera_detect_msg_pub_.publish(xyz_msg);
//     return;
//   }
//   other_camera_detect_msg_pub_[recv_device_id].publish(xyz_msg);

// }



int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  nh.param("self_id", self_id_, -1);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("device_num", device_num_, 0);
  nh.param("max_freq", broadcast_freq_, 1000.0);
  nh.param("topic_send_way", topic_send_way_, false);

  id_list_.resize(device_num_ + ground_station_num_);
  ip_list_.resize(device_num_ + ground_station_num_);

  for (int i = 0; i < device_num_ + ground_station_num_; ++i)
  {
    nh.param((i < device_num_ ? "device_ip_" + to_string(i) : "ground_station_ip_" + to_string(i-device_num_)), ip_list_[i], std::string("127.0.0.1"));
    id_list_[i]=i;
  }  
  self_id_in_bridge_ = self_id_;
  if (is_groundstation_)
  {
    self_id_in_bridge_ = remap_ground_station_id(self_id_);
  }
  //the ground statation ID = self ID + device_num_
  if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99)
  {
    ROS_WARN("[swarm bridge] Wrong self_id!");
    exit(EXIT_FAILURE);
  }

  ros::Subscriber delta_time_sub = nh.subscribe("/delta_time_topic", 1, delta_time_callback);
  imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu_topic", 10, imu_sub_tcp_callback, ros::TransportHints().tcpNoDelay());
  map_sub_  = nh.subscribe<sensor_msgs::PointCloud2>("/lio_sam/mapping/map_global", 10,map_sub_tcp_callback, ros::TransportHints().tcpNoDelay());

  //camera_detect_xyz_sub_ = nh.subscribe<custom_msgs::cloud_xyz>("/cloud_xyz_topic", 10, camera_dedtect_xyz_sub_tcp_callback, ros::TransportHints().tcpNoDelay());
  all_other_imu_pub_ = nh.advertise<sensor_msgs::Imu>("/other_device_imu", 1);
  //all_other_camera_detect_msg_pub_ = nh.advertise<custom_msgs::cloud_xyz>("/other_camera_detect_xyz", 1);
  all_other_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/other_device_map", 1);

  std::string topic_head;
  topic_head = "";
  imu_pub_.resize(device_num_);
  other_camera_detect_msg_pub_.resize(device_num_);
  map_pub_.resize(device_num_);
  for(int i = 0; i < device_num_;++i)
  {
    std::string imu_topic, other_camera_msg_topic, map_topic;
    imu_topic = topic_head + "/imu" + std::to_string(i) + "_topic"; 
    //other_camera_msg_topic = topic_head + "/camera" + std::to_string(i) + "_detect_points_topic";
    map_topic =  topic_head + "/map" + std::to_string(i) + "_topic"; 
    imu_pub_[i] = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
    //other_camera_detect_msg_pub_[i] = nh.advertise<custom_msgs::cloud_xyz>(other_camera_msg_topic, 1);
    map_pub_[i] =  nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1);
  }
  //initalize the bridge 
  bridge.reset(new ReliableBridge(self_id_in_bridge_,ip_list_,id_list_,100000));    

  register_callbak_to_all_devices("/imu", imu_bridge_callback);
 // register_callbak_to_all_devices("/camera_detect_xyz", camera_detect_xyz_bridge_callback);
  register_callbak_to_all_devices("/map", map_bridge_callback);

  ros::spin();
  bridge->StopThread();
  return 0;


}

