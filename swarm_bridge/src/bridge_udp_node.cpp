#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
//#include <custom_msgs/cloud_xyz.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <string.h>



#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define UDP_PORT 8081
#define BUF_LEN 10485760   // 10MB
#define BUF_LEN_SHORT 1024 // 1KB



bool topic_send_way_;
double odom_broadcast_freq_;
std::string udp_ip_;
int device_id_, device_num_;
int udp_server_fd_, udp_send_fd_;// 套接字描述符，
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;//目标地址信息结构

//recv msg
sensor_msgs::Imu udp_recv_imu_msg_;
//custom_msgs::cloud_xyz udp_recv_xyz_msg_;
sensor_msgs::PointCloud2 udp_recv_map_msg_;


ros::Subscriber imu_sub_, camera_detect_xyz_sub_,map_sub_;
std::vector<ros::Publisher> imu_pub_;
std::vector<ros::Publisher> other_camera_detect_msg_pub_;
std::vector<ros::Publisher> map_pub_;

ros::Publisher all_other_imu_pub_;
ros::Publisher all_other_camera_detect_msg_pub_;
ros::Publisher all_other_map_pub_;

double delta_time_;

enum MESSAGE_TYPE
{
    IMU = 100,
    DEETECT_XYZ,
    MAP
} massage_type_;

int init_broadcast(const char *ip, const int port)
{
    int fd;

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
    {
        ROS_ERROR("[bridge_node]Socket sender creation error!");
        exit(EXIT_FAILURE);
    }

    int so_broadcast = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
    {
        ROS_ERROR("Error in setting Broadcast option");
        exit(EXIT_FAILURE);
    }

    addr_udp_send_.sin_family = AF_INET;
    addr_udp_send_.sin_port = htons(port);//指定端口,需要将端口转换成大端字节序

    if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
{
    struct sockaddr_in address;
    int opt = 1;

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                    &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    //ipv4采用的结构体
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // Forcefully attaching socket to the port
    if (bind(server_fd, (struct sockaddr *)&address,
            sizeof(address)) < 0)//套接字描述符，
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    return server_fd;
}

template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
{
    //message stored order is type, size, msg (xun)    
    auto ptr = (uint8_t *)(udp_send_buf_);

    *((int*)ptr) = device_id_;
    ptr += sizeof(int);
    *((MESSAGE_TYPE*)ptr) = msg_type;
    ptr += sizeof(MESSAGE_TYPE);

    namespace ser = ros::serialization;
    uint32_t msg_size = ser::serializationLength(msg);

    *((uint32_t *)ptr) = msg_size;
    ptr += sizeof(uint32_t);

    ser::OStream stream(ptr, msg_size);
    ser::serialize(stream, msg);

    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t) + sizeof(int);
}

template <typename T>
int deserializeTopic(T &msg)
{
    auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE) + sizeof(int));

    uint32_t msg_size = *((uint32_t *)ptr);
    ptr += sizeof(uint32_t);

    namespace ser = ros::serialization;
    ser::IStream stream(ptr, msg_size);
    ser::deserialize(stream, msg);

    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t) + sizeof(int);
}

void imu_sub_udp_callback(const sensor_msgs::ImuConstPtr &msg)
{
    sensor_msgs::Imu imu_msg = *msg;
    double t = imu_msg.header.stamp.toSec();
    imu_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
    // static ros::Time t_last;
    // ros::Time t_now = ros::Time::now();
    // if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0)
    // {
    //     return;
    // }
    // t_last = t_now;
    int len = serializeTopic(MESSAGE_TYPE::IMU, imu_msg);
    if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
    {
        ROS_ERROR("UDP SEND ERROR (1)!!!");
    }
}



// void camera_dedtect_xyz_sub_udp_callback(const custom_msgs::cloud_xyzConstPtr &msg)
// {
//     custom_msgs::cloud_xyz xyz_msg = *msg;
//     double t = xyz_msg.header.stamp.toSec();
//     xyz_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
//     int len = serializeTopic(MESSAGE_TYPE::DEETECT_XYZ, xyz_msg);
//     if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
//     {
//         ROS_ERROR("UDP SEND ERROR (2)!!!");
//     }
// }

void map_sub_udp_callback(const sensor_msgs::PointCloud2ConstPtr  &msg)
{
    sensor_msgs::PointCloud2 map_msg = *msg;
    double t = map_msg.header.stamp.toSec();
    map_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
    int len = serializeTopic(MESSAGE_TYPE::MAP, map_msg);
    std::cout<< "length of map"<< len <<std::endl;
    int i = sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_));
    if (i <= 0)
    {
        std::cout <<i<<"  "<<errno<<strerror(errno)<<std::endl;
        ROS_ERROR("UDP SEND ERROR (3)!!!");
    }
}

void udp_recv_fun()
{
    int valread;
    struct sockaddr_in addr_client;
    socklen_t addr_len;

    // Connect
    if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
    {
        ROS_ERROR("[bridge_node]Socket recever creation error!");
        exit(EXIT_FAILURE);
    }

    while (true)
    {
        if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
        {
            perror("recvfrom() < 0, error:"); 

            exit(EXIT_FAILURE);
        }
        // std::cout<< "JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ" << std::endl;
        // std::cout << valread << std::endl;
        char *ptr = udp_recv_buf_;
        int recv_device_id = *((int *)ptr);//id
        ptr += sizeof(int);
        switch (*((MESSAGE_TYPE *)ptr))
        {

            case MESSAGE_TYPE::IMU:
            {
                if (valread == deserializeTopic(udp_recv_imu_msg_))
                {
                    udp_recv_imu_msg_.header.stamp = ros::Time().fromSec(udp_recv_imu_msg_.header.stamp.toSec() + delta_time_);
                    udp_recv_imu_msg_.header.frame_id = std::string("device_") + std::to_string(recv_device_id);
                    if(topic_send_way_ )
                    {
                        all_other_imu_pub_.publish(udp_recv_imu_msg_);
                    }
                    else
                    {
                        imu_pub_[recv_device_id].publish(udp_recv_imu_msg_);
                    }
                }
                else
                {
                    ROS_ERROR("Received message length not matches the sent one (1)!!!");
                    continue;
                }

                break;
            }
            case MESSAGE_TYPE::DEETECT_XYZ:
            {
                // if (valread == deserializeTopic(udp_recv_xyz_msg_))
                // {
                //     udp_recv_xyz_msg_.header.stamp = ros::Time().fromSec(udp_recv_xyz_msg_.header.stamp.toSec() + delta_time_);
                //     udp_recv_xyz_msg_.header.frame_id = std::string("device") + std::to_string(recv_device_id);
                //     if(topic_send_way_)
                //     {
                //         all_other_camera_detect_msg_pub_.publish(udp_recv_xyz_msg_);
                //     }
                //     else
                //     {
                //         other_camera_detect_msg_pub_[recv_device_id].publish(udp_recv_xyz_msg_);
                //     }
                // }
                // else
                // {
                //     ROS_ERROR("Received message length not matches the sent one (2)!!!");
                //     continue;
                // }Corner.

                // break;

            }
            case MESSAGE_TYPE::MAP:
            {
                if (valread == deserializeTopic(udp_recv_map_msg_))
                {
                    udp_recv_map_msg_.header.stamp = ros::Time().fromSec(udp_recv_map_msg_.header.stamp.toSec() + delta_time_);
                    udp_recv_map_msg_.header.frame_id = std::string("device_") + std::to_string(recv_device_id);
                    if(topic_send_way_)
                    {
                        all_other_map_pub_.publish(udp_recv_map_msg_);///这里
                    }
                    else
                    {
                        map_pub_[recv_device_id].publish(udp_recv_map_msg_);
                    }
                }
                else
                {
                    ROS_ERROR("Received message length not matches the sent one (3)!!!");
                    continue;
                }

                break;
                
            }
            default:
            {
                ROS_ERROR("Unknown received message type???");
                break;
            }
            
        }
    }
}


void delta_time_callback(const std_msgs::Float64ConstPtr& delta_time_msg)
{
    delta_time_ = delta_time_msg -> data;
}

int main(int argc, char **argv)
{


    // std::string test = "device_5";
    // std::cout << "???" << std::endl;
    // if(test.length() < 8)
    // {
    //     ROS_ERROR("Frame ID ERROE(1)");
    //     return -1;
    // }
    // std::string str_num = test.substr(7);
    // int id = std::atoi(str_num.c_str());
    // std::cout << id << std::endl;
    // std::cout << "!!!" << std::endl;
    ros::init(argc, argv, "swarm_bridge");
    ros::NodeHandle nh("~");

    nh.param("topic_send_way", topic_send_way_, false);
    nh.param("broadcast_ip", udp_ip_, std::string("127.0.0.255"));
    nh.param("device_id", device_id_, -1);
    nh.getParam("device_num", device_num_);
    nh.param("odom_max_freq", odom_broadcast_freq_, 1000.0);
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/imu_topic", 10, imu_sub_udp_callback, ros::TransportHints().tcpNoDelay());
  //  camera_detect_xyz_sub_ = nh.subscribe<custom_msgs::cloud_xyz>("/cloud_xyz_topic", 10, camera_dedtect_xyz_sub_udp_callback, ros::TransportHints().tcpNoDelay());
    map_sub_  = nh.subscribe<sensor_msgs::PointCloud2>("/lio_sam/mapping/map_global", 10,map_sub_udp_callback, ros::TransportHints().tcpNoDelay());

    ros::Subscriber delta_time_sub = nh.subscribe("/delta_time_topic", 1, delta_time_callback);

    all_other_imu_pub_ = nh.advertise<sensor_msgs::Imu>("/other_device_imu", 1);
    //all_other_camera_detect_msg_pub_ = nh.advertise<custom_msgs::cloud_xyz>("/other_camera_detect_xyz", 1);
    all_other_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/other_device_map", 60);

    std::string topic_head;
    topic_head = "";
    imu_pub_.resize(device_num_);
    other_camera_detect_msg_pub_.resize(device_num_);
    map_pub_.resize(device_num_);
    // nh.getParam("topic_head", topic_head);
    for(int i = 0; i < device_num_;++i)
    {
        std::string imu_topic, other_camera_msg_topic, map_topic;
        imu_topic = topic_head + "/imu" + std::to_string(i) + "_topic"; 
        other_camera_msg_topic = topic_head + "/camera" + std::to_string(i) + "_detect_points_topic";
        map_topic = topic_head + "/map" + std::to_string(i) + "_topic"; 
        imu_pub_[i] = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
        //other_camera_detect_msg_pub_[i] = nh.advertise<custom_msgs::cloud_xyz>(other_camera_msg_topic, 1);
        map_pub_[i] = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 60);
    }
    
    boost::thread udp_recv_thd(udp_recv_fun);
    udp_recv_thd.detach();
    ros::Duration(0.1).sleep();

    // UDP connect
    udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

    std::cout << "[rosmsg_udp_bridge] start running" << std::endl;

    ros::spin();

    close(udp_server_fd_);
    close(udp_send_fd_);
    return 0;
}