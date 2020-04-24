#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>

#define LASERSCAN_TOPIC "scan"
#define POINTCLOUD_TOPIC "cloud_points"

#define SAVE_DATA
#ifdef SAVE_DATA
const int MAX_NUM_SAMPLES = 100;
int num_samples_saved = 0;
std::vector<pcl::PointXYZ> save_points[MAX_NUM_SAMPLES];

#endif

ros::Publisher pointcloud_publisher;

void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg)
{
    #ifndef SAVE_DATA
    ROS_INFO("Sensor_msgs received: %s", laserscan_msg->header.frame_id.c_str());
    #endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ> ());
	point_cloud_pcl->header.frame_id = laserscan_msg->header.frame_id;

    
    float angle_range = (laserscan_msg->angle_max - laserscan_msg->angle_min);
	int num_angles = (angle_range)/laserscan_msg->angle_increment;
    point_cloud_pcl->width = num_angles + 1;
    point_cloud_pcl->height = 1;
    point_cloud_pcl->points.reserve(point_cloud_pcl->width * point_cloud_pcl->height);
    float angle = laserscan_msg->angle_min;

    int window_halfsize_angles = 2;//[degrees]

    for(int i = 0; i <= num_angles; i++)
    {
        #ifndef SAVE_DATA
        if(i >= num_angles/2 - window_halfsize_angles &&  i <= num_angles/2 + window_halfsize_angles)
            ROS_INFO("range[%d]:%.2f", i, laserscan_msg->ranges[i]);
        #endif
        //if(i >= num_angles/2 - window_halfsize_angles &&  i <= num_angles/2 + window_halfsize_angles)
        point_cloud_pcl->points.push_back(pcl::PointXYZ(cos(angle) * laserscan_msg->ranges[i], sin(angle) * laserscan_msg->ranges[i], 0));
        //else point_cloud_pcl->points.push_back(pcl::PointXYZ(0,0,0));
        angle += laserscan_msg->angle_increment;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*point_cloud_pcl, cloud_msg);
    pointcloud_publisher.publish(cloud_msg);

    #ifdef SAVE_DATA
    for(int i = 0; i < num_angles; i++)
    {
        if(i >= num_angles/2 - window_halfsize_angles &&  i <= num_angles/2 + window_halfsize_angles)
            save_points[num_samples_saved].push_back(point_cloud_pcl->points[i]);
    }

    num_samples_saved++;
    #endif
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lrf_ros_node");

	ros::NodeHandle node_handle;
    #ifdef SAVE_DATA
    for(int i = 0; i < argc; i++)
    {
        ROS_INFO("input args[%d]: %s", i, argv[i]);
    }


    if(argc < 2)
    {
        ROS_INFO("Wrong args format! Use:");
        ROS_INFO("rosrun lrf_ros lrf_ros_node [save_file_name]");
        return -1;
    }
    
    ROS_INFO("Collecting sensor data");
    #else
    ROS_INFO("Lidar processing node started.");

    #endif

    ros::Subscriber laserscan_subscriber = node_handle.subscribe(LASERSCAN_TOPIC, 1, laserscan_callback);
    pointcloud_publisher = node_handle.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, 1);
    #ifdef SAVE_DATA
    while(ros::ok())
    {
        if(num_samples_saved >= MAX_NUM_SAMPLES)
        {
            break;
        }
        ros::spinOnce();
    }

    std::ofstream save_data_file;
    std::stringstream save_file_name;
    save_file_name << argv[1] << ".txt";
    
    
    save_data_file.open(save_file_name.str().c_str());
    if(!save_data_file.is_open())
    {
        ROS_INFO("Couldn't open file \"%s.txt\"", argv[1]);
        return -2;
    }
    
    ROS_INFO("Saving sensor data to file \"%s.txt\"", argv[1]);
    
    for(int i = 0; i < MAX_NUM_SAMPLES; i++)
    {
        for(int k = 0; k < save_points[i].size(); k++)
            save_data_file << (save_points[i])[k] << ", ";
        save_data_file << "\n";
    }
            

    save_data_file.close();
    #else
    ros::spin();
    #endif


    return 0;
}