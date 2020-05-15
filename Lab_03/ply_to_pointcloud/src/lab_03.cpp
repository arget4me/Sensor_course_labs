#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "happly/happly.h"
#include <string>
#include <vector>
#include <iostream>
#include <array>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ply_to_pointcloud_node");
	ros::NodeHandle nh;

	const char* topics[] = {"cloud_150",
				"cloud_100",
				"cloud_50"};
	ros::Publisher cloud_publish[3];
	for(int i = 0; i < 3; i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>& cloud = *cloud_ptr;
		cloud.header.frame_id = "points";

		cloud.height = 1;
		sensor_msgs::PointCloud2 point_cloud;

		happly::PLYData plyIn(argv[1 + i]);
		std::vector<std::string> s = plyIn.getElementNames();
	
		std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
		cloud.width = vPos.size();	
		cloud.points.resize(cloud.height * cloud.width);
		cloud.points.clear();

		for(int i = 0; i < vPos.size(); i++)
		{
			pcl::PointXYZ p;
			p.x = vPos[i][0];
			p.y = vPos[i][2];
			p.z = vPos[i][1];

			//std::cout << p << std::endl;
			cloud.points.push_back(p);
		}
		//RANSAC
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    			pcl::SACSegmentation<pcl::PointXYZ> seg;
	
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.01);

			seg.setInputCloud(cloud_ptr);
		        seg.segment(*inliers, *coefficients);


			ROS_INFO("Plane: %fx, %fy, %fz, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
		}
	
		pcl::toROSMsg(cloud, point_cloud);
		//ROS_INFO("CONVERTED");
		cloud_publish[i] = nh.advertise<sensor_msgs::PointCloud2>(topics[i], 1, true);
	
		cloud_publish[i].publish(point_cloud);
	}
	ros::spin();


	return 0;
}
