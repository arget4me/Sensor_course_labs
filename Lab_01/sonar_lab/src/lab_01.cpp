#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>

std::vector<float> sonar_range_data(100);

#define ARDUINO_RANGE_TOPIC "/ultrasound/range"



void arduino_range_callback(const sensor_msgs::Range::ConstPtr& range_msg)
{
    int i = sonar_range_data.size();
    ROS_INFO("[%d]: Sonar range: %.2f.", i, range_msg->range);
    sonar_range_data.push_back(range_msg->range);
}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "sonar_lab");

    ros::NodeHandle node_handle;
    for(int i = 0; i < argc; i++)
    {
        ROS_INFO("input args[%d]: %s", i, argv[i]);
    }


    if(argc < 2)
    {
        ROS_INFO("Wrong args format! Use:");
        ROS_INFO("rosrun sonar_lab sonar_lab_node [save_file_name]");
        return -1;
    }

    sonar_range_data.clear();

    ROS_INFO("Waiting for data...");
    ros::Subscriber arduino_range_subscriber = node_handle.subscribe(ARDUINO_RANGE_TOPIC, 1, arduino_range_callback);

    while(ros::ok())
    {
        if(sonar_range_data.size() >= 100)
        {
            break;
        }
        ros::spinOnce();
    }

    std::stringstream save_file_name;
    save_file_name << argv[1] << ".txt";
    
    std::ofstream save_data_file;
    save_data_file.open(save_file_name.str().c_str());
    for(int i = 0; i < 100; i++)
    {
        save_data_file << sonar_range_data[i] << "\n";
    }
    save_data_file.close();


    return 0;
}



