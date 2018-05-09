#ifndef DATA_PUBLISHING_H
#define DATA_PUBLISHING_H

#include "ros/ros.h"

// Initializes the publisher. Required for publishing ROS messages
void publisherInit(ros::NodeHandle node);

// Publishes data to the topic "lowrance_data". The data needs to be properly formated first
void publishRadarData(std::vector<uint8_t> data_to_publish);

#endif
