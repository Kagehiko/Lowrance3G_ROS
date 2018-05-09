#include "interface/data_publishing.hpp"

#include "std_msgs/UInt8MultiArray.h"

static ros::Publisher publisher;

// Initializes the publisher. Required for publishing ROS messages
void publisherInit(ros::NodeHandle node){
  publisher = node.advertise<std_msgs::UInt8MultiArray>("lowrance_data", 1000);
}

// Publishes data to the topic "lowrance_data". The data needs to be properly formated first
void publishRadarData(std::vector<uint8_t> data_to_publish){
  std_msgs::UInt8MultiArray array;

  array.data.clear();

  //Push the data from our std::vector to the array
  for (auto i = 0; i != data_to_publish.size(); i++){
    array.data.push_back(data_to_publish.at(i));
  }
  //Publish array
  publisher.publish(array);
}
