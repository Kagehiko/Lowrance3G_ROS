#include <iostream>
#include <thread>
#include <string>
#include <sstream>
#include <mutex>
#include <math.h>

#include "ros/ros.h"

#include "interface/command_sender.hpp"
#include "interface/message_receiver.hpp"
#include "interface/radar_configurator.hpp"
#include "interface/data_publishing.hpp"



int main(int argc, char* argv[]){

    
  // ROS node and publisher initialization
  ros::init(argc, argv, "lowrance_comms_interface");
  ros::NodeHandle lowrance_node;
  
  publisherInit(lowrance_node);
  
  // Parse console commands
  radar_configurator initializations;

  if(initializations.parse_console_args(argc, argv) == false){
    //Parser couldn't understand arguments, so shut down the node right now
    return 0;
  }

  initializations.fillEmptyWithRosParameterServerValues();

  if(initializations.fillEmptyWithDefaults() == false){
    // This method returns false if an IP is still not loaded,
    // at which point the threads can't be initialized and the
    // node shuts down.
    return 0;
  }

  std::mutex radar_ans_mutex;
  bool radar_answering = true;
  {
    std::thread thread_1(commandSenderThread,   initializations, &radar_ans_mutex, &radar_answering);
    std::thread thread_2(messageReceiverThread, initializations.getCardIp(), &radar_ans_mutex, &radar_answering);

    // This thread may block due to the socket reading
    thread_2.detach();

    // Wait for thread 1 to end
    thread_1.join();
  }

  // Thread 2 will go out of scope and be destroyed

  return 0;
}


/*
std::string network_card_ip;

if(argc==1){
  ROS_WARN("%s", "No IP specified via command line. Please input the network card's local IP:");
  std::cin >> network_card_ip;
} else {
  network_card_ip = std::string(argv[1]);
  ROS_INFO("Using network card with IP %s", network_card_ip.c_str());
}

radar_init_list initializations;
*/
