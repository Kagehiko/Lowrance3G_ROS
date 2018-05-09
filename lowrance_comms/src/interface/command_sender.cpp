#include "interface/command_sender.hpp"

#include "interface/asio.hpp"

#include "ros/ros.h"
#include "interface/network_configuration.hpp"

#include <chrono>
#include <thread>
#include <inttypes.h>

// Auxiliary function that turns on the radar
static bool startRadar(asio::ip::udp::socket& socket, asio::ip::udp::endpoint& destination){
  try{
    // Register 0x00, write command (0xC1), write a 1 (0x01) to turn on radar
    // Register 0x01, write command (0xC1), write a 1 (0x01) to turn on radar
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x00, 0xC1, 0x01})), destination);
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x01, 0xC1, 0x01})), destination);
    socket.send_to(asio::buffer(std::vector<uint8_t>({0xA0, 0xC1})), destination);
    return true;
  }
  catch(...){
    return false;
  }
}

// Auxiliary function that configures the radar
static bool configureRadar(asio::ip::udp::socket& socket, asio::ip::udp::endpoint& destination, radar_configurator config){
  try{
    // Configure range
    //uint32_t range = config.getRange() * 10; //meters to decimeters
    double range_float = 10.0F * 4.0F * ((((double)config.getRange()) * 1.414213562F) / 10.0F);
    uint32_t range = (uint32_t) range_float;
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x03, 0xC1, static_cast<uint8_t>(range&0xFF), static_cast<uint8_t>((range>>8)&0xFF), static_cast<uint8_t>((range>>16)&0xFF), 0x00})), destination);

    // Configure interference rejection
    uint8_t interference_rejection = config.getInterferenceRejection();
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x08, 0xC1, static_cast<uint8_t>(interference_rejection)})), destination);

    // Configure target boost
    uint8_t target_boost = config.getTargetBoost();
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x0A, 0xC1, static_cast<uint8_t>(target_boost)})), destination);

    // Configure local interference filter
    uint8_t local_int_filter = config.getLocalIntFilter();
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x0E, 0xC1, static_cast<uint8_t>(local_int_filter)})), destination);

    // Configure scan speed
    uint8_t scan_speed = config.getScanSpeed();
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x0F, 0xC1, static_cast<uint8_t>(scan_speed)})), destination);

    // Configure gain
    if(config.getAutoGain() == true){
      socket.send_to(asio::buffer(std::vector<uint8_t>({0x06, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xA1})), destination);
    }else{
      uint8_t gain_value = config.getManualGainValue();
      socket.send_to(asio::buffer(std::vector<uint8_t>({0x06, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, gain_value})), destination);
    }

    // Configure rain filter
    uint8_t rain_filter = config.getRainFilter();
    socket.send_to(asio::buffer(std::vector<uint8_t>({0x06, 0xC1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, rain_filter})), destination);

    //Configure Sea Clutter filter
    if(config.getAutoSeaClutter() == true){
      socket.send_to(asio::buffer(std::vector<uint8_t>({0x06, 0xC1, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD3})), destination);
    }else{
      uint8_t sea_clutter_filter_value = config.getSeaClutterFilterValue();
      socket.send_to(asio::buffer(std::vector<uint8_t>({0x06, 0xC1, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, sea_clutter_filter_value})), destination);
    }

  }
  catch(...){
    return false;
  }

  return true;
}

// A thread that sends initialization, configuration
// and "keep alive" packets to the radar
void commandSenderThread(radar_configurator initializations, std::mutex* radar_ans_mutex, bool* radar_answering){

  // Create and configure udp socket
  asio::io_service service;
  asio::ip::udp::socket* socket = new asio::ip::udp::socket(service);
  std::string multicast_ip = "236.6.7.10";
  int multicast_port = 6680;

  if (configureMulticastUdpSocket(*socket,initializations.getCardIp(),multicast_ip,multicast_port,false) == false){
    ROS_ERROR("Can't configure UDP socket for sending data. Check connection to radar and IP configuration");
    delete socket;
    if(!ros::isShuttingDown()){
      ros::shutdown();
    }
    return;
  }

  // Create destination endpoint
  asio::ip::udp::endpoint destination(asio::ip::address::from_string(multicast_ip), multicast_port);

  // Turn on the radar and send a keep alive packet
  if(startRadar(*socket, destination) == false){
    ROS_ERROR("Can't send startup packets. Check connection to radar");
    delete socket;
    if(!ros::isShuttingDown()){
      ros::shutdown();
    }
    return;
  }

  ROS_INFO("Configuring radar with:");
  ROS_INFO("IP: %s",initializations.getCardIp().c_str());
  ROS_INFO("Range: %u",initializations.getRange());
  ROS_INFO("Interference rejection: %u",initializations.getInterferenceRejection());
  ROS_INFO("Target Boost: %u",initializations.getTargetBoost());
  ROS_INFO("Local Interference Filter: %u", initializations.getLocalIntFilter());
  ROS_INFO("Scan speed: %u", initializations.getScanSpeed());
  if(initializations.getAutoGain() == true){
    ROS_INFO("Gain: auto");
  }else{
    ROS_INFO("Gain: %u", initializations.getManualGainValue());
  }
  ROS_INFO("Rain Filter: %u", initializations.getRainFilter());
  if(initializations.getAutoSeaClutter() == true){
    ROS_INFO("Sea Clutter filter: auto");
  }else{
    ROS_INFO("Sea Clutter filter: %u", initializations.getSeaClutterFilterValue());
  }

  if(configureRadar(*socket, destination, initializations) == false){
    ROS_ERROR("Couldn't send configuration packets. Check connection to radar");
    delete socket;
    if(!ros::isShuttingDown()){
      ros::shutdown();
    }
    return;
  }

  ROS_INFO("%s", "Radar initialization finished");

  while(ros::ok()){

    // Send a "keep alive" packet approximately each 10 seconds
    // Check each second if the node is still alive and if the
    // radar is active. If it is active, check if the configs
    // on the ROS Parameter Server are still the same

    for(auto i=0; i!=10; i++){
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if(ros::ok() != true) return;

      radar_ans_mutex->lock();
      bool local_radar_answering = *radar_answering;
      *radar_answering = true;
      radar_ans_mutex->unlock();

      if(local_radar_answering == false){
        bool return_val = true;
        delete socket;
        socket = new asio::ip::udp::socket(service);

        return_val = return_val && configureMulticastUdpSocket(*socket,initializations.getCardIp(),multicast_ip,multicast_port,false);
        return_val = return_val && startRadar(*socket, destination);
        return_val = return_val && configureRadar(*socket, destination, initializations);
        if(return_val == false){
          delete socket;
          ROS_ERROR("Couldn't reinitialize and reconfigure radar. Check connection to radar");
          if(!ros::isShuttingDown()){
            ros::shutdown();
          }
          return;
        }else{
          ROS_INFO("Radar reinitialized and reconfigured");
        }
      }else{

        // If the radar is answering properly, let's check for any changes in the ROS Paremeter Server
        radar_configurator new_configs;
        new_configs.fillEmptyWithRosParameterServerValues(true);

        // IP must never change during run-time
        new_configs.setCardIp(initializations.getCardIp());

        // If someone deleted values from the parameter server, reload the defaults
        new_configs.fillEmptyWithDefaults();
        if (new_configs != initializations){
          if(configureRadar(*socket,destination,new_configs) == true){
            ROS_INFO("Radar configurations updated due to changes in the ROS Parameter Server");
            initializations = new_configs;
          }else{
            ROS_ERROR("Couldn't update configuration parameters. Terminating node...");
            delete socket;
            if(!ros::isShuttingDown()){
              ros::shutdown();
            }
            return;
          }
        }
      }
    }

    ROS_INFO("%s", "Sending keep alive packet");
    // Register 0xA0, write command (0xC1).
    // No data is necessary to keep alive, unlike the reverse engineering paper said.
    try{
      socket->send_to(asio::buffer(std::vector<uint8_t>({0xA0, 0xC1})), destination);
    }
    catch(...){
      ROS_ERROR("Couldn't send keep alive packet. Check connection to radar");
      if(!ros::isShuttingDown()){
        ros::shutdown();
      }
      return;
    }
  }
}
