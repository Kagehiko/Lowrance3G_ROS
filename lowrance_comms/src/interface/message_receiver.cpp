
#include "interface/message_receiver.hpp"

#include "interface/network_configuration.hpp"
#include "interface/data_publishing.hpp"

#include "interface/asio.hpp"

#include <chrono>
#include <thread>

static void preparePacketData(double timestamp_fp, std::vector<uint8_t> packet_data);

// A thread that receives image data from the radar
void messageReceiverThread(std::string network_card_ip, std::mutex* radar_ans_mutex, bool* radar_answering){

  // Create socket.
  asio::io_service service;
  //asio::ip::udp::socket socket(service);
  asio::ip::udp::socket* socket = new asio::ip::udp::socket(service);
  if(!configureMulticastUdpSocket(*socket,network_card_ip,"236.6.7.8",6678,true)){
    ROS_ERROR("Can't configure UDP socket for receiving data. Check connection to radar and IP configuration");
    delete socket;
    if(!ros::isShuttingDown()){
      ros::shutdown();
    }
    return;
  }

  asio::ip::udp::endpoint sender;
  std::vector<uint8_t> buffer;
  std::size_t bytes_readable = 0;
  auto no_answer_count = 0;
  while(ros::ok()){

    // Poll until data is available.
    while (!bytes_readable){

      if(ros::ok() == false){
        delete socket;
        return;
      }

      // After 5 seconds without answer, ask the command_sender thread to re-initialize the radar
      if(no_answer_count == 5){
        ROS_WARN("Radar not sending packets");
        radar_ans_mutex->lock();
        *radar_answering = false;
        radar_ans_mutex->unlock();
        no_answer_count = 0;
        if(ros::ok() != true) return;
        delete socket;
        asio::ip::udp::socket* socket = new asio::ip::udp::socket(service);
        if(!configureMulticastUdpSocket(*socket,network_card_ip,"236.6.7.8",6678,true)){
          continue;
        }
      }
      // Issue command to socket to get number of bytes readable.
      asio::socket_base::bytes_readable num_of_bytes_readable(true);
      socket->io_control(num_of_bytes_readable);

      // Get the value from the command.
      bytes_readable = num_of_bytes_readable.get();

      // If there is no data available, then sleep 1 second.
      if (!bytes_readable){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        no_answer_count++;
        if(ros::ok() == false){
          delete socket;
          return;
        }
      }
    }

    // Resize the buffer to store all available data.
    buffer.resize(bytes_readable);

    socket->receive_from(asio::buffer(buffer, bytes_readable), sender);


    //socket.read_some(asio::buffer(buffer, bytes_readable));
    //socket.async_receive(asio::buffer(buffer, bytes_readable), socket);

    //Get a timestamp on the received packet
    double timestamp =ros::Time::now().toSec();

    //ROS_INFO("Received %lu bytes", ((unsigned long int)buffer.size()));

    //Check if the packet is indeed a image packet
    if(buffer.size() == 17160){
      // Messages received, so reset the "no answer" counter
      radar_ans_mutex->lock();
      *radar_answering = true;
      radar_ans_mutex->unlock();
      no_answer_count = 0;
      //Publish the received data in a ROS topic
      preparePacketData(timestamp, buffer);
    }
  }
}

// Prepares the received packet data to be published in a ROS topic
void preparePacketData(double timestamp, std::vector<uint8_t> packet_data){

  // Data that will be published in the topic
  std::vector<uint8_t> data_to_publish = {};

  // Break timestamp into 10 bytes (8 for integer part and 2 for decimal part)
  uint64_t timestamp_int_part;
  uint16_t timestamp_frac_part;

  // Get integer and fractional parts into different doubles
  double temp_int_part, temp_frac_part;
  temp_frac_part = modf(timestamp, &temp_int_part);

  // Typecast dobules to the correct types
  timestamp_int_part = static_cast<uint64_t> (temp_int_part);
  timestamp_frac_part = static_cast<uint16_t>(temp_frac_part*1000);

  // Store timestamp integer part (big endian format)
  for(auto i=0; i!=8; i++){
    data_to_publish.push_back( (uint8_t) ((timestamp_int_part & static_cast<uint64_t>(0xFF * pow(2,(8*(7-i)))) )>>(8*(7-i))));
  }

  // Store timestamp fractional part (big endian format)
  for(auto i=0; i!=2; i++){
    data_to_publish.push_back( (uint8_t) ((timestamp_frac_part & static_cast<uint64_t>(0xFF * pow(2,(8*(1-i)))) )>>(8*(1-i))));
  }

  // Constants for accessing packet data
  auto const frame_header_size = 8;

  auto const scanline_header_angle_LSB = 8;
  auto const scanline_header_angle_MSB = scanline_header_angle_LSB + 1;

  auto const scanline_header_scale_LSB = 12;
  auto const scanline_header_scale_MSB = scanline_header_scale_LSB + 1;

  auto const scanline_header_size = 24;
  auto const scanline_data_size = 512;
  auto const scanline_total_size = scanline_header_size + scanline_data_size;

  // Gather the data from the 32 scanlines that were received in the packet
  for(auto scanline_index=0; scanline_index !=32; scanline_index++){

    // Clear vector data while keeping the timestamp intact (first 10 bytes)
    data_to_publish = std::vector<uint8_t>(data_to_publish.begin(), data_to_publish.begin()+10);

    // Store scanline angle data (big endian)
    data_to_publish.push_back(packet_data.at( frame_header_size+scanline_header_angle_MSB+(scanline_total_size*scanline_index) ));
    data_to_publish.push_back(packet_data.at( frame_header_size+scanline_header_angle_LSB+(scanline_total_size*scanline_index) ));

    uint16_t scan_radius;
    scan_radius = (packet_data.at(frame_header_size+scanline_header_scale_MSB+(scanline_total_size*scanline_index))<<8);
    scan_radius |= packet_data.at(frame_header_size+scanline_header_scale_LSB+(scanline_total_size*scanline_index));

    // Scan radius comes divided by 4 (for some reason?)
    //scan_radius = scan_radius * 4;
    scan_radius = (uint16_t) (((double)scan_radius)*(10.0F/1.414213562F));
    data_to_publish.push_back( ((scan_radius>>8)&0xFF) );
    data_to_publish.push_back( (scan_radius&0xFF) );

    // If we find zeroes in the data, we will ommit them
    bool previous_data_was_not_zero = false;

    // Go through each byte of the scanline
    for(uint16_t scanline_data_index=0; scanline_data_index!= 512; scanline_data_index++){

      auto current_index = frame_header_size + scanline_header_size + (scanline_total_size*scanline_index) + scanline_data_index;

      if(previous_data_was_not_zero == true){

        if(packet_data.at(current_index) != 0x00){
          // If we are saving a bunch of data and we haven't found a 0x00, save the data
          data_to_publish.push_back(packet_data.at(current_index));
        } else {
          // We were saving data but now found a zero, so ignore this data
          previous_data_was_not_zero = false;
        }

      } else {
        // If we were not saving data but now we have found data, then save a 0x00
        // to indicate we found new data, save the byte index where we found it, and save the byte itself
        if(packet_data.at(current_index) != 0x00){
          data_to_publish.push_back(0x00);
          data_to_publish.push_back((scanline_data_index>>8)&0xFF);
          data_to_publish.push_back(scanline_data_index&0xFF);
          data_to_publish.push_back(packet_data.at(current_index));
          previous_data_was_not_zero = true;
        } else {
          // We weren't saving data and the new data is still zero, so ignore it
          continue;
        }
      }
    }

    //We now have gathered the data from the scanline, so we must publish it
    publishRadarData(data_to_publish);
  }
}
