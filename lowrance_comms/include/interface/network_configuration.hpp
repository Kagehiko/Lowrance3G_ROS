#ifndef NETWORK_CONFIGURATION_H
#define NETWORK_CONFIGURATION_H

#include "interface/asio.hpp"

// Opens and configures a UDP multicast socket for sending or receiving data
bool configureMulticastUdpSocket(asio::ip::udp::socket& socket,
                                 std::string network_card_ip,
                                 std::string multicast_ip,
                                 uint16_t multicast_port,
                                 bool is_receiver);

#endif
