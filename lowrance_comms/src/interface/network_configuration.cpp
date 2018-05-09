#include "interface/network_configuration.hpp"


// Opens and configures a UDP multicast socket for sending or receiving data
bool configureMulticastUdpSocket(
    asio::ip::udp::socket& socket,
    std::string network_card_ip,
    std::string multicast_ip,
    uint16_t multicast_port,
    bool is_receiver){

  try{
    asio::ip::address mc_address = asio::ip::address::from_string(multicast_ip);
    asio::ip::address_v4 local_address = asio::ip::address_v4::from_string(network_card_ip);

    socket.open(asio::ip::udp::v4());

    //asio::ip::udp::socket::non_blocking_io non_blocking_io(true);
    //socket.io_control(non_blocking_io);

    // Allow other processes to reuse the address, permitting other processes on
    // the same machine to use the multicast address.
    socket.set_option(asio::ip::udp::socket::reuse_address(true));

    // Guarantee the loopback is enabled so that multiple processes on the same
    // machine can receive data that originates from the same socket.
    socket.set_option(asio::ip::multicast::enable_loopback(true));

    if(is_receiver == false){
      // Set network interface address from which data will be sent
      socket.set_option(asio::ip::multicast::outbound_interface(local_address));
      // Bind socket to any port (0 means any port) since we want to send data
      socket.bind(asio::ip::udp::endpoint(asio::ip::address_v4::any(),0));
      // Join group
      socket.set_option(asio::ip::multicast::join_group(mc_address));
    } else {
      // Bind socket to the multicast port
      socket.bind(asio::ip::udp::endpoint(asio::ip::address_v4::any(),multicast_port));
      // Join group, but specify the network card's IP
      socket.set_option(asio::ip::multicast::join_group(mc_address.to_v4(),local_address));
    }
    return true;
  }
  catch(...){
    return false;
  }
}
