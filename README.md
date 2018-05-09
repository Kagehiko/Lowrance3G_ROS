# Lowrance3G_ROS
A ROS node for communicating with a Lowrance 3G radar over ethernet with all commands implemented.

[Lowrance's 3G broadband radar](https://www.lowrance.com/lowrance/type/radar/lowrance-3g-bb-radar-kit-usa-/) is a excelent tool for Autonomous Surface Vehicles, however they unfortunately do not disclose how to communicate with the radar.

This repo hosts a [ROS](http://www.ros.org/) node that you can use to communicate with such a radar. Note that this node was developed for ROS Kinectic. It uses the standalone version of the ASIO library for the network stuff.

To use, simply copy the lowrance_comms folder into your catkin workspace "src" folder and call `catkin_make`. Then, start the node by calling `rosrun lowrance_comms lowrance_comms_interface -i YOUR_NETWORK_CARD_IP_HERE`. Make sure to provide **your** network card's IP with the argument -i and **not** the radar's IP. The reason for this is because the radar communicates over multicast UDP, meaning you'll be sending packets to the broadcast address and not directly to the radar.

The node is capable of launching without any arguments but you would need to at least have the network card IP stored in the ROS Parameter Server beforehand.

Radar data is published on a topic called `lowrance_data`. A file named `PACKAGE README` inside the package folder describes the format of the data and a list of optional arguments that you can provide when launching the node. Alternatively, you can also change these parameters during runtime by changing them in the ROS Parameter Server. When the node detects a change, it will configure the radar accordingly.
