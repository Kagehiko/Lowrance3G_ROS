# Lowrance3G_ROS
A ROS node for communicating with a Lowrance 3G radar over ethernet with all commands implemented.

[Lowrance's 3G broadband radar](https://www.lowrance.com/lowrance/type/radar/lowrance-3g-bb-radar-kit-usa-/) is a excelent tool for Autonomous Surface Vehicles, however they unfortunately do not disclose how to communicate with the radar.

This repo hosts a [ROS](http://www.ros.org/) node that you can use to communicate with such a radar. Note that this node was developed for ROS Kinectic. It uses the standalone version of the ASIO library for the network stuff.
