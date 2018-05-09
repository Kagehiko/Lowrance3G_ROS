#ifndef COMMAND_SENDER_H
#define COMMAND_SENDER_H

#include <string>
#include <mutex>

#include "interface/radar_configurator.hpp"

// A thread that sends commands to the lowrance radar
void commandSenderThread(radar_configurator initializations, std::mutex* radar_ans_mutex, bool* radar_answering);

#endif
