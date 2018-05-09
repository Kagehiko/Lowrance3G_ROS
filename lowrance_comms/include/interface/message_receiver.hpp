#ifndef MESSAGE_RECEIVER_H
#define MESSAGE_RECEIVER_H

#include <string>
#include <mutex>

//Function prototyes
void messageReceiverThread(std::string network_card_ip, std::mutex* radar_ans_mutex, bool* radar_answering);

#endif
