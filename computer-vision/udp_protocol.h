#pragma once

#include "computer_vision.pb.h"

#include <string>

/* caution here: dst is an IP address, not a DNS name */
void udp_send_result(std::string dst, int port, ComputerVisionResult result);
