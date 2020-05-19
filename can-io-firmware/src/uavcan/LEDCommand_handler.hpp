#ifndef LEDCOMMAND_HANDLER_HPP
#define LEDCOMMAND_HANDLER_HPP

#include <uavcan/uavcan.hpp>
#include <cvra/io/LEDCommand.hpp>

void LEDCommand_handler(
    const uavcan::ReceivedDataStructure<cvra::io::LEDCommand::Request>& req,
    cvra::io::LEDCommand::Response &rsp);

#endif
