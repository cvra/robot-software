#pragma once

#include <iostream>
#include <functional>
#include "absl/container/flat_hash_map.h"

// A shell command handler takes 3 arguments:
// - An output stream, which allows the command to display information back to the user.
// - argc and argv, the argument vector in UNIX fashion:
using TerminalHandler = std::function<void(std::ostream&, int, const char**)>;

// Listens on the provided TCP port and interprets commands coming that way.
// The first argument is a map from command name to handlers
// The second one is the TCP port number to use.
// This function supports several simulatenous clients and spawns one thread per connection.
void shell_tcp_serve(const absl::flat_hash_map<std::string, TerminalHandler> commands, int port_number);
