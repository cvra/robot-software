#include "shell.h"

#include <sstream>
#include <thread>
#include <absl/strings/match.h>
#include <absl/strings/str_format.h>
#include <absl/strings/str_split.h>
#include <error/error.h>
#include <sys/socket.h>
#include <netinet/in.h>

class SocketAutocloser {
    int socket;

public:
    SocketAutocloser(int s)
        : socket(s)
    {
    }
    ~SocketAutocloser() { close(socket); }
};

class SocketBuf : public std::stringbuf {
    int socket_;

public:
    SocketBuf(int socket)
        : socket_(socket)
    {
    }

    int sync() override
    {
        auto out = str();
        const int flags = 0;
        send(socket_, out.data(), out.length(), flags);
        str("");

        return 0;
    }
};

static void handle(const absl::flat_hash_map<std::string, TerminalHandler> commands, int socket);

void shell_tcp_serve(const absl::flat_hash_map<std::string, TerminalHandler> commands, int port_number)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_number);

    if (bind(server_fd, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) < 0) {
        ERROR("Failed to bind");
    }

    const int max_queue = 1;
    if (listen(server_fd, max_queue) < 0) {
        ERROR("Failed to listen");
    }

    int new_socket;

    while (true) {
        socklen_t addrlen = sizeof(address);
        new_socket = accept(server_fd,
                            reinterpret_cast<struct sockaddr*>(&address),
                            &addrlen);

        if (new_socket < 0) {
            WARNING("Could not accept new socket");
            continue;
        }

        NOTICE("Accepted new connection");

        // We spawn one thread to handle that connection, which is responsible
        // for the socket from now on, including closing it.
        std::thread server_thread(handle, commands, new_socket);
        server_thread.detach();
    }
}

static void handle(const absl::flat_hash_map<std::string, TerminalHandler> commands, int socket)
{
    SocketAutocloser _(socket);

    // TODO: Allow the stream to be used as input, for example for confirmation dialogs ([y/N])
    SocketBuf buf(socket);
    std::ostream stream(&buf);

    const std::string prompt{"cvra >"};

    while (true) {
        stream << prompt;
        std::flush(stream);

        // First, read a complete line from the socket
        std::string command;

        while (!absl::EndsWith(command, "\n")) {
            char c;
            if (read(socket, &c, 1) < 1) {
                return;
            }
            command.push_back(c);
        }

        // Remove the empty newline at the end
        command.pop_back();

        if (command.empty()) {
            continue;
        }

        // Split arguments
        std::vector<std::string> args = absl::StrSplit(command, absl::ByAnyChar(" \t"), absl::SkipEmpty());

        std::vector<const char*> argv;
        std::transform(
            args.begin(), args.end(), std::back_inserter(argv),
            [](std::string& v) { return v.c_str(); });

        if (commands.contains(args[0])) {
            commands.at(args[0])(stream, argv.size(), argv.data());
        } else {
            stream << absl::StrFormat("Unknown command \"%s\"", args[0]) << std::endl;
        }
    }
}
