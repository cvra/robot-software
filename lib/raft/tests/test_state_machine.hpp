#pragma once

#include "../raft.hpp"

class TestStateMachine
{
    public:
    enum class Operation {
        FOO = 0xca,
        BAR = 0xfe,
    };

    void apply(Operation op)
    {
        (void) op;
    }
};

using TestRaftState = raft::State<TestStateMachine>;
using TestPeer = raft::Peer<TestStateMachine>;
using TestMessage = raft::Message<TestStateMachine>;
