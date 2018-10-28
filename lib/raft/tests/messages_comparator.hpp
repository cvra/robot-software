#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../raft.hpp"

class RaftMessageComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(const void* object1, const void* object2);
    virtual SimpleString valueToString(const void* object);
};

// Helper functions to allow use of CHECK_EQUAL with enum classes
SimpleString StringFrom(raft::Message::Type type);
SimpleString StringFrom(raft::NodeState state);

