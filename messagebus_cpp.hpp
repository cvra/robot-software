#ifndef MESSAGEBUS_CPP_HPP
#define MESSAGEBUS_CPP_HPP

namespace messagebus
{
template <typename T>
class TopicWrapper
{
public:
    TopicWrapper(messagebus_topic_t *t);

    /// Wrapper around messagebus_topic_publish
    void publish(const T &msg);

    /// Wrapper around messagebus_topic_read
    bool read(T &msg);

    /// Wrapper around messagebus_topic_wait
    T wait();

    /// Returns true if this wraps a valid topic (i.e. not nullptr)
    operator bool();

private:
    messagebus_topic_t *topic;
};

template <typename T>
TopicWrapper<T> find_topic(messagebus_t &bus, const char *topic_name)
{
    auto topic = messagebus_find_topic(&bus, topic_name);
    return TopicWrapper<T>(topic);
}

template <typename T>
TopicWrapper<T> find_topic_blocking(messagebus_t &bus, const char *topic_name)
{
    auto topic = messagebus_find_topic_blocking(&bus, topic_name);
    return TopicWrapper<T>(topic);
}

template <typename T>
TopicWrapper<T>::TopicWrapper(messagebus_topic_t *t)
    : topic(t)
{
}

template <typename T>
void TopicWrapper<T>::publish(const T &msg)
{
    messagebus_topic_publish(topic, &msg, sizeof(T));
}

template <typename T>
bool TopicWrapper<T>::read(T &msg)
{
    return messagebus_topic_read(topic, &msg, sizeof(T));
}

template <typename T>
T TopicWrapper<T>::wait()
{
    T res;
    messagebus_topic_wait(topic, &res, sizeof(T));
    return res;
}

template <typename T>
TopicWrapper<T>::operator bool()
{
    return topic != nullptr;
}

} // namespace messagebus

#endif
