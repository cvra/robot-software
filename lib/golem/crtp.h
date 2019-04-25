#ifndef GOLEM_IMPL_CRTP_HELPER_H
#define GOLEM_IMPL_CRTP_HELPER_H

// Curiously recurring template pattern (CRTP) helper class
// From https://www.fluentcpp.com/2017/05/19/crtp-helper/
template <typename T, template <typename...> class crtpType>
class Crtp {
public:
    T& underlying() { return static_cast<T&>(*this); }
    const T& underlying() const { return static_cast<const T&>(*this); }
};

#endif /* GOLEM_IMPL_CRTP_HELPER_H */
