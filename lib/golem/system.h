#ifndef GOLEM_SYSTEM_H
#define GOLEM_SYSTEM_H

#include <memory>
#include <type_traits>
#include <utility>

namespace golem {

// Models the hardware interface of your robot system
// Accepts any class that implements the following functions for given Feedback and Input types:
// - Feedback measure() const;
// - void apply(const Input& input);
template <typename Feedback, typename Input>
class System {
public:
    System() = default;
    System(const System&) = delete;
    System& operator=(const System&) = delete;
    System(System&&) = default;
    System& operator=(System&&) = default;

    template <typename T>
    System(T&& impl)
        : m_impl(new model_t<typename std::decay<T>::type>(std::forward<T>(impl)))
    {
    }

    template <typename T>
    System& operator=(T&& impl)
    {
        m_impl.reset(new model_t<typename std::decay<T>::type>(std::forward<T>(impl)));
        return *this;
    }

    Feedback measure() const
    {
        return m_impl->do_measure();
    }
    void apply(const Input& in)
    {
        return m_impl->do_apply(in);
    }

private:
    struct concept_t {
        virtual ~concept_t() {}
        virtual Feedback do_measure() const = 0;
        virtual void do_apply(const Input&) = 0;
    };
    template <typename T>
    struct model_t : public concept_t {
        model_t() = default;
        model_t(const T& v)
            : m_data(v)
        {
        }
        model_t(T&& v)
            : m_data(std::move(v))
        {
        }

        Feedback do_measure() const override
        {
            return m_data.measure();
        }
        void do_apply(const Input& in) override
        {
            return m_data.apply(in);
        }

        T m_data;
    };

    std::unique_ptr<concept_t> m_impl;
};
} // namespace golem

#endif /* GOLEM_SYSTEM_H */
