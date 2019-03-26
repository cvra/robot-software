#ifndef GOLEM_CONTROLLER_H
#define GOLEM_CONTROLLER_H

#include <memory>
#include <type_traits>
#include <utility>

namespace golem {

// Models a controller
// Accepts any class that implements the following functions for given State, Consign, and Input types:
// - void set(const Consign&);
// - Input update(const State&) const;
// - Consign error() const;
template <typename State, typename Consign, typename Input>
class Controller {
public:
    Controller() = default;
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;
    Controller(Controller&&) = default;
    Controller& operator=(Controller&&) = default;

    template <typename T>
    Controller(T&& impl)
        : m_impl(new model_t<typename std::decay<T>::type>(std::forward<T>(impl)))
    {
    }

    template <typename T>
    Controller& operator=(T&& impl)
    {
        m_impl.reset(new model_t<typename std::decay<T>::type>(std::forward<T>(impl)));
        return *this;
    }

    void set(const Consign& target)
    {
        return m_impl->do_set(target);
    }
    Input update(const State& state)
    {
        return m_impl->do_update(state);
    }
    Consign error() const
    {
        return m_impl->do_error();
    }

private:
    struct concept_t {
        virtual ~concept_t() {}
        virtual void do_set(const Consign&) = 0;
        virtual Input do_update(const State&) = 0;
        virtual Consign do_error() const = 0;
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

        void do_set(const Consign& target) override
        {
            return m_data.set(target);
        }
        Input do_update(const State& state) override
        {
            return m_data.update(state);
        }
        Consign do_error() const override
        {
            return m_data.error();
        }

        T m_data;
    };

    std::unique_ptr<concept_t> m_impl;
};
} // namespace golem

#endif /* GOLEM_CONTROLLER_H */
