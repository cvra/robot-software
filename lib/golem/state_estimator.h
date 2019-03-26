#ifndef GOLEM_STATE_ESTIMATOR_H
#define GOLEM_STATE_ESTIMATOR_H

#include <memory>
#include <type_traits>
#include <utility>

namespace golem {

// Models the state estimator of your system
// Accepts any class that implements the following functions for given State and Feedback types:
// - void update(const Feedback&) const;
// - State get() const;
template <typename State, typename Feedback>
class StateEstimator {
public:
    StateEstimator() = default;
    StateEstimator(const StateEstimator&) = delete;
    StateEstimator& operator=(const StateEstimator&) = delete;
    StateEstimator(StateEstimator&&) = default;
    StateEstimator& operator=(StateEstimator&&) = default;

    template <typename T>
    StateEstimator(T&& impl)
        : m_impl(new model_t<typename std::decay<T>::type>(std::forward<T>(impl)))
    {
    }

    template <typename T>
    StateEstimator& operator=(T&& impl)
    {
        m_impl.reset(new model_t<typename std::decay<T>::type>(std::forward<T>(impl)));
        return *this;
    }

    State get() const
    {
        return m_impl->do_get();
    }
    void update(const Feedback& in)
    {
        return m_impl->do_update(in);
    }

private:
    struct concept_t {
        virtual ~concept_t() {}
        virtual State do_get() const = 0;
        virtual void do_update(const Feedback&) = 0;
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

        State do_get() const override
        {
            return m_data.get();
        }
        void do_update(const Feedback& in) override
        {
            return m_data.update(in);
        }

        T m_data;
    };

    std::unique_ptr<concept_t> m_impl;
};
} // namespace golem

#endif /* GOLEM_STATE_ESTIMATOR_H */
