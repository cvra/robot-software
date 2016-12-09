#ifndef MESSAGEBUS_H
#define MESSAGEBUS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <unistd.h>
#include <stdbool.h>

#define TOPIC_NAME_MAX_LENGTH 64

typedef struct topic_s {
    void *buffer;
    size_t buffer_len;
    void *lock;
    void *condvar;
    char name[TOPIC_NAME_MAX_LENGTH + 1];
    bool published;
    struct messagebus_watcher_s *watchers;
    struct topic_s *next;
} messagebus_topic_t;

typedef struct {
    struct {
        messagebus_topic_t *head;
    } topics;
    void *lock;
    void *condvar;
} messagebus_t;

typedef struct messagebus_watchgroup_s {
    void *lock;
    void *condvar;
    messagebus_topic_t *published_topic;
} messagebus_watchgroup_t;

typedef struct messagebus_watcher_s {
    messagebus_watchgroup_t *group;
    struct messagebus_watcher_s *next;
} messagebus_watcher_t;

#define MESSAGEBUS_TOPIC_FOREACH(_bus, _topic_var_name) \
    for (int __control = -1; __control < 2; __control++) \
        if (__control < 0) { \
            messagebus_lock_acquire((_bus)->lock); \
        } else if (__control > 0) { \
            messagebus_lock_release((_bus)->lock); \
        } else  \
            for (messagebus_topic_t *(_topic_var_name) = (_bus)->topics.head; \
                 topic != NULL; \
                 (_topic_var_name) = (_topic_var_name)->next)



/** Initializes a topic object
 *
 * @parameter [in] topic The topic object to create.
 * @parameter [in] buffer,buffer_len The buffer where the topic messages will
 * @parameter [in] topic_lock The lock to use for this topic.
 * @parameter [in] topic_condvar The condition variable to use for this topic.
 * be stored.
 */
void messagebus_topic_init(messagebus_topic_t *topic, void *topic_lock, void *topic_condvar,
                           void *buffer, size_t buffer_len);

/** Initializes a new message bus with no topics.
 *
 * @parameter [in] bus The messagebus to init.
 * @parameter [in] lock The lock to use for this bus.
 * @parameter [in] condvar The condition variable used to signal threads
 * waiting on this bus.
 */
void messagebus_init(messagebus_t *bus, void *lock, void *condvar);

/** Initializes the presence of the topic on the bus.
 *
 * @parameter [in] bus The bus on which the topic must be advertised.
 * @parameter [in] topic The topic object to advertise.
 * @parameter [in] name The topic name, used to refer to it from the rest
 * of the application.
 *
 * @note The topic name will be truncated to TOPIC_NAME_MAX_LENGTH characters.
 */
void messagebus_advertise_topic(messagebus_t *bus, messagebus_topic_t *topic, const char *name);

/** Finds a topic on the bus.
 *
 * @parameter [in] bus The bus to scan.
 * @parameter [in] name The name of the topic to search.
 *
 * @return A pointer to the topic if it is found, NULL otherwise.
 */
messagebus_topic_t *messagebus_find_topic(messagebus_t *bus, const char *name);

/** Waits until a topic is found on the bus.
 *
 * @parameter [in] bus The bus to scan.
 * @parameter [in] name The name of the topic to search.
 */
messagebus_topic_t *messagebus_find_topic_blocking(messagebus_t *bus, const char *name);

/** Publish a topics on the bus.
 *
 * @parameter [in] topic A pointer to the topic to publish.
 * @parameter [in] buf Pointer to a buffer containing the data to publish.
 * @parameter [in] buf_len Length of the data buffer.
 *
 * @warning If the buffer is too big to fit in the topic, no message is sent and
 * false is returned.
 * @returns True if successful, otherwise.
 */
bool messagebus_topic_publish(messagebus_topic_t *topic, void *buf, size_t buf_len);

/** Reads the content of a single topic.
 *
 * @parameter [in] topic A pointer to the topic to read.
 * @parameter [out] buf Pointer where the read data will be stored.
 * @parameter [out] buf_len Length of the buffer.
 *
 * @returns true if the topic was published on at least once.
 * @returns false if the topic was never published to
 */
bool messagebus_topic_read(messagebus_topic_t *topic, void *buf, size_t buf_len);

/** Wait for an update to be published on the topic.
 *
 * @parameter [in] topic A pointer to the topic to read.
 * @parameter [out] buf Pointer where the read data will be stored.
 * @parameter [out] buf_len Length of the buffer.
 */
void messagebus_topic_wait(messagebus_topic_t *topic, void *buf, size_t buf_len);

/** Initializes a watch group.
 *
 * Watch group are used to wait on a set of topics in parallel (similar to
 * select(2) on UNIX). Each watchgroup has a lock and condition variable
 * associated to it.
 *
 * @parameter [in] lock The lock to use for this group.
 * @parameter [in] condvar The condition variable to use for this group.
 */
void messagebus_watchgroup_init(messagebus_watchgroup_t *group, void *lock,
                                void *condvar);

/** Adds a topic to a given group.
 *
 * @warning Removing a watchgroup is not supported for now.
 */
void messagebus_watchgroup_watch(messagebus_watcher_t *watcher,
                                 messagebus_watchgroup_t *group,
                                 messagebus_topic_t *topic);

messagebus_topic_t *messagebus_watchgroup_wait(messagebus_watchgroup_t *group);

/** @defgroup portable Portable functions, platform specific.
 * @{*/

/** Acquire a reentrant lock (mutex). */
extern void messagebus_lock_acquire(void *lock);

/** Release a lock previously acquired by messagebus_lock_acquire. */
extern void messagebus_lock_release(void *lock);

/** Signal all tasks waiting on the given condition variable. */
extern void messagebus_condvar_broadcast(void *var);

/** Wait on the given condition variable. */
extern void messagebus_condvar_wait(void *var);

/** @} */

#ifdef __cplusplus
}
#endif
#endif
