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
    char name[TOPIC_NAME_MAX_LENGTH+1];
    struct topic_s *next;
    bool published;
} topic_t;

typedef struct {
    struct {
        topic_t *head;
    } topics;
} messagebus_t;


/** Initializes a topic object
 *
 * @parameter [in] topic The topic object to create.
 * @parameter [in] buffer,buffer_len The buffer where the topic messages will
 * be stored.
 */
void topic_init(topic_t *topic, void *buffer, size_t buffer_len);

/** Initializes a new message bus with no topics.
 *
 * @parameter [in] bus The messagebus to init.
 */
void messagebus_init(messagebus_t *bus);

/** Initializes the presence of the topic on the bus.
 *
 * @parameter [in] bus The bus on which the topic must be advertised.
 * @parameter [in] topic The topic object to advertise.
 * @parameter [in] name The topic name, used to refer to it from the rest
 * of the application.
 *
 * @note The topic name will be truncated to TOPIC_NAME_MAX_LENGTH characters.
 */
void messagebus_advertise_topic(messagebus_t *bus, topic_t *topic, const char *name);

/** Finds a topic on the bus.
 *
 * @parameter [in] bus The bus to scan.
 * @parameter [in] name The name of the topic to search.
 *
 * @return A pointer to the topic if it is found, NULL otherwise.
 */
topic_t *messagebus_find_topic(messagebus_t *bus, const char *name);

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
bool messagebus_publish(topic_t *topic, void *buf, size_t buf_len);

/** Reads the content of a single topic.
 *
 * @parameter [in] topic A pointer to the topic to read.
 * @parameter [out] buf Pointer where the read data will be stored.
 * @parameter [out] buf_len Length of the buffer.
 *
 * @returns true if the topic was published on at least once.
 * @returns false if the topic was never published to
 */
bool messagebus_read(topic_t *topic, void *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif
#endif
