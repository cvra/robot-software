#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "../../messagebus.h"
#include "port.h"

messagebus_t bus;

static void* producer(void *p)
{
    messagebus_topic_t *topic;
    int counter = 0;

    char *topic_name = (char *)p;

    printf("[publisher] waiting for topic \"%s\"\n", topic_name);

    while (1) {
        topic = messagebus_find_topic_blocking(&bus, topic_name);
        printf("[publisher] writing %d on topic %s\n", counter, topic->name);
        messagebus_topic_publish(topic, &counter, sizeof counter);
        counter += 1;
        sleep(2);
    }

    return NULL;
}

static void *observer(void *p)
{
    messagebus_watchgroup_t group;

    condvar_wrapper_t wrapper = {PTHREAD_MUTEX_INITIALIZER,
                                 PTHREAD_COND_INITIALIZER};

    messagebus_watcher_t watchers[2];
    messagebus_watchgroup_init(&group, &wrapper, &wrapper);
    messagebus_watchgroup_watch(&watchers[0],
                                &group,
                                messagebus_find_topic_blocking(&bus, "foo"));
    messagebus_watchgroup_watch(&watchers[1],
                                &group,
                                messagebus_find_topic_blocking(&bus, "bar"));

    while (1) {
        messagebus_topic_t *topic;
        topic = messagebus_watchgroup_wait(&group);
        printf("[observer] Received a message of size %ld on \"%s\"\n",
               topic->buffer_len,
               topic->name);
    }
}

static void create_topic(const char *name)
{
    messagebus_topic_t *topic = malloc(sizeof(messagebus_topic_t));
    int *buffer = malloc(sizeof(int));
    condvar_wrapper_t *sync = malloc(sizeof(condvar_wrapper_t));
    pthread_mutex_init(&sync->mutex, NULL);
    pthread_cond_init(&sync->cond, NULL);

    messagebus_topic_init(topic, sync, sync, buffer, sizeof(int));
    messagebus_advertise_topic(&bus, topic, name);
}

int main(int argc, const char **argv)
{
    (void) argc;
    (void) argv;

    /* Create the message bus. */
    condvar_wrapper_t bus_sync = {PTHREAD_MUTEX_INITIALIZER,
                                  PTHREAD_COND_INITIALIZER};
    messagebus_init(&bus, &bus_sync, &bus_sync);

    /* Creates a topic and publish it on the bus. */
    create_topic("foo");
    create_topic("bar");

    /* Create publishers */
    pthread_t producer_foo_thd, producer_bar_thd;
    pthread_create(&producer_foo_thd, NULL, producer, "foo");
    sleep(1);
    pthread_create(&producer_bar_thd, NULL, producer, "bar");

    /* Observer thread */
    pthread_t observer_thd;
    pthread_create(&observer_thd, NULL, observer, NULL);

    while (1) {
    }
}
