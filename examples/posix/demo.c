#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include "../../messagebus.h"

messagebus_t bus;

static void* producer(void *p)
{
    topic_t *topic;
    int counter = 0;

    while (1) {
        topic = messagebus_find_topic(&bus, "myint");
        printf("publishing %d on topic %p\n", counter, topic);
        messagebus_publish(topic, &counter, sizeof counter);
        counter += 1;
        sleep(2);
    }

    return NULL;
}

static void *consumer(void *p)
{
    topic_t *topic;
    int received;

    while (1) {
        topic = messagebus_find_topic(&bus, "myint");

        messagebus_read(topic, &received, sizeof received);
        printf("read %d on topic %p\n", received, topic);

        usleep(500000);
    }

    return NULL;
}

int main(int argc, const char **argv)
{
    (void) argc;
    (void) argv;

    /* Create the message bus. */
    pthread_mutex_t bus_lock = PTHREAD_MUTEX_INITIALIZER;
    messagebus_init(&bus, &bus_lock);

    /* Creates a topic and publish it on the bus. */
    topic_t topic;
    int buffer;
    pthread_mutex_t topic_lock = PTHREAD_MUTEX_INITIALIZER;
    topic_init(&topic, &topic_lock, &buffer, sizeof buffer);
    messagebus_advertise_topic(&bus, &topic, "myint");

    pthread_t producer_thd, consumer_thd;
    pthread_create(&producer_thd, NULL, producer, NULL);
    pthread_create(&consumer_thd, NULL, consumer, NULL);

    while(1) {
    }
}
