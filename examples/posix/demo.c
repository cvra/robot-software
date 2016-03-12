#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include "../../messagebus.h"
#include "port.h"

messagebus_t bus;

static void* producer(void *p)
{
    topic_t *topic;
    int counter = 0;

    while (1) {
        topic = messagebus_find_topic(&bus, "myint");
        printf("[publisher] writing %d on topic %s\n", counter, topic->name);
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
    int consumer_number = (int)p;

    while (1) {
        topic = messagebus_find_topic(&bus, "myint");

        messagebus_wait(topic, &received, sizeof received);
        printf("[consumer %d] read %d on topic %s\n",
               consumer_number, received, topic->name);
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
    pthread_cond_t topic_cond = PTHREAD_COND_INITIALIZER;

    condvar_wrapper_t wrapper = {&topic_lock, &topic_cond};

    topic_init(&topic, &topic_lock, &wrapper, &buffer, sizeof buffer);

    messagebus_advertise_topic(&bus, &topic, "myint");

    pthread_t producer_thd, consumer_thd;

    /* Creates a few consumer threads. */
    pthread_create(&consumer_thd, NULL, consumer, (void *)1);
    pthread_create(&consumer_thd, NULL, consumer, (void *)2);
    pthread_create(&consumer_thd, NULL, consumer, (void *)3);

    /* Creates the producer thread */
    pthread_create(&producer_thd, NULL, producer, NULL);

    while(1) {
    }
}
