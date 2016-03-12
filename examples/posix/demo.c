#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include "../../messagebus.h"
#include "port.h"

messagebus_t bus;

static void* producer(void *p)
{
    messagebus_topic_t *topic;
    int counter = 0;
    int producer_number = (int)p;

    while (1) {
        topic = messagebus_find_topic(&bus, "myint");
        printf("[publisher %d] writing %d on topic %s\n",
                producer_number, counter, topic->name);
        messagebus_topic_publish(topic, &counter, sizeof counter);
        counter += 1;
        sleep(2);
    }

    return NULL;
}

static void *consumer(void *p)
{
    messagebus_topic_t *topic;
    int received;
    int consumer_number = (int)p;

    while (1) {
        topic = messagebus_find_topic(&bus, "myint");

        messagebus_topic_wait(topic, &received, sizeof received);
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
    messagebus_topic_t topic;
    int buffer;

    condvar_wrapper_t wrapper = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};
    messagebus_topic_init(&topic, &wrapper, &wrapper, &buffer, sizeof buffer);

    messagebus_advertise_topic(&bus, &topic, "myint");

    /* Creates a few consumer threads. */
    pthread_t producer_thd, consumer_thd;
    pthread_create(&consumer_thd, NULL, consumer, (void *)1);
    pthread_create(&consumer_thd, NULL, consumer, (void *)2);
    pthread_create(&consumer_thd, NULL, consumer, (void *)3);

    /* Creates the producer threads, slightly offset */
    pthread_create(&producer_thd, NULL, producer, (void *)1);
    sleep(5);
    pthread_create(&producer_thd, NULL, producer, (void *)2);

    while(1) {
    }
}
