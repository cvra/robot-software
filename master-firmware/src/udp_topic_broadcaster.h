#ifndef UDP_TOPIC_BROADCASTER_H
#define UDP_TOPIC_BROADCASTER_H

/** @file udp_topic_broadcaster.h
 *
 * Module that watches the internal bus and sends them over UDP for debug and
 * external processing on a companion computer.
 *
 *
 * 1. The first thread is responsible for reacting to a message sent on the
 * bus. It must do so very quickly because all messages sent while it is
 * processing are lost. In order to do so, it has a higher priority than most
 * threads (HIGHPRIO). It encodes the message as protobuf, and sends it to the
 * other thread for processing.
 * 2. The other thread waits for a message to come and sends it over UDP.  It
 * has normal priority and can take as long as it needs, since messages are
 * stored into a buffer.
 */

#ifdef __cplusplus
extern "C" {
#endif

void udp_topic_broadcast_start(void);

#ifdef __cplusplus
}
#endif

#endif
