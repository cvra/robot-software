#include "can_uwb_ip_netif.hpp"

#include <ch.h>
#include <hal.h>

#include <cvra/uwb_beacon/DataPacket.hpp>

#include <lwip/opt.h>
#include <lwip/def.h>
#include <lwip/mem.h>
#include <lwip/pbuf.h>
#include <lwip/sys.h>
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <lwip/tcpip.h>
#include <netif/etharp.h>
#include <lwip/netifapi.h>

#if LWIP_DHCP
#include <lwip/dhcp.h>
#endif
using DataPacket = cvra::uwb_beacon::DataPacket;

/* Semaphores synchronizing communication between the lwip thread and the
 * UAVCAN thread. */
static BSEMAPHORE_DECL(sem_tx_ready, true);
static BSEMAPHORE_DECL(sem_tx_done, true);

static DataPacket tx_packet, rx_packet;

static MUTEX_DECL(lock_rx_packet);
static BSEMAPHORE_DECL(sem_rx_available, true);
static EVENTSOURCE_DECL(event_rx);

static uavcan::LazyConstructor<uavcan::Publisher<DataPacket>> data_pub;

static err_t low_level_output(struct netif* netif, struct pbuf* p);

static void data_packet_cb(const uavcan::ReceivedDataStructure<DataPacket>& msg)
{
    chMtxLock(&lock_rx_packet);
    rx_packet = msg;
    chMtxUnlock(&lock_rx_packet);
    chBSemSignal(&sem_rx_available);

    /* Wakes up IP thread */
    chEvtBroadcast(&event_rx);
}

err_t lwip_uwb_ip_netif_init(struct netif* interface)
{
    osalDbgAssert((interface != NULL), "netif != NULL");
    interface->state = NULL;
    interface->name[0] = 'w';
    interface->name[1] = 'l';

    /* Must be smaller than 1k */
    interface->mtu = 750;
    interface->hwaddr_len = ETHARP_HWADDR_LEN;
    interface->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

    interface->output = etharp_output;
    interface->linkoutput = low_level_output;

    return ERR_OK;
}

int can_uwb_ip_netif_init(uavcan::INode& node)
{
    static uavcan::Subscriber<DataPacket> subscriber(node);
    data_pub.construct<uavcan::INode&>(node);

    return subscriber.start(data_packet_cb);
}

static err_t low_level_output(struct netif* netif, struct pbuf* p)
{
    (void)netif;
    /* First, copy all the data */
    tx_packet = DataPacket();
    for (auto q = p; q != NULL; q = q->next) {
        uint8_t* buf = reinterpret_cast<uint8_t*>(q->payload);
        for (auto i = 0u; i < q->len; i++) {
            tx_packet.data.push_back(buf[i]);
        }
    }

    /* Signal the UAVCAN thread that a DataPacket is ready */
    chBSemSignal(&sem_tx_ready);

    /* Finally, wait for the UAVCAN to tell us we are ready. */
    chBSemSignal(&sem_tx_done);

    return ERR_OK;
}

bool lwip_uwb_ip_read(struct netif* netif, struct pbuf** pbuf)
{
    (void)netif;
    if (chBSemWaitTimeout(&sem_rx_available, TIME_IMMEDIATE) != MSG_OK) {
        return false;
    }

    chMtxLock(&lock_rx_packet);

    auto len = rx_packet.data.size();
    *pbuf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    auto can_msg_index = 0u;

    for (auto q = *pbuf; q != NULL; q = q->next) {
        uint8_t* buf = reinterpret_cast<uint8_t*>(q->payload);
        for (auto j = 0u; j < q->len; j++) {
            buf[j] = rx_packet.data[can_msg_index];
            can_msg_index++;
        }
    }

    chMtxUnlock(&lock_rx_packet);

    return true;
}

int can_uwb_ip_netif_spin(uavcan::INode& node)
{
    (void)node;
    /* Check if data is available for transmit */
    if (chBSemWaitTimeout(&sem_tx_ready, TIME_IMMEDIATE) == MSG_OK) {
        data_pub->broadcast(tx_packet);

        /* Signal the lwip thread that we are done with this. */
        chBSemSignal(&sem_tx_done);
    }

    return 0;
}

event_source_t* can_uwb_ip_netif_get_event_source(void)
{
    return &event_rx;
}
