# List of the required lwIP files.
LWIP = 	lwip/

LWNETIFSRC = \
        ${LWIP}/src/netif/etharp.c

LWCORESRC = $(wildcard $(LWIP)/src/core/*.c) $(wildcard $(LWIP)/src/core/ipv4/*.c)

LWAPISRC = \
        ${LWIP}/src/api/api_lib.c \
        ${LWIP}/src/api/api_msg.c \
        ${LWIP}/src/api/err.c \
        ${LWIP}/src/api/netbuf.c \
        ${LWIP}/src/api/netdb.c \
        ${LWIP}/src/api/netifapi.c \
        ${LWIP}/src/api/sockets.c \
        ${LWIP}/src/api/tcpip.c

LWIP_BINDINGS_SRC = lwip_bindings/arch/sys_arch.c lwip_bindings/lwipthread.c

LWSRC = $(LWNETIFSRC) $(LWCORESRC) $(LWAPISRC) $(LWIP_BINDINGS_SRC)

LWINC = \
        lwip_bindings \
        ${LWIP}/src/include \
        ${LWIP}/src/include/ipv4

