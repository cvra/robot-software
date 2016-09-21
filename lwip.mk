# List of the required lwIP files.

LWNETIFSRC = $(LWIP)/src/netif/etharp.c

LWCORESRC = $(wildcard $(LWIP)/src/core/*.c) $(wildcard $(LWIP)/src/core/ipv4/*.c)

LWAPISRC = $(wildcard $(LWIP)/src/api/*.c)

LWIP_BINDINGS_SRC = src/lwip_bindings/arch/sys_arch.c src/lwip_bindings/lwipthread.c

LWSRC = $(LWNETIFSRC) $(LWCORESRC) $(LWAPISRC) $(LWIP_BINDINGS_SRC)

LWINC = src/lwip_bindings $(LWIP)/src/include $(LWIP)/src/include/ipv4

