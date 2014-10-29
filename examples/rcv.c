#include <stdio.h>
#include "serial_datagram.h"

void callback_fn(const void *dtgrm, size_t len)
{
    printf("len %ld ", len);
    printf("[");
    int i;
    for (i = 0; i < len ; i++)
        printf("%02hhx ", ((const char*)dtgrm)[i]);
    printf("] '%s'\n", dtgrm);
    fflush(stdout);
}

int main(void)
{
    static char buf[1000];
    serial_datagram_rcv_handler_t h;
    serial_datagram_rcv_handler_init(&h, buf, sizeof(buf), callback_fn);
    while (1) {
        char in[10];
        int len = fread(in, sizeof(char), sizeof(in), stdin);
        if (len == 0)
            return 0;
        int err = serial_datagram_receive(&h, in, len);
        if (err) {
            printf("error %d\n", err);
        }
    }
}
