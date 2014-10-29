#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial_datagram.h"

void sendfn(void *arg, const void *buf, size_t len)
{
    (void)arg;
#if 0 /* print frame in hex to stderr */
    int i;
    for (i = 0; i < len ; i++)
        fprintf(stderr, "%02hhx ", buf[i]);
#endif
    fwrite(buf, sizeof(char), len, stdout);
    fflush(stdout);
}

int main(void)
{
    char msg[1000];
    while (1) {
        fgets(msg, sizeof(msg), stdin);
        serial_datagram_send(msg, strlen(msg)+1, sendfn, NULL);
    }
    getc(stdin);
    return 0;
}
