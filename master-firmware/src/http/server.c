#include <error/error.h>
#include <fatfs/ff.h>
#include <lwip/opt.h>
#include <lwip/arch.h>
#include <lwip/api.h>

#include "server.h"

#if LWIP_NETCONN == 0
#error "HTTP server requires the netconn API"
#endif

#define HTTP_PORT 80

static const char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/plain\r\n\r\n";
static const char http_error[] = "HTTP/1.1 500 Internal Server Error\r\n\r\n<h1>Internal server error</h1><br>Cannot open log file.";
static const char http_index_html[] =
    "<html><head><title>Congrats!</title></head><body><h1>Welcome to our lwIP HTTP server!</h1><p>This is a small test page, served by httpserver-netconn.</body></html>";

/** Serve one HTTP connection accepted in the http thread */
static void
http_server_netconn_serve(struct netconn* conn)
{
    struct netbuf* inbuf;
    char* buf;
    u16_t buflen;
    err_t err;

    /* Read the data from the port, blocking if nothing yet there.
       We assume the request (the part we care about) is in one netbuf */
    err = netconn_recv(conn, &inbuf);

    if (err != ERR_OK) {
        goto end;
    }

    netbuf_data(inbuf, (void**)&buf, &buflen);

    /* Is this an HTTP GET command? (only check the first 5 chars, since
       there are other formats for GET, and we're keeping it very simple )*/
    if (buflen >= 5 && buf[0] == 'G' && buf[1] == 'E' && buf[2] == 'T' && buf[3] == ' ' && buf[4] == '/') {
        FRESULT err;
        FIL logfile_fp;
        err = f_open(&logfile_fp, "/log.txt", FA_READ);

        if (err != FR_OK) {
            WARNING("Cannot open log file.");
            /* Send an error 500. */
            netconn_write(conn, http_error, sizeof(http_error) - 1,
                          NETCONN_NOCOPY);
        } else {
            /* Send HTTP header */
            netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);

            do {
                static char buffer[128];
                UINT byte_count;
                f_read(&logfile_fp, buffer, sizeof(buffer), &byte_count);
                err = netconn_write(conn, buffer, byte_count, NETCONN_COPY);
            } while (!f_eof(&logfile_fp) && err == ERR_OK);
        }
    }

end:
    /* Close the connection (server closes in HTTP) */
    netconn_close(conn);

    /* Delete the buffer (netconn_recv gives us ownership,
       so we have to make sure to deallocate the buffer) */
    netbuf_delete(inbuf);
}

/** The main function, never returns! */
static THD_FUNCTION(http_server_thread, arg)
{
    struct netconn *conn, *newconn;
    err_t err;
    LWIP_UNUSED_ARG(arg);

    chRegSetThreadName("http");

    NOTICE("starting HTTP server");

    /* Create a new TCP connection handle */
    /* Bind to port 80 (HTTP) with default IP address */
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, IP_ADDR_ANY, HTTP_PORT);

    if (conn == NULL) {
        ERROR("cannot bind to port %d.", HTTP_PORT);
        goto end;
    }

    /* Put the connection into LISTEN state */
    netconn_listen(conn);

    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while (err == ERR_OK);
    WARNING("netconn_accept received error %d, aborting.", err);
    netconn_close(conn);

end:
    netconn_delete(conn);
    chThdExit(MSG_OK);
}

/** Initialize the HTTP server (start its thread) */
void http_server_start(void)
{
    static THD_WORKING_AREA(http_server_wa, 1024);
    chThdCreateStatic(http_server_wa, sizeof(http_server_wa), NORMALPRIO,
                      http_server_thread, NULL);
}
