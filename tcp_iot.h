#include <czmq.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define TCP_OK      0
#define TCP_ERROR   1

int
tcp_init_connection (char *ip, char *port);

int
tcp_close_connection ();

int
tcp_send_token (const void *buf, unsigned int len);

int
tcp_send_position (const void *buf, unsigned int len);

int
tcp_rcv_correction (void *buf, unsigned int len);
