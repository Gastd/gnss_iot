#ifndef TCP_IOT_H
#define TCP_IOT_H

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <arpa/inet.h> /*close*/
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h> /*FD_SET, FD_ISSET, FD_ZERO macros*/

#define TCP_OK      0
#define TCP_ERROR   1

int tcp_init_connection (char *ip, char *port);

int tcp_close_connection ();

int tcp_send_token (const void *buf, unsigned int len);

int tcp_send_position (const void *buf, unsigned int len);

int tcp_rcv_correction (void *buf, unsigned int len);

#endif
