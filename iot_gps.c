#include <czmq.h>
#include <termios.h>
#include <fcntl.h>
#include "tcp_iot.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define MAX_BYTES  100

typedef struct
{
    uint8_t gps_time[10];
    uint8_t llh[5][10];
} Buffer;

double coord[2] = {0, 0};

int fdd = 0;

uint8_t gps_data_[MAX_BYTES];

Buffer buf;

// char *str = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

int
nmea_parser(/* const char * message */)
{
    uint8_t data_ready = 0, data_read, msg_length;

    for(int i = 0; (!data_ready)&&(i < MAX_BYTES); i++)
    {
        if ((errno == read (fdd, &data_read, sizeof (uint8_t))) != 0)
        {
                printf ("ERROR: byte read failed\n");
                break;
        }

        gps_data_[i] = data_read;

        if ((gps_data_[i] == '\n') && (gps_data_[i-1] == '\r'))
        {
            data_ready = 1;
            msg_length = i;
        }

    }

    if (strncmp((const char *)gps_data_, "GPGGA", 5) == 0)
    {
        for(int i = 0, state = 0, begin = 0, end = 0; i < msg_length; ++i)
        {
            if (gps_data_[i] != ',')
                continue;

            switch (state)
            {
                case 0: // msg header
                {
                    state++;
                    begin = i+1;
                    break;
                }
                case 1: // utc gps time
                {
                    end = i;
                    memcpy(&buf.gps_time, &gps_data_[begin], end - begin);
                    buf.gps_time[end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 2: // latitude
                {
                    end = i;
                    memcpy(&buf.llh[0], &gps_data_[begin], end - begin);
                    buf.llh[0][end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 3: // noth or south
                {
                    end = i;
                    memcpy(&buf.llh[1], &gps_data_[begin], end - begin);
                    buf.llh[1][end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 4: // logitude
                {
                    end = i;
                    memcpy(&buf.llh[2], &gps_data_[begin], end - begin);
                    buf.llh[2][end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 5: // east or West
                {
                    end = i;
                    memcpy(&buf.llh[3], &gps_data_[begin], end - begin);
                    buf.llh[3][end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                // case 2: // latitude
                // {
                //     end = i;
                //     memcpy(time, &gps_data_[begin], end - begin);
                // }
            }

        }
    }

    printf ("time: %s\n", buf.gps_time);
    printf ("latitude: %s %s\n", buf.llh[0], buf.llh[1]);
    printf ("logitude: %s %s\n", buf.llh[2], buf.llh[3]);

    return data_ready;
}


int
set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("%d error from tcgetattr\n", errno);
            return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
            printf ("error %d from tcsetattr\n", errno);
            return -1;
    }
    return 0;
}

void
set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("error %d from tggetattr\n", errno);
            return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf ("error %d setting term attributes\n", errno);
}

uint32_t ByteSwap (uint32_t n)
{ 
   return ( ((n & 0x000000FF)<<24) + ((n & 0x0000FF00)<<8) + ((n & 0x00FF0000)>>8) + (( n & 0xFF000000)>>24) );
}

char *token = "2B03FC5C6FB7A54CBB09DA3582A08450";

// int xmain()
// {
//     void *ctx = zmq_ctx_new();
//     void *sock = zmq_socket(ctx, ZMQ_STREAM);
//     int rc = zmq_connect(sock, "tcp://127.0.0.1:5555");
//     if (rc != 0)
//         printf ("%s", zmq_strerror(zmq_errno()));
//     assert(rc == 0);

//     // Retrieve the socket identity. This is important in this
//     // particular scenario because when sending data using a
//     // 'ZMQ_STREAM' socket the implementation uses the first
//     // frame as the *identity* to route the message to. The ZMQ
//     // implementation strips off this first frame before sending
//     // the data to the endpoint.
//     //char *identity = zsocket_identity (sock);

//     // Must currently resort to the libzmq low-level lib to obtain
//     // raw identity information because CZMQ is returning the binary
//     // identity data as a char* and often there is 0 in the data which
//     // prematurely terminates the char* data.
//     uint8_t id [256];
//     size_t id_size = 256;
//     // rc = zmq_getsockopt (sock, ZMQ_IDENTITY, id, &id_size);
//     assert (rc == 0);

//     zframe_t *frame;
//     while (!zctx_interrupted)
//     {
//         zmsg_t *msg = zmsg_new ();
//         // Supply ZMQ with the identity to route the message to.
//         zmsg_addmem (msg, id, id_size);
//         // add message data.
//         zmsg_addstr (msg, "Hello");
//         zmsg_send(&msg, sock);


//         // When receiving TCP data, a 'ZMQ_STREAM' socket shall 
//         // prepend a frame containing the *identity* of the 
//         // originating peer to the message before passing it to 
//         // the application. Messages received are fair-queued from 
//         // among all connected peers.
//         //
//         // So in a multi-connection environment our simple
//         // assumption that we will receive a reply from the
//         // endpoint we just sent a message to is naive but will
//         // suffice for this simple test.

//         // Read off the *identity* first
//         frame = zframe_recv (sock);
//         if (!frame)
//             break; // interrupted
//         zframe_destroy (&frame);

//         // Now read off the message.
//         char *response = zstr_recv (sock);
//         printf ("Response: %s\n", response);
//         free (response);

//         // crude delay between consequtive requests
//         zclock_sleep (2000);
//     }

//     // zmq_ctx_destroy (&ctx);
//     return 0;


//     // for (request_nbr = 0; request_nbr != 10; request_nbr++) {
//     //     printf ("Sending Hello %ld…\n", strlen (token));
//     //     zmq_send (socket, token, 33, 0);
//     //     zmq_recv (socket, buffer, 4, 0);
//     //     printf ("Received World %d\n", request_nbr);
//     //     buffer[29] = '\0';
//     //     printf ("buffer: %s\n", buffer);
//     //     sleep (2);
//     // }

//     // zmq_close (socket);
//     // zmq_ctx_destroy (context);
//     // return 0;

//     char *portname = "/dev/ttyS0";

//     int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0)
//     {
//         printf("error %d opening %s: %s\n", errno, portname, strerror (errno));
//         return 0;
//     }

//     // set_interface_attribs (fd, B115200, 0);       // set speed to 115,200 bps, 8n1 (no parity)
//     set_interface_attribs (fd, B9600, 0);         // set speed to 9,600 bps, 8n1 (no parity)
//     set_blocking (fd, 0);                         // set no blocking

//     nmea_parser();

//     return 0;
// }

int main()
{
    tcp_init_connection (NULL, NULL);

    tcp_send_token (token, strlen (token));

    while (1)
    {
        tcp_send_position (coord, sizeof (coord));
        tcp_rcv_correction (coord, sizeof (coord));
        printf ("%f\n", coord[0]);
        printf ("%f\n", coord[1]);
    }
    tcp_close_connection ();
    return 0;
}
