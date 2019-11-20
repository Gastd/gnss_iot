#include <termios.h>
#include <fcntl.h>
#include "tcp_iot.h"
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include "minmea.h"
#include "state_machine.h"

#define MAX_BYTES  100
#define INDENT_SPACES "  "

typedef struct
{
    uint8_t gps_time[10];
    uint8_t llh[4][10];
} Buffer;

double coord[2] = {0, 0};

int fdd = 0;

char gps_data_[MINMEA_MAX_LENGTH];

Buffer buf;

char *nmea_msg = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

int
get_nmea(/* const char * message */)
{
    uint8_t data_ready = 0, data_read, msg_length = 0;

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
    printf("%s", gps_data_);
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
nmea_parser_fake( const char * message )
{
    // msg_length = strlen(message);
    // strncpy((char *) gps_data_, message, msg_length);

    printf("%s", message);
    switch (minmea_sentence_id(message, false)) {
        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, message)) {
                printf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                        frame.latitude.value, frame.latitude.scale,
                        frame.longitude.value, frame.longitude.scale,
                        frame.speed.value, frame.speed.scale);
                printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                        minmea_rescale(&frame.latitude, 1000),
                        minmea_rescale(&frame.longitude, 1000),
                        minmea_rescale(&frame.speed, 1000));
                printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                        minmea_tocoord(&frame.latitude),
                        minmea_tocoord(&frame.longitude),
                        minmea_tofloat(&frame.speed));
            }
            else {
                printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GLL: {
            struct minmea_sentence_gll frame;
            if (minmea_parse_gll(&frame, message)) {
                printf(INDENT_SPACES "$xxGLL: UTC %d:%d:%d\n",
                       frame.time.hours,
                       frame.time.minutes,
                       frame.time.seconds);
                printf(INDENT_SPACES "$xxGLL: latitude (%d/%d)\n",
                       frame.latitude.value, frame.latitude.scale);
                printf(INDENT_SPACES "$xxGLL: longitude (%d/%d)\n",
                       frame.longitude.value, frame.longitude.scale);
                printf(INDENT_SPACES "$xxGLL: fix status %c\n", frame.status);
                printf(INDENT_SPACES "$xxGLL: fix mode %c\n", frame.mode);
            }
            else {
                printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GGA: {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, message)) {
                printf(INDENT_SPACES "$xxGGA: UTC %d:%d:%d\n",
                       frame.time.hours,
                       frame.time.minutes,
                       frame.time.seconds);
                printf(INDENT_SPACES "$xxGGA: latitude (%d/%d)\n",
                       frame.latitude.value, frame.latitude.scale);
                printf(INDENT_SPACES "$xxGGA: longitude (%d/%d)\n",
                       frame.longitude.value, frame.longitude.scale);
                printf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
                printf(INDENT_SPACES "$xxGGA: rcv tracking #%d satellites\n", frame.satellites_tracked);
                printf(INDENT_SPACES "$xxGGA: HDOP (%d/%d)\n", frame.hdop.value, frame.hdop.scale);
                printf(INDENT_SPACES "$xxGGA: altitude above mean seal level (%d/%d) %c\n",
                       frame.altitude.value, frame.altitude.scale, frame.altitude_units);
                printf(INDENT_SPACES "$xxGGA: height of geoid above WGS84 (%d/%d) %c\n",
                       frame.height.value, frame.height.scale, frame.height_units);
                printf(INDENT_SPACES "$xxGGA: DGPS age (%d/%d)\n",
                       frame.dgps_age.value, frame.dgps_age.scale);
            }
            else {
                printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GST: {
            struct minmea_sentence_gst frame;
            if (minmea_parse_gst(&frame, message)) {
                printf(INDENT_SPACES "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
                        frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                        frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                        frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
                printf(INDENT_SPACES "$xxGST fixed point latitude,longitude and altitude error deviation"
                       " scaled to one decimal place: (%d,%d,%d)\n",
                        minmea_rescale(&frame.latitude_error_deviation, 10),
                        minmea_rescale(&frame.longitude_error_deviation, 10),
                        minmea_rescale(&frame.altitude_error_deviation, 10));
                printf(INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                        minmea_tofloat(&frame.latitude_error_deviation),
                        minmea_tofloat(&frame.longitude_error_deviation),
                        minmea_tofloat(&frame.altitude_error_deviation));
            }
            else {
                printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GSA: {
            struct minmea_sentence_gsa frame;
            if (minmea_parse_gsa(&frame, message)) {
                printf(INDENT_SPACES "$xxGSA: Auto selection of 2D or 3D fix %c\n", frame.mode);
                printf(INDENT_SPACES "$xxGSA: fix type %d\n", frame.fix_type);
                printf(INDENT_SPACES "$xxGSA: PDOP (%d/%d)\n", frame.pdop.value, frame.pdop.scale);
                printf(INDENT_SPACES "$xxGSA: HDOP (%d/%d)\n", frame.hdop.value, frame.hdop.scale);
                printf(INDENT_SPACES "$xxGSA: VDOP (%d/%d)\n", frame.vdop.value, frame.vdop.scale);
                printf(INDENT_SPACES "$xxGSA: PRNs used for fix ");
                for (int i = 0; i < 12; i++)
                    printf("%d, ", frame.sats[i]);
                printf("\n");
            }
            else {
                printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, message)) {
                printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                printf(INDENT_SPACES "$xxGSV: sattelites in view: %d\n", frame.total_sats);
                for (int i = 0; i < 4; i++)
                    printf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                        frame.sats[i].nr,
                        frame.sats[i].elevation,
                        frame.sats[i].azimuth,
                        frame.sats[i].snr);
            }
            else {
                printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_VTG: {
           struct minmea_sentence_vtg frame;
           if (minmea_parse_vtg(&frame, message)) {
                printf(INDENT_SPACES "$xxVTG: true track degrees = %f\n",
                       minmea_tofloat(&frame.true_track_degrees));
                printf(INDENT_SPACES "        magnetic track degrees = %f\n",
                       minmea_tofloat(&frame.magnetic_track_degrees));
                printf(INDENT_SPACES "        speed knots = %f\n",
                        minmea_tofloat(&frame.speed_knots));
                printf(INDENT_SPACES "        speed kph = %f\n",
                        minmea_tofloat(&frame.speed_kph));
           }
           else {
                printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
           }
        } break;

        case MINMEA_SENTENCE_ZDA: {
            struct minmea_sentence_zda frame;
            if (minmea_parse_zda(&frame, message)) {
                printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                       frame.time.hours,
                       frame.time.minutes,
                       frame.time.seconds,
                       frame.date.day,
                       frame.date.month,
                       frame.date.year,
                       frame.hour_offset,
                       frame.minute_offset);
            }
            else {
                printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
            }
        } break;

        case MINMEA_INVALID: {
            printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
        } break;

        default: {
            printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
        } break;
    }

    // printf ("time: %s\n", buf.gps_time);
    // printf ("latitude: %s %s\n", buf.llh[0], buf.llh[1]);
    // printf ("logitude: %s %s\n", buf.llh[2], buf.llh[3]);

    return 0;
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

char *dev_token = "44CBFA8583976546994071A680C01C5A";

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
    fsm_settings_iot_t dev_setup = 
    {
        .time_limit = -1,
        .data_type  = -1,
        .token      = dev_token
    };

    fsm_create ();
    fsm_create_setup (dev_setup);
    fsm_transition ();
    char temp[30];

    while (1)
    {
        nmea_parser_fake (nmea_msg);
        sprintf (temp, "%s %s", (char *)buf.llh[0], (char *)buf.llh[2]);
        printf ("w/o correction: %s\n", temp);
        fsm_update_data (temp, 30);
        fsm_transition ();
        fsm_get_data (temp, 30);
        temp[29] = '\0';
        printf ("with correction: %s\n", temp);
        sleep (1);
    }

    fsm_end ();
    return 0;
}
