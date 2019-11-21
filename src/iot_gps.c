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

#define MAX_BYTES  MINMEA_MAX_LENGTH
#define INDENT_SPACES "     "

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
    int i = 0, b = 0;
	memset(gps_data_, 0, sizeof gps_data_);
    for(i = 0; (!data_ready)&&(i < MAX_BYTES); i++)
    {
		//printf("i = %d e b = %d\n", i, b);
		
        if ((errno == read (fdd, &data_read, sizeof (uint8_t))) != 0)
        {
                printf ("ERROR: byte read failed\n");
                break;
        }
		
		//if ((data_read == 0x0a))
		//	printf("YES\n");
			
        if ((data_read == '$') && (b == 0))
        {
            gps_data_[b] = data_read;
            //printf("YES $\n");
            b++;
        }
        else if ((b > 0) && gps_data_[0] == '$')
        {
            gps_data_[b] = data_read;
            b++;
        }
        else if ((data_read == '\n'))
        {
            gps_data_[b] = data_read;
            gps_data_[b+1] = '\0';
            b++;
            data_ready = 1;
            msg_length = b;
            //printf("YES final de linha\n");
            break;
        }
        else
        {
            b = 0;
        }
        if ((data_read == '\n'))
        {
            gps_data_[b] = data_read;
            gps_data_[b+1] = '\0';
			b++;
            data_ready = 1;
            msg_length = b;
            //printf("YES final de linha\n");
            break;
        }
    }
    printf("gps_data_: %s", gps_data_);

    // printf ("time: %s\n", buf.gps_time);
    // printf ("latitude: %s %s\n", buf.llh[0], buf.llh[1]);
    // printf ("logitude: %s %s\n", buf.llh[2], buf.llh[3]);

    return msg_length;
}

int
nmea_parser_fake( const char * message, int len)
{
    // msg_length = strlen(message);
    // char str[80];
    // strncpy(str, message, len);

    printf("getting %d bytes in message: %s", len, message);
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
                int i = 0;
                for (i = 0; i < 12; i++)
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
                int i = 0;
                for (i = 0; i < 4; i++)
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
    tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

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
            return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf ("error %d setting term attributes\n", errno);
}

// uint32_t ByteSwap (uint32_t n)
// { 
//    return ( ((n & 0x000000FF)<<24) + ((n & 0x0000FF00)<<8) + ((n & 0x00FF0000)>>8) + (( n & 0xFF000000)>>24) );
// }

 char *dev_token = "44CBFA8583976546994071A680C01C5A";
/*
int main()
{
    char *portname = "/dev/ttyACM0";

    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("error %d opening %s: %s\n", errno, portname, strerror (errno));
        return 0;
    }

    // set_interface_attribs (fd, B115200, 0);       // set speed to 115,200 bps, 8n1 (no parity)
    set_interface_attribs (fd, B9600, 0);         // set speed to 9,600 bps, 8n1 (no parity)
    set_blocking (fd, 0);                         // set no blocking

    nmea_parser();

    return 0;
}
*/

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

    char *portname = "/dev/ttyACM0";

    fdd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fdd < 0)
    {
        printf("error %d opening %s: %s\n", errno, portname, strerror (errno));
        return 0;
    }

    set_interface_attribs (fdd, B9600, 0);         // set speed to 9,600 bps, 8n1 (no parity)
    set_blocking (fdd, 1);                         // set no blocking

    while (1)
    {
        int len = (int) get_nmea ();
        //fgets(gps_data_, sizeof(gps_data_), fdd);
        printf("len %d\n", len);
        nmea_parser_fake (gps_data_, len);
        //sprintf (temp, "%s %s", (char *)buf.llh[0], (char *)buf.llh[2]);
        //printf ("w/o correction: %s\n", temp);
        //fsm_update_data (temp, 30);
        //fsm_transition ();
        //fsm_get_data (temp, 30);
        //temp[29] = '\0';
        //printf ("with correction: %s\n", temp);
        // sleep (1);
    }

    fsm_end ();
    return 0;
}
