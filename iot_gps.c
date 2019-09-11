#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define MAX_BYTES  100

uint8_t gps_data_[MAX_BYTES];
uint8_t time[10];
uint8_t llh[5][10];
int fdd = 0;

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
                    memcpy(time, &gps_data_[begin], end - begin);
                    time[end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 2: // latitude
                {
                    end = i;
                    memcpy(llh[0], &gps_data_[begin], end - begin);
                    llh[0][end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 3: // noth or south
                {
                    end = i;
                    memcpy(llh[1], &gps_data_[begin], end - begin);
                    llh[1][end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 4: // logitude
                {
                    end = i;
                    memcpy(llh[2], &gps_data_[begin], end - begin);
                    llh[2][end+1] = '\0';
                    state++;
                    begin = i+1;
                    break;
                }
                case 5: // east or West
                {
                    end = i;
                    memcpy(llh[3], &gps_data_[begin], end - begin);
                    llh[3][end+1] = '\0';
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

    printf ("time: %s\n", time);
    printf ("latitude: %s %s\n", llh[0], llh[1]);
    printf ("logitude: %s %s\n", llh[2], llh[3]);

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

int main()
{
        char *portname = "/dev/ttyS0";

        int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
                        printf("error %d opening %s: %s\n", errno, portname, strerror (errno));
                        return 0;
        }

        // set_interface_attribs (fd, B115200, 0);         // set speed to 115,200 bps, 8n1 (no parity)
        set_interface_attribs (fd, B9600, 0);         // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (fd, 0);                                           // set no blocking

        nmea_parser();

        return 0;
}
