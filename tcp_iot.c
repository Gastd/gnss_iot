#include "tcp_iot.h"

#define TCP_OK      0
#define TCP_ERROR   1

char *ip_connection = "tcp://localhost";
char *tcp_port = "8888";

int sock = 0, valread;
int acesso;
// double coord[2] = {44, 66};
// char *token = "2B03FC5C6FB7A54CBB09DA3582A08450";
char buffer[1024] = {0};
struct sockaddr_in serv_addr;

int
tcp_init_connection (char *ip, char *port)
{
    if (ip != NULL)
        ip_connection = ip;

    if (port != NULL)
        tcp_port = port;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf ("\n Socket creation error \n");
        return TCP_ERROR;
    }

    int PORT = atoi (tcp_port);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8888);

    /* Convert IPv4 and IPv6 addresses from text to binary form */
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
    {
        printf ("\nInvalid address/ Address not supported \n");
        return TCP_ERROR;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf ("\nConnection Failed \n");
        return TCP_ERROR;
    }

    valread = read (sock, buffer, 1024);
    printf ("%s\n", buffer);

    return TCP_OK;
}

int
tcp_close_connection ()
{
    close (sock);

    return TCP_OK;
}

int
tcp_send_token (const void *buf, unsigned int len)
{
    int acesso;
    send (sock, buf, len, 0);
    printf ("Accessing with token: %s\n", (char *)buf);
    read(sock, &acesso, sizeof(acesso));
    sleep (1);

    return TCP_OK;
}

int
tcp_send_position (const void *buf, unsigned int len)
{
    send (sock, buf, len, 0);

    return TCP_OK;
}

int
tcp_rcv_correction (void *buf, unsigned int len)
{
    char* aux;
    read (sock, buf, len);

    // memcpy (buf, buffer, len);

    // printf ("%lf\n", coord[0]);
    // printf ("%lf\n", coord[1]);
    // sleep (1);

    return TCP_OK;
}
