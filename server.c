//  Hello World sever
#include <zmq.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

int main (void)
{
    //  Socket to talk to clients
    void *context = zmq_ctx_new ();
    void *responder = zmq_socket (context, ZMQ_STREAM);
    int rc = zmq_bind (responder, "tcp://*:8888");
    if (rc != 0)    return -1;

    while (1) {
        char buffer [40];
        zmq_recv (responder, buffer, 40, 0);
        printf ("Received Hello %s\n", buffer);
        // sleep (1);          //  Do some 'work'
        zmq_send (responder, "ok\0\0", 4, 0);
    }
    return 0;
}
