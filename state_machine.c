#include <czmq.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define FSM_OK      0
#define FSM_ERROR   1

#define INIT  0
#define LOGIN 1
#define SETUP 2
#define SINFO 3
#define RINFO 4

typedef struct
{
    int time_limit;
    int data_type;
} Settings;

typedef struct
{
    int current_state;
    bool is_initialized;
    bool login;
    bool is_setted;
    int send_request;
    char *buffer;
    unsigned int buf_len;
    Settings current_setup;
} State;

char *token = 
"DE369E3C8DD238F8CAD9066E55C2DD2B6250F84ED23AB10135DA3AF91F31D720984B32AD3F4501E8A653DF4B83F8272D2B03FC5C6FB7A54CBB09DA3582A08450C3D71B5E33E2D7795A5A0CA29593AC20F1DC67CE4777EC8AC8F779840B6A9BE7";

State state;

int
fsm_create ()
{
    state.current_state = INIT;
    state.is_initialized = false;
    state.login = false;
    state.is_setted = true;
    state.send_request = 0;

    return FSM_OK;
}

int
fsm_create_setup (settings setup)
{
    state.current_setup = setup;
    state.is_setted = false;
    
    return FSM_OK;
}

int
fsm_init ()
{
    if (init_tcp_connection () == TCP_OK)
    {
        state.is_initialized = true;
        return FSM_OK;
    }
    else
    {
        state.is_initialized = false;
        return FSM_ERROR;
    }
}

int
fsm_send_setup ()
{
    return FSM_OK;
}

void
fsm_transition ()
{
    switch (state.current_state = INIT)
    {
        case INIT:
            if (fsm_init () == FSM_OK);
                init_connection();
                state.current_state = LOGIN;
            break;

        case LOGIN:
            if (fsm_login () == FSM_OK)
            {
                if (state.is_setted == true)
                    state.current_state = SINFO;
                else
                    state.current_state = SETUP;
            }
            break;

        case SETUP:
            if (fsm_send_setup () == FSM_OK)
            {
                state.current_state = SINFO;
            }
            break;

        case SINFO:
            if (fsm_send_info () == FSM_OK)
            {
                tcp_send_position (state.buffer, state.buf_len);
                state.current_state = RINFO;
                // state.send_request = -1;
            }
            break;

        case RINFO:
            if (fsm_request_info () == FSM_OK)
            {
                tcp_rcv_correction (state.buffer, state.buf_len);
                state.current_state = SINFO;
                // state.send_request = 1;
            }
            break;
    }

    return;
}

// send token and request new port
void
init_connection ()
{
    tcp_init_connection (NULL, NULL);

    send_token (token, strlen (token));
}

void
fsm_update_data (char *buffer, unsigned int len)
{
    state.buffer = buffer;
    state.buf_len = len;

    return FSM_OK;
}
