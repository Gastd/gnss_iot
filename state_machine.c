#include <czmq.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#define INIT  0
#define LOGIN 1
#define SETUP 2
#define SINFO 3
#define RINFO 4

struct
{
    int time_limit;
    int data_type;
}Settings;

struct
{
    int current_state;
    bool is_initialized;
    bool login;
    bool is_setted;
    int send_request;
    Settings current_setup;
}State;

State state;

int
fsm_create()
{
    state.current_state = INIT;
    state.is_initialized = false;
    state.login = false;
    state.is_setted = true;
    state.send_request = 0;

    return FSM_OK;
}

int
fsm_create_setup(settings setup)
{
    state.current_setup = setup;
    state.is_setted = false;
    
    return FSM_OK;
}

int
fsm_init()
{
    if (init_tcp_connection() == TCP_OK)
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
fsm_send_setup()
{
    return FSM_OK;
}

void
fsm_transition()
{
    switch (state.current_state = INIT)
    {
        case INIT:
            if (fsm_init() == FSM_OK);
                state.current_state = LOGIN;
            break;

        case LOGIN:
            if (fsm_login() == FSM_OK)
            {
                if (state.is_setted == true)
                    state.current_state = SINFO;
                else
                    state.current_state = SETUP;
            }
            break;

        case SETUP:
            if (fsm_send_setup() == FSM_OK)
            {
                state.current_state = SINFO;
            }
            break;

        case SINFO:
            if (fsm_send_info() == FSM_OK)
            {
                state.current_state = RINFO;
                // state.send_request = -1;
            }
            break;

        case RINFO:
            if (fsm_request_info() == FSM_OK)
            {
                state.current_state = SINFO;
                // state.send_request = 1;
            }
            break;
    }

    return;
}
