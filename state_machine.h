#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdbool.h>
#include "tcp_iot.h"

#define FSM_OK      0
#define FSM_ERROR   1

typedef enum
{
    FSM_STATE_INIT,
    FSM_STATE_LOGIN,
    FSM_STATE_SETUP,
    FSM_STATE_SINFO,
    FSM_STATE_RINFO
} fsm_states_t;

typedef struct
{
    int time_limit;
    int data_type;
    char *token;
} fsm_settings_iot_t;

typedef struct
{
    int current_state;
    bool is_initialized;
    bool login;
    bool is_setted;
    int send_request;
    char *buffer;
    unsigned int buf_len;
    fsm_settings_iot_t current_setup;
} fsm_state_iot_t;


int fsm_create ();

int fsm_create_setup (fsm_settings_iot_t setup);

void fsm_transition ();

// send token and request new port
void init_connection ();

int fsm_update_data (char *buffer, unsigned int len);

int fsm_get_data (char *buffer, unsigned int len);

int fsm_end();

#endif
