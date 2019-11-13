#include "state_machine.h"

// char *token = 
// "DE369E3C8DD238F8CAD9066E55C2DD2B6250F84ED23AB10135DA3AF91F31D720984B32AD3F4501E8A653DF4B83F8272D2B03FC5C6FB7A54CBB09DA3582A08450C3D71B5E33E2D7795A5A0CA29593AC20F1DC67CE4777EC8AC8F779840B6A9BE7";

static fsm_state_iot_t m_state;

int fsm_init ();
int fsm_send_setup ();
int fsm_login ();
int fsm_send_info ();
int fsm_request_info ();

int
fsm_create ()
{
    m_state.current_state = FSM_STATE_INIT;
    m_state.is_initialized = false;
    m_state.login = false;
    m_state.is_setted = true;
    m_state.send_request = 0;

    return FSM_OK;
}

int
fsm_create_setup (fsm_settings_iot_t setup)
{
    m_state.current_setup = setup;
    m_state.is_setted = false;
    
    return FSM_OK;
}

int
fsm_init ()
{
    if (tcp_init_connection (NULL, NULL) == TCP_OK)
    {
        m_state.is_initialized = true;
        return FSM_OK;
    }
    // else
    // {
    //     m_state.is_initialized = false;
    //     return FSM_ERROR;
    // }
}

int
fsm_send_setup ()
{
    return FSM_OK;
}

void
fsm_transition ()
{
    switch (m_state.current_state)
    {
        case FSM_STATE_INIT:
            if (fsm_init () == FSM_OK)
            {
                // init_connection ();
                m_state.current_state = FSM_STATE_LOGIN;
            }
            break;

        case FSM_STATE_LOGIN:
            if (fsm_login () == FSM_OK)
            {
                if (m_state.is_setted == true)
                    m_state.current_state = FSM_STATE_SINFO;
                else
                    m_state.current_state = FSM_STATE_SETUP;
            }
            break;

        case FSM_STATE_SETUP:
            if (fsm_send_setup () == FSM_OK)
            {
                m_state.current_state = FSM_STATE_SINFO;
            }
            break;

        case FSM_STATE_SINFO:
            if (fsm_send_info () == FSM_OK)
            {
                tcp_send_position (m_state.buffer, m_state.buf_len);
                m_state.current_state = FSM_STATE_RINFO;
                // m_state.send_request = -1;
            }
            break;

        case FSM_STATE_RINFO:
            if (fsm_request_info () == FSM_OK)
            {
                tcp_rcv_correction (m_state.buffer, m_state.buf_len);
                m_state.current_state = FSM_STATE_SINFO;
                // m_state.send_request = 1;
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

    tcp_send_token (m_state.current_setup.token, strlen (m_state.current_setup.token));
}

int
fsm_login ()
{
    if (tcp_send_token (m_state.current_setup.token, strlen (m_state.current_setup.token)) == TCP_OK )
        return FSM_OK;
    else
        return FSM_ERROR;
}

int
fsm_update_data (char *buffer, unsigned int len)
{
    m_state.buffer = buffer;
    m_state.buf_len = len;

    return FSM_OK;
}

int
fsm_get_data (char *buffer, unsigned int len)
{
    memcpy(buffer, m_state.buffer, len);

    return FSM_OK;
}


int fsm_send_info ()
{
    return FSM_OK;
}

int fsm_request_info ()
{
    return FSM_OK;
}

int
fsm_end()
{
    tcp_close_connection ();
}
