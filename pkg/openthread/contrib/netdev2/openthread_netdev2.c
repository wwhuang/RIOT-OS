/*
 * Copyright (C) 2016 José Ignacio Alamos
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 * @author  José Ignacio Alamos <jialamos@uc.cl>
 */

#include "msg.h"
#include "ot.h"
//#include "openthread/openthread.h"
#include "openthread/platform/alarm.h"
#include "openthread/platform/uart.h"
#include "net/netdev2.h"
#include <assert.h>

#ifdef MODULE_OPENTHREAD_NCP
#include <ncp/ncp.h>
#endif

#define ENABLE_DEBUG (0)
#include "debug.h"

#include <errno.h>
#include "random.h"

#define OPENTHREAD_QUEUE_LEN      (8)
static msg_t _queue[OPENTHREAD_QUEUE_LEN];

static kernel_pid_t _pid;


/* OpenThread will call this when switching state from empty tasklet to non-empty tasklet. */
void otSignalTaskletPending(void)
{
    //Unused
}

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}


void otPlatSettingsInit(otInstance *aInstance) 
{
}

ThreadError otPlatSettingsGet(otInstance *aInstance, uint16_t aKey, int aIndex, uint8_t *aValue,
                              uint16_t *aValueLength)
{
    return kThreadError_None;
}

ThreadError otPlatSettingsSet(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength) 
{
    return kThreadError_None;
}

ThreadError otPlatSettingsAdd(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength) 
{
    return kThreadError_None;
}


ThreadError otPlatSettingsDelete(otInstance *aInstance, uint16_t aKey, int aIndex) 
{
    return kThreadError_None;
}


void *_openthread_event_loop(void *arg)
{
    _pid = thread_getpid();

    msg_init_queue(_queue, OPENTHREAD_QUEUE_LEN);
    netdev2_t *dev;
    msg_t msg;

    /* init OpenThread */
    otInstance* sInstance = otInstanceInit();
    assert(sInstance);

#ifdef MODULE_OPENTHREAD_CLI
    otCliUartInit();
#else
#ifdef MODULE_OPENTHREAD_NCP
    otNcpInit();
#endif
#endif
    /* enable OpenThread UART */
    otPlatUartEnable();


    while (1) {
        /* Process OpenThread tasklets */
        begin_mutex();
		otTaskletsProcess(sInstance);
        end_mutex();

        msg_receive(&msg);
        switch (msg.type) {
            case OPENTHREAD_XTIMER_MSG_TYPE_EVENT:
                /* Tell OpenThread a time event was received */
				printf("Timer fired\n");
                begin_mutex();
                otPlatAlarmFired(sInstance);
                end_mutex();
                break;
            case OPENTHREAD_NETDEV2_MSG_TYPE_EVENT:
                /* Received an event from driver */
                dev = (netdev2_t *) msg.content.ptr;
                dev->driver->isr(dev);
                break;
#if defined(MODULE_OPENTHREAD_CLI) || defined(MODULE_OPENTHREAD_NCP)
            case OPENTHREAD_SERIAL_MSG_TYPE_EVENT:
                /* Tell OpenThread about the receotion of a CLI command */
                begin_mutex();
                otPlatUartReceived((uint8_t *) msg.content.ptr, 1);
                end_mutex();
                break;
#endif

        }
    }

    return NULL;
}

void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    if (event == NETDEV2_EVENT_ISR) {
        assert(_pid != KERNEL_PID_UNDEF);
        msg_t msg;

        msg.type = OPENTHREAD_NETDEV2_MSG_TYPE_EVENT;
        msg.content.ptr = dev;

        if (msg_send(&msg, _pid) <= 0) {
            DEBUG("openthread_netdev2: possibly lost interrupt.\n");
        }
    }
    else {
        switch (event) {
            case NETDEV2_EVENT_RX_COMPLETE:
                DEBUG("openthread_netdev2: Reception of a pcket\n");
                recv_pkt(dev);
                break;
            case NETDEV2_EVENT_TX_COMPLETE:
            case NETDEV2_EVENT_TX_COMPLETE_DATA_PENDING:
            case NETDEV2_EVENT_TX_NOACK:
            case NETDEV2_EVENT_TX_MEDIUM_BUSY:
                DEBUG("openthread_netdev2: Transmission of a pcket\n");
                send_pkt(dev, event);
                break;
            default:
                break;
        }
    }
}

/* get OpenThread thread pid */
kernel_pid_t openthread_get_pid(void)
{
    return _pid;
}

/* starts OpenThread thread */
int openthread_netdev2_init(char *stack, int stacksize, char priority,
                            const char *name, netdev2_t *netdev)
{

    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;

    netopt_enable_t enable = NETOPT_ENABLE;
    netdev->driver->set(netdev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    _pid = thread_create(stack, stacksize,
                         priority, THREAD_CREATE_STACKTEST,
                         _openthread_event_loop, NULL, name);

    if (_pid <= 0) {
        return -EINVAL;
    }

    return _pid;
}
/** @} */