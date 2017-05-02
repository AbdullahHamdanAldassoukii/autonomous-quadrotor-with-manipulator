/*
 * comms.h
 *
 *  Created on: Oct 5, 2014
 *      Author: jonathan
 */

#ifndef COMMS_H_
#define COMMS_H_

#include <stdint.h>
#include <stdbool.h>

//#include "datastruct/container.h"
#include "container.h"

typedef enum comms_channel_t
{
    CHANNEL_ALL,
    CHANNEL_KILL,
    CHANNEL_CHANNELS,
    CHANNEL_TELEMETRY,
    CHANNEL_LED,
    CHANNEL_CFG_USB_SN,
    CHANNEL_CFG_DATA_FREQUENCY,
    CHANNEL_DEBUG,
    CHANNEL_NUM_CHANNELS

} comms_channel_t;

typedef enum comms_status_t
{
    COMMS_STATUS_INVALID_ARGUMENT,
    COMMS_STATUS_BUFFER_FULL,
    COMMS_STATUS_WAITING,
    COMMS_STATUS_IN_PROGRESS,
    COMMS_STATUS_NO_ACTION,
    COMMS_STATUS_DONE,
    COMMS_STATUS_NUM_STATUSES

} comms_status_t;

typedef struct comms_t comms_t;

extern container_funcs_t *comms_cfuncs;

typedef void (*publisher_t)(container_t *data);

typedef void (*subscriber_t)(void *usr, uint16_t id, comms_channel_t channel,
                             const uint8_t *msg, uint16_t len);




comms_t* comms_create(uint32_t buf_len_rx, uint32_t buf_len_tx,
                      uint32_t num_tx_orig, publisher_t publisher);

void comms_subscribe(comms_t *comms, comms_channel_t channel,
                     subscriber_t subscriber, void *usr);

comms_status_t comms_publish(comms_t *comms,
                             comms_channel_t channel,
                             const uint8_t *msg,
                             uint16_t msg_len);

comms_status_t comms_publish_id(comms_t *comms,
                                uint16_t id,
                                comms_channel_t channel,
                                const uint8_t *msg,
                                uint32_t tx_orig_num,
                                uint16_t msg_len);

inline comms_status_t comms_transmit(comms_t *comms);

void comms_handle(comms_t *comms, uint8_t byte);

void comms_destroy(comms_t *comms);

#endif /* COMMS_H_ */
