/*
 * comms.c
 *
 *  Created on: Oct 5, 2014
 *      Author: Jonathan
 */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "comms.h"
#include "circular.h"

#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))

typedef struct subscriber_container_t
{
    subscriber_t subs;
    void *usr;

} subscriber_container_t;


struct comms_t
{
    uint8_t *buf_rx;
    uint16_t buf_rx_len;

    int32_t num_tx_origins;
    int32_t curr_tx_orig_num;
    container_t **buf_tx; // array of tx msg buffers

    publisher_t publisher;

    // An array of pointers to arrays of
    // subscriber containers
    subscriber_container_t **subscribers[CHANNEL_NUM_CHANNELS];
    uint8_t num_subscribers[CHANNEL_NUM_CHANNELS];

    // Variables to handle decoding
    comms_channel_t decode_channel;
    uint16_t decode_data_len;
    uint16_t decode_num_data_read;
    uint16_t decode_checksum;
    uint16_t decode_id;
    uint8_t  decode_state;
    uint8_t   checksum_rx1,  checksum_rx2;
    uint8_t  *checksum_tx1, *checksum_tx2; // All checksum arrays are of length num_tx_origins
};


#define START_BYTE1 0xB1
#define START_BYTE2 0x75

#define SELECTED_MSG_NONE -1

static void fletcher_checksum_clear_rx(comms_t *comms);
static void fletcher_checksum_add_byte_rx(comms_t *comms, uint8_t byte);
static uint16_t fletcher_checksum_calculate_rx(comms_t *comms);
static void fletcher_checksum_clear_tx(comms_t *comms, uint32_t tx_origin_num);
static void fletcher_checksum_add_byte_tx(comms_t *comms, uint32_t tx_origin_num, uint8_t byte);
static uint16_t fletcher_checksum_calculate_tx(comms_t *comms, uint32_t tx_origin_num);
static inline uint16_t fletcher_checksum_calculate(uint8_t checksum1, uint8_t checksum2);

container_funcs_t *comms_cfuncs;

comms_t* comms_create(uint32_t buf_rx_len, uint32_t buf_tx_len,
                      uint32_t num_tx_origins, publisher_t publisher)
{
    if(num_tx_origins == 0) return NULL;

    // This initializes everything to null in ret. Must be calloc not malloc
    comms_t *ret = (comms_t*) calloc(1, sizeof(comms_t));
    if(ret == NULL) return NULL;

    comms_cfuncs = circular_funcs_init();

    ret->buf_rx_len = buf_rx_len;
    ret->buf_rx = (uint8_t*) calloc(ret->buf_rx_len, sizeof(uint8_t));
    if(ret->buf_rx == NULL)
    {
        comms_destroy(ret);
        return NULL;
    }

    // This makes our lives easier when handling
    // the buffering of messages to be published
    if(buf_tx_len == 0) buf_tx_len = 1;

    ret->num_tx_origins = num_tx_origins;

    ret->buf_tx = calloc(ret->num_tx_origins, comms_cfuncs->size_of());
    memset(ret->buf_tx, 0.0, ret->num_tx_origins * comms_cfuncs->size_of());
    if(ret->buf_tx == NULL)
    {
        comms_destroy(ret);
        return NULL;
    }

    ret->checksum_tx1 = calloc(ret->num_tx_origins, sizeof(uint8_t));
    ret->checksum_tx2 = calloc(ret->num_tx_origins, sizeof(uint8_t));
    if(!ret->checksum_tx1 || !ret->checksum_tx2)
    {
        comms_destroy(ret);
        return NULL;
    }

    uint32_t i;
    for(i = 0; i < ret->num_tx_origins; ++i)
    {
        ret->buf_tx[i] = comms_cfuncs->create(buf_tx_len, sizeof(uint8_t));
        if(ret->buf_tx[i] == NULL)
        {
            comms_destroy(ret);
            return NULL;
        }
        fletcher_checksum_clear_tx(ret, i);
    }

    ret->curr_tx_orig_num = SELECTED_MSG_NONE;

    ret->publisher = publisher;

    ret->decode_state = 0;
    ret->decode_id = 0;
    ret->decode_channel = (comms_channel_t) 0;
    ret->decode_data_len = 0;
    fletcher_checksum_clear_rx(ret);

    for(i = 0; i < CHANNEL_NUM_CHANNELS; ++i)
    {
        ret->subscribers[i] = NULL;
        ret->num_subscribers[i] = 0;
    }

    return ret;
}

void comms_subscribe(comms_t *comms,
                     comms_channel_t channel,
                     subscriber_t subscriber,
                     void *usr)
{
    if(channel >= CHANNEL_NUM_CHANNELS) while(1);
    comms->num_subscribers[channel]++;
    comms->subscribers[channel] = (subscriber_container_t**) realloc(comms->subscribers[channel],
                                                                     comms->num_subscribers[channel] *
                                                                     sizeof(subscriber_container_t*));
    comms->subscribers[channel][comms->num_subscribers[channel] - 1] =
        (subscriber_container_t*) malloc(sizeof(subscriber_container_t));
    comms->subscribers[channel][comms->num_subscribers[channel] - 1]->subs = subscriber;
    comms->subscribers[channel][comms->num_subscribers[channel] - 1]->usr  = usr;
}

static comms_status_t publish_flush(comms_t *comms, uint32_t tx_origin_num)
{
    if(comms->curr_tx_orig_num == tx_origin_num)
    {
        if(!comms_cfuncs->is_empty(comms->buf_tx[tx_origin_num]))
            comms->publisher(comms->buf_tx[tx_origin_num]);

        if(comms_cfuncs->is_empty(comms->buf_tx[tx_origin_num]))
            return COMMS_STATUS_DONE;
        else
            return COMMS_STATUS_IN_PROGRESS;
    }
    else
    {
        return COMMS_STATUS_WAITING;
    }
}

static comms_status_t publish(comms_t *comms, uint32_t tx_origin_num,
                              bool publish_hw, uint8_t data)
{
    if(!comms_cfuncs->push_back(comms->buf_tx[tx_origin_num], &data))
        return COMMS_STATUS_BUFFER_FULL;

    if(comms_cfuncs->is_full(comms->buf_tx[tx_origin_num]) && publish_hw)
        return publish_flush(comms, tx_origin_num);

    return COMMS_STATUS_NO_ACTION;
}

inline comms_status_t comms_publish(comms_t *comms,
                                   comms_channel_t channel,
                                   const uint8_t *msg,
                                   uint16_t msg_len)
{
    return comms_publish_id(comms, 0, channel, msg, 0, msg_len);
}

comms_status_t comms_publish_id(comms_t *comms,
                               uint16_t id,
                               comms_channel_t channel,
                               const uint8_t *msg,
                               uint32_t tx_origin_num,
                               uint16_t msg_len)
{
#define NUM_METADATA 1+1+2+1+2+2

//uint32_t num_popped = 0; \
//while(!comms_cfuncs->is_empty(comms->buf_tx[tx_origin_num])) \
//{ \
    //comms_cfuncs->remove_back(comms->buf_tx[tx_origin_num]); \
    //if(++num_popped == S) \
        //break; \
//} \

#define PUBLISH_CHECK_FULL_BUFFER(C,N,D,P,S)\
    do { \
        if(publish(C,N,P,D) == COMMS_STATUS_BUFFER_FULL) \
        { \
            return COMMS_STATUS_BUFFER_FULL; \
        } \
        ++S; \
    } while(0)


    if(tx_origin_num >= comms->num_tx_origins) return COMMS_STATUS_INVALID_ARGUMENT;

    bool publish_hw = false;
    if(comms->curr_tx_orig_num == SELECTED_MSG_NONE)
    {
        comms->curr_tx_orig_num = tx_origin_num;
        publish_hw = true;
    }

    // XXX This functionality has not yet been built in
//    if(comms->config_flags & COMMS_CONFIG_ACCEPT_FULL_MESSAGES)
//    {
        // If we wont have room to buffer the whole message
        if(comms_cfuncs->capacity(comms->buf_tx[tx_origin_num]) -
                comms_cfuncs->size(comms->buf_tx[tx_origin_num]) < msg_len + NUM_METADATA)
            return COMMS_STATUS_BUFFER_FULL;
//    }

    uint32_t num_so_far = 0;

    fletcher_checksum_clear_tx(comms, tx_origin_num);

    fletcher_checksum_add_byte_tx(comms, tx_origin_num, START_BYTE1);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, START_BYTE1, publish_hw, num_so_far);

    fletcher_checksum_add_byte_tx(comms, tx_origin_num, START_BYTE2);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, START_BYTE2, publish_hw, num_so_far);

    uint8_t id1 = (id & 0xff00) >> 8;
    fletcher_checksum_add_byte_tx(comms, tx_origin_num, id1);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, id1, publish_hw, num_so_far);

    uint8_t id2 = id & 0x00ff;
    fletcher_checksum_add_byte_tx(comms, tx_origin_num, id2);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, id2, publish_hw, num_so_far);

    fletcher_checksum_add_byte_tx(comms, tx_origin_num, channel);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, channel, publish_hw, num_so_far);

    uint8_t len1 = (msg_len & 0xff00) >> 8;
    fletcher_checksum_add_byte_tx(comms, tx_origin_num, len1);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, len1, publish_hw, num_so_far);

    uint8_t len2 = msg_len & 0x00ff;
    fletcher_checksum_add_byte_tx(comms, tx_origin_num, len2);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, len2, publish_hw, num_so_far);

    uint8_t i;
    for(i = 0; i< msg_len; ++i)
    {
        fletcher_checksum_add_byte_tx(comms, tx_origin_num, msg[i]);
        PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num, msg[i], publish_hw, num_so_far);
    }

    fletcher_checksum_calculate_tx(comms, tx_origin_num);

    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num,
                              comms->checksum_tx1[tx_origin_num], publish_hw, num_so_far);
    PUBLISH_CHECK_FULL_BUFFER(comms, tx_origin_num,
                              comms->checksum_tx2[tx_origin_num], publish_hw, num_so_far);

    comms_status_t ret = COMMS_STATUS_WAITING;
    if(publish_hw)
    {
        ret = publish_flush(comms, tx_origin_num);
        comms->curr_tx_orig_num = SELECTED_MSG_NONE;
    }

    return ret;

#undef NUM_METADATA
}

comms_status_t comms_transmit(comms_t *comms)
{
    comms_status_t ret;

    if(comms->num_tx_origins == 0) return COMMS_STATUS_INVALID_ARGUMENT;

    if(comms->curr_tx_orig_num == SELECTED_MSG_NONE)
        comms->curr_tx_orig_num = 0;

    // XXX: safety measure for debug
    if (comms->curr_tx_orig_num <= SELECTED_MSG_NONE || comms->curr_tx_orig_num >= comms->num_tx_origins)
        while (1);

    uint32_t start_tx_origin_num = comms->curr_tx_orig_num;

    while((ret = publish_flush(comms, comms->curr_tx_orig_num)) == COMMS_STATUS_DONE)
    {
        if(++comms->curr_tx_orig_num == comms->num_tx_origins)
            comms->curr_tx_orig_num = 0;

        // XXX: safety measure for debug
        if (comms->curr_tx_orig_num <= SELECTED_MSG_NONE || comms->curr_tx_orig_num >= comms->num_tx_origins)
            while (1);

        if(comms->curr_tx_orig_num == start_tx_origin_num)
            break;
    }
    if(ret == COMMS_STATUS_DONE)
        comms->curr_tx_orig_num = SELECTED_MSG_NONE;
    return ret;
}

void comms_handle(comms_t *comms, uint8_t byte)
{
#define STATE_START1    0
#define STATE_START2    1
#define STATE_ID1       2
#define STATE_ID2       3
#define STATE_CHANNEL   4
#define STATE_DATALEN1  5
#define STATE_DATALEN2  6
#define STATE_DATA      7
#define STATE_CHECKSUM1 8
#define STATE_CHECKSUM2 9

try_again:

    switch(comms->decode_state)
    {
        case STATE_START1:
            if(byte == START_BYTE1)
            {
                comms->decode_state = STATE_START2;
                fletcher_checksum_add_byte_rx(comms, byte);
            }
            break;

        case STATE_START2:
            if(byte == START_BYTE2)
            {
                comms->decode_state = STATE_ID1;
                fletcher_checksum_add_byte_rx(comms, byte);
            }
            else
            {
                comms->decode_state = STATE_START1;
                fletcher_checksum_clear_rx(comms);
                goto try_again;
            }
            break;

        case STATE_ID1:
            comms->decode_id = byte;
            comms->decode_state = STATE_ID2;
            fletcher_checksum_add_byte_rx(comms, byte);
            break;

        case STATE_ID2:
            comms->decode_id = (comms->decode_id << 8) | byte;
            comms->decode_state = STATE_CHANNEL;
            fletcher_checksum_add_byte_rx(comms, byte);
            break;

        case STATE_CHANNEL:
            comms->decode_channel = (comms_channel_t) byte;
            if(comms->decode_channel < CHANNEL_NUM_CHANNELS)
            {
                comms->decode_state = STATE_DATALEN1;
                fletcher_checksum_add_byte_rx(comms, byte);
            }
            else
            {
                comms->decode_state = STATE_START1;
                fletcher_checksum_clear_rx(comms);
                goto try_again;
            }
            break;

        case STATE_DATALEN1:
            comms->decode_data_len = byte;
            comms->decode_state = STATE_DATALEN2;
            fletcher_checksum_add_byte_rx(comms, byte);
            break;

        case STATE_DATALEN2:
            comms->decode_data_len = (comms->decode_data_len << 8) | byte;
            if(comms->decode_data_len < comms->buf_rx_len)
            {
                if(comms->decode_data_len == 0)
                    comms->decode_state = STATE_CHECKSUM1;
                else
                {
                    comms->decode_state = STATE_DATA;
                    comms->decode_num_data_read = 0;
                }
                fletcher_checksum_add_byte_rx(comms, byte);
            }
            else
            {
                comms->decode_state = STATE_START1;
                fletcher_checksum_clear_rx(comms);
                goto try_again;
            }
            break;

        case STATE_DATA:
            comms->buf_rx[comms->decode_num_data_read++] = byte;
            fletcher_checksum_add_byte_rx(comms, byte);
            if(comms->decode_num_data_read == comms->decode_data_len)
            {
                comms->decode_state = STATE_CHECKSUM1;
            }
            break;

        case STATE_CHECKSUM1:
            comms->decode_checksum = byte;
            comms->decode_state = STATE_CHECKSUM2;
            break;

        case STATE_CHECKSUM2:
            comms->decode_checksum = fletcher_checksum_calculate(comms->decode_checksum, byte);
            uint16_t calc_checksum = fletcher_checksum_calculate_rx(comms);
            comms->decode_state = STATE_START1;
            fletcher_checksum_clear_rx(comms);
            if(comms->decode_checksum == calc_checksum)
            {
                uint8_t i;
                for(i = 0; i < comms->num_subscribers[comms->decode_channel]; ++i)
                {
                    subscriber_t sub = comms->subscribers[comms->decode_channel][i]->subs;
                    void *usr = comms->subscribers[comms->decode_channel][i]->usr;
                    sub(usr, comms->decode_id, comms->decode_channel, comms->buf_rx, comms->decode_data_len);
                }
                for(i = 0; i < comms->num_subscribers[CHANNEL_ALL]; ++i)
                {
                    subscriber_t sub = comms->subscribers[CHANNEL_ALL][i]->subs;
                    void *usr = comms->subscribers[CHANNEL_ALL][i]->usr;
                    sub(usr, comms->decode_id, comms->decode_channel, comms->buf_rx, comms->decode_data_len);
                }
            }
            else
                goto try_again;
            break;
    }

#undef STATE_START1
#undef STATE_START2
#undef STATE_ID1
#undef STATE_ID2
#undef STATE_CHANNEL
#undef STATE_DATALEN1
#undef STATE_DATALEN2
#undef STATE_DATA
#undef STATE_CHECKSUM1
#undef STATE_CHECKSUM2
}

void comms_destroy(comms_t *comms)
{
    uint8_t i, j;

    for(i = 0; i < CHANNEL_NUM_CHANNELS; ++i)
    {
        for(j = 0; j < comms->num_subscribers[i]; ++j)
            free(comms->subscribers[i][j]);
        if(comms->subscribers[i] != NULL)
            free(comms->subscribers[i]);
    }

    if(comms->buf_tx)
    {
        for(i = 0; i < comms->num_tx_origins; ++i)
            if(comms->buf_tx[i])
                comms_cfuncs->destroy(comms->buf_tx[i]);
        free(comms->buf_tx);
    }

    if(comms->checksum_tx1)
        free(comms->checksum_tx1);

    if(comms->checksum_tx2)
        free(comms->checksum_tx2);

    if(comms->buf_rx)
        free(comms->buf_rx);

    free(comms);
}




static void fletcher_checksum_clear_rx(comms_t *comms)
{
    comms->checksum_rx1 = comms->checksum_rx2 = 0;
}

static void fletcher_checksum_add_byte_rx(comms_t *comms, uint8_t byte)
{
    // fletcher checksum calc
    comms->checksum_rx1 += byte;
    comms->checksum_rx2 += comms->checksum_rx1;
}

static uint16_t fletcher_checksum_calculate_rx(comms_t *comms)
{
    return fletcher_checksum_calculate(comms->checksum_rx1, comms->checksum_rx2);
}


static void fletcher_checksum_clear_tx(comms_t *comms, uint32_t tx_origin_num)
{
    comms->checksum_tx1[tx_origin_num] = comms->checksum_tx2[tx_origin_num] = 0;
}

static void fletcher_checksum_add_byte_tx(comms_t *comms, uint32_t tx_origin_num, uint8_t byte)
{
    // fletcher checksum calc
    comms->checksum_tx1[tx_origin_num] += byte;
    comms->checksum_tx2[tx_origin_num] += comms->checksum_tx1[tx_origin_num];
}

static uint16_t fletcher_checksum_calculate_tx(comms_t *comms, uint32_t tx_origin_num)
{
    return fletcher_checksum_calculate(comms->checksum_tx1[tx_origin_num],
                                       comms->checksum_tx2[tx_origin_num]);
}


static inline uint16_t fletcher_checksum_calculate(uint8_t checksum1, uint8_t checksum2)
{
    return ((uint16_t)checksum1 << 8) + (uint16_t)checksum2;
}
