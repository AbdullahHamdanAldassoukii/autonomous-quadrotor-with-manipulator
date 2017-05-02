#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>   // SIGINT and SIGTERM defines for signal interrupts
#include <errno.h>    // Error type definitions such as EINVAL or EBUSY
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>

#include <lcm/lcm.h>

#include "io/comms.h"
#include "io/serial.h"

#include "../lcmtypes/channels_t.h"
#include "../lcmtypes/imu_data_t.h"
#include "../lcmtypes/kill_t.h"
#include "../lcmtypes/led_t.h"
#include "../lcmtypes/rpms_t.h"
#include "../lcmtypes/telemetry_t.h"
#include "../lcmtypes/cfg_data_frequency_t.h"
#include "../lcmtypes/cfg_usb_serial_num_t.h"

char *USAGE = "Usage: ./comms_driver <-x xbee_dev_path> <-u usb_dev_path>";
bool loopback_mode = false;
static bool verbose = false;

#define verbose_printf(...) \
    do{\
        if(verbose) printf(__VA_ARGS__);\
    }while(0)\

static uint64_t timestamp_now(void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    //XXX Not sure if this will work across hour boundaries
    return (uint64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

static void* xbee_run(void*);
static void* usb_run(void*);
static void publish_xbee(container_t *data);
static void publish_usb (container_t *data);
static void handler_channels(void *usr, uint16_t id, comms_channel_t channel,
                             const uint8_t *msg, uint16_t len);
static void handler_channels_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                                 const channels_t *msg, void *user);
static void handler_kill(void *usr, uint16_t id, comms_channel_t channel,
                         const uint8_t *msg, uint16_t len);
static void handler_kill_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                             const kill_t *msg, void *user);
static void handler_led(void *usr, uint16_t id, comms_channel_t channel,
                         const uint8_t *msg, uint16_t len);
static void handler_led_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                             const led_t *msg, void *user);
static void handler_cfg_usb_serial_num(void *usr, uint16_t id, comms_channel_t channel,
                                       const uint8_t *msg, uint16_t len);
static void handler_cfg_usb_serial_num_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                                           const cfg_usb_serial_num_t *msg, void *user);
static void handler_cfg_data_frequency(void *usr, uint16_t id, comms_channel_t channel,
                                       const uint8_t *msg, uint16_t len);
static void handler_cfg_data_frequency_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                                           const cfg_data_frequency_t *msg, void *user);

pthread_t xbee_thread;
comms_t *xbee_comms;
serial_t *xbee = NULL;

pthread_t usb_thread;
comms_t *usb_comms;
serial_t *usb = NULL;

lcm_t *lcm;

volatile bool done = false;
void interrupt(int sig)
{
    static int8_t num_exit_attempts = 0;
    num_exit_attempts++;
    if(num_exit_attempts == 1)
        fprintf(stderr, "Caught ctrl+c and signalling exit to driver\n");
    if(num_exit_attempts == 3)
    {
        fprintf(stderr, "Force quitting\n");
        exit(1);
    }

    done = true;
}


int main(int argc, char *argv[])
{
    uint8_t i;
    for(i = 0; i < argc; ++i)
    {
        if((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0))
        {
            fprintf(stdout, "%s\n", USAGE);
            exit(0);
        }
        if(strcmp(argv[i], "-v") == 0)
        {
            verbose = true;
            fprintf(stdout, "Running in verbose mode\n");
        }

        if(strcmp(argv[i], "-l") == 0)
        {
            loopback_mode = true;
            fprintf(stdout, "Running in loopback mode\n");
        }
    }

    (void) signal(SIGINT, interrupt);
    (void) signal(SIGTERM, interrupt);

    lcm = lcm_create(NULL);
    while(lcm == NULL)
    {
        fprintf(stderr, "LCM failed to initialize. Reattempting....\n");
        usleep(1000000);
        lcm = lcm_create(NULL);
    }


    // Create Serial Device for XBee
    char *xbee_dev_name = "/dev/XBee";
    for(i = 0; i < argc; ++i)
    {
        if(strcmp(argv[i], "-x") == 0)
        {
            if(i + 1 < argc)
                xbee_dev_name = argv[i+1];
            break;
        }
    }
    xbee = serial_create(xbee_dev_name, B9600);
    if(xbee == NULL)
        fprintf(stderr, "XBee device does not exist at %s\n", xbee_dev_name);
    else
        fprintf(stdout, "XBee device successfully opened at 9600 baud on %s\n", xbee_dev_name);

    // Create Serial Device for USB
    char *usb_dev_name = "/dev/stack";
    for(i = 0; i < argc; ++i)
    {
        if(strcmp(argv[i], "-u") == 0)
        {
            if(i + 1 < argc)
                usb_dev_name = argv[i+1];
            break;
        }
    }
    usb = serial_create(usb_dev_name, B115200);
    if(usb == NULL)
        fprintf(stderr, "Usb device does not exist at %s\n", usb_dev_name);
    else
        fprintf(stdout, "Usb device successfully opened at 115200 baud on %s\n", usb_dev_name);


    // If no open comm ports and not in loopback mode, quit.
    if(!xbee && !usb && !loopback_mode)
    {
        fprintf(stderr, "Failed to open any comms devices. Exiting...\n");
        lcm_destroy(lcm);
        exit(1);
    }


    // Create comms devices
    if(xbee)
    {
        xbee_comms = comms_create(1000, 1000, 1, publish_xbee);
        comms_subscribe(xbee_comms, CHANNEL_KILL, handler_kill, NULL);
        comms_subscribe(xbee_comms, CHANNEL_LED, handler_led, NULL);
        comms_subscribe(xbee_comms, CHANNEL_CHANNELS, handler_channels, NULL);
        comms_subscribe(xbee_comms, CHANNEL_CFG_USB_SN, handler_cfg_usb_serial_num, NULL);
        comms_subscribe(xbee_comms, CHANNEL_CFG_DATA_FREQUENCY, handler_cfg_data_frequency, NULL);
        pthread_create(&xbee_thread, NULL, xbee_run, NULL);
    }

    // Create comms devices
    if(usb)
    {
        usb_comms = comms_create(1000, 1000, 1, publish_usb);
        comms_subscribe(usb_comms, CHANNEL_KILL, handler_kill, NULL);
        comms_subscribe(usb_comms, CHANNEL_LED, handler_led, NULL);
        comms_subscribe(usb_comms, CHANNEL_CHANNELS, handler_channels, NULL);
        comms_subscribe(usb_comms, CHANNEL_CFG_USB_SN, handler_cfg_usb_serial_num, NULL);
        comms_subscribe(usb_comms, CHANNEL_CFG_DATA_FREQUENCY, handler_cfg_data_frequency, NULL);
        pthread_create(&usb_thread, NULL, usb_run, NULL);
    }

    kill_t_subscription_t  *kill_subs =
        kill_t_subscribe(lcm, "KILL_.*_TX", handler_kill_lcm, NULL);
    led_t_subscription_t  *led_subs =
        led_t_subscribe(lcm, "LED_.*_TX", handler_led_lcm, NULL);
    channels_t_subscription_t *channels_subs =
        channels_t_subscribe(lcm, "CHANNELS_.*_TX", handler_channels_lcm, NULL);
    cfg_data_frequency_t_subscription_t *data_freq_subs =
        cfg_data_frequency_t_subscribe(lcm, "CFG_DATA_FREQUENCY_.*_TX",
                                       handler_cfg_data_frequency_lcm, NULL);
    cfg_usb_serial_num_t_subscription_t *usb_serial_num_subs =
        cfg_usb_serial_num_t_subscribe(lcm, "CFG_USB_SERIAL_NUM_.*_TX",
                                       handler_cfg_usb_serial_num_lcm, NULL);

    while(!done)
    {
        // Set up timeout on lcm_handle to prevent blocking at program exit
        int lcm_fd = lcm_get_fileno(lcm);
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);

        struct timeval timeout = { 0, 100000 }; // wait 100ms
        int status = select(lcm_fd + 1, &fds, NULL, NULL, &timeout);

        if(done) break;

        if(status != 0 && FD_ISSET(lcm_fd, &fds)) {
            if ( lcm_handle(lcm) ) {
                fprintf( stderr, "LCM handle error, aborting\n" );
                done = true;
            }
        }
    }
    fprintf(stdout, "Lcm driver destroyed.\n");

    kill_t_unsubscribe(lcm, kill_subs);
    led_t_unsubscribe(lcm, led_subs);
    channels_t_unsubscribe(lcm, channels_subs);
    cfg_data_frequency_t_unsubscribe(lcm, data_freq_subs);
    cfg_usb_serial_num_t_unsubscribe(lcm, usb_serial_num_subs);

    if(xbee)
    {
        pthread_cancel(xbee_thread);
        comms_destroy(xbee_comms);
        serial_destroy(xbee);
        fprintf(stdout, "Xbee driver destroyed.\n");
    }

    if(usb)
    {
        pthread_cancel(usb_thread);
        comms_destroy(usb_comms);
        serial_destroy(usb);
        fprintf(stdout, "Usb driver destroyed.\n");
    }

    lcm_destroy(lcm);

    return 0;
}



uint8_t parse_id(const char *channel_name)
{
    static char *suffix = "_TX";

    uint8_t suffix_len = strlen(suffix);
    uint8_t len = strlen(channel_name);

    if(suffix_len > len) return 0;

    // If string ends with _TX
    if(strncmp(channel_name + len - suffix_len, suffix, suffix_len) == 0)
    {
        // search backwards for id where channel name could be in form of "..._id_TX"
        uint8_t i;
        uint8_t start_idx = len - 1 - suffix_len;
        for(i = start_idx; i >= 0; --i)
        {
            if(channel_name[i] == '_')
            {
                char *id_str = malloc(sizeof(char) * (start_idx - i + 1));
                id_str[start_idx - i] = '\0';
                strncpy(id_str, &channel_name[i+1], start_idx - i);
                uint8_t id = atoi(id_str);
                free(id_str);
                return id;
                break;
            }
        }
    }

    return 0;
}


static void* xbee_run(void* arg)
{
    fprintf(stdout, "Starting xbee receive thread\n");
    char data[1];
    while(!done)
    {
        serial_read(xbee, data, 1);
        comms_handle(xbee_comms, data[0]);
    }
    return NULL;
}

static void publish_xbee(container_t *data)
{
    verbose_printf("TX UART: ");
    uint16_t i, data_len = comms_cfuncs->size(data);
    for(i = 0; i < data_len; ++i)
        verbose_printf("%x ", *(const uint8_t*)comms_cfuncs->at(data,i));
    verbose_printf("\n");

    while(comms_cfuncs->size(data) > 0)
    {
        serial_write(xbee, (const uint8_t*)comms_cfuncs->front(data), 1);
        comms_cfuncs->remove_front(data);
    }
}

static void* usb_run(void* arg)
{
    fprintf(stdout, "Starting usb receive thread\n");
    char data[1];
    while(!done)
    {
        serial_read(usb, data, 1);
        comms_handle(usb_comms, data[0]);
    }
    return NULL;
}

static void publish_usb(container_t *data)
{
    verbose_printf("TX USB: ");
    uint16_t i, data_len = comms_cfuncs->size(data);
    for(i = 0; i < data_len; ++i)
        verbose_printf("%x ", *(const uint8_t*)comms_cfuncs->at(data,i));
    verbose_printf("\n");

    while(comms_cfuncs->size(data) > 0)
    {
        serial_write(usb, (const uint8_t*)comms_cfuncs->front(data), 1);
        comms_cfuncs->remove_front(data);
    }
}

static void handler_kill(void *usr, uint16_t id, comms_channel_t channel,
                         const uint8_t *msg, uint16_t len)
{
    char lcm_channel[20];
    snprintf(lcm_channel, 20, "KILL_%d_RX", id);

    verbose_printf("%s: ", lcm_channel);
    uint16_t i;
    for(i = 0; i < len; ++i)
    {
        verbose_printf("%x ", msg[i]);
    }
    verbose_printf("\n");

    kill_t kill;
    memset(&kill, 0, sizeof(kill_t));
    __kill_t_decode_array(msg, 0, len, &kill, 1);
    kill_t_publish(lcm, lcm_channel, &kill);
    kill_t_decode_cleanup(&kill);
}

static void handler_kill_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                             const kill_t *msg, void *user)
{
    uint8_t id = parse_id(channel);

    verbose_printf("Received msg on lcm channel %s with id %d\n", channel, id);

    uint32_t maxlen = __kill_t_encoded_array_size(msg, 1);
    uint8_t *buf = (uint8_t *) malloc(sizeof(uint8_t) * maxlen);
    uint32_t len = __kill_t_encode_array(buf, 0, maxlen, msg, 1);
    if(usb)  comms_publish_id( usb_comms, id, CHANNEL_KILL, buf, 0, len);
    if(xbee) comms_publish_id(xbee_comms, id, CHANNEL_KILL, buf, 0, len);
    if(loopback_mode) handler_kill(NULL, id, CHANNEL_KILL, buf, len);
    free(buf);
}

static void handler_led(void *usr, uint16_t id, comms_channel_t channel,
                         const uint8_t *msg, uint16_t len)
{
    char lcm_channel[20];
    snprintf(lcm_channel, 20, "LED_%d_RX", id);

    verbose_printf("%s: ", lcm_channel);
    uint16_t i;
    for(i = 0; i < len; ++i)
    {
        verbose_printf("%x ", msg[i]);
    }
    verbose_printf("\n");

    led_t led;
    memset(&led, 0, sizeof(led_t));
    __led_t_decode_array(msg, 0, len, &led, 1);
    led_t_publish(lcm, lcm_channel, &led);
    led_t_decode_cleanup(&led);
}

static void handler_led_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                             const led_t *msg, void *user)
{
    uint8_t id = parse_id(channel);

    verbose_printf("Received msg on lcm channel %s with id %d\n", channel, id);

    uint32_t maxlen = __led_t_encoded_array_size(msg, 1);
    uint8_t *buf = (uint8_t *) malloc(sizeof(uint8_t) * maxlen);
    uint32_t len = __led_t_encode_array(buf, 0, maxlen, msg, 1);
    if(usb)  comms_publish_id( usb_comms, id, CHANNEL_LED, buf, 0, len);
    if(xbee) comms_publish_id(xbee_comms, id, CHANNEL_LED, buf, 0, len);
    if(loopback_mode) handler_led(NULL, id, CHANNEL_LED, buf, len);
    free(buf);
}

static void handler_channels(void *usr, uint16_t id, comms_channel_t channel,
                             const uint8_t *msg, uint16_t len)
{
    char lcm_channel[40];
    snprintf(lcm_channel, 40, "CHANNELS_%d_RX", id);

    verbose_printf("%s: ", lcm_channel);
    uint16_t i;
    for(i = 0; i < len; ++i)
    {
        verbose_printf("%x ", msg[i]);
    }
    verbose_printf("\n");

    channels_t channels;
    memset(&channels, 0, sizeof(channels_t));
    __channels_t_decode_array(msg, 0, len, &channels, 1);
    channels_t_publish(lcm, lcm_channel, &channels);
    __channels_t_decode_array_cleanup(&channels, 1);
}

static void handler_channels_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                                 const channels_t *msg, void *user)
{
    uint8_t id = parse_id(channel);

    verbose_printf("Received msg on lcm channel %s with id %d\n", channel, id);

    uint32_t maxlen = __channels_t_encoded_array_size(msg, 1);
    uint8_t *buf = (uint8_t *) malloc(sizeof(uint8_t) * maxlen);
    uint32_t len = __channels_t_encode_array(buf, 0, maxlen, msg, 1);
    if(usb)  comms_publish_id( usb_comms, id, CHANNEL_CHANNELS, buf, 0, len);
    if(xbee) comms_publish_id(xbee_comms, id, CHANNEL_CHANNELS, buf, 0, len);
    if(loopback_mode) handler_channels(NULL, id, CHANNEL_CHANNELS, buf, len);
    free(buf);
}

static void handler_cfg_usb_serial_num(void *usr, uint16_t id, comms_channel_t channel,
                                       const uint8_t *msg, uint16_t len)
{
    char lcm_channel[40];
    snprintf(lcm_channel, 40, "CFG_USB_SERIAL_NUM_%d_RX", id);

    verbose_printf("%s: ", lcm_channel);
    uint16_t i;
    for(i = 0; i < len; ++i)
    {
        verbose_printf("%x ", msg[i]);
    }
    verbose_printf("\n");

    cfg_usb_serial_num_t cfg_usb_serial_num;
    memset(&cfg_usb_serial_num, 0, sizeof(cfg_usb_serial_num_t));
    __cfg_usb_serial_num_t_decode_array(msg, 0, len, &cfg_usb_serial_num, 1);
    cfg_usb_serial_num_t_publish(lcm, lcm_channel, &cfg_usb_serial_num);
    __cfg_usb_serial_num_t_decode_array_cleanup(&cfg_usb_serial_num, 1);
}

static void handler_cfg_usb_serial_num_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                                           const cfg_usb_serial_num_t *msg, void *user)
{
    uint8_t id = parse_id(channel);

    verbose_printf("Received msg on lcm channel %s with id %d\n", channel, id);

    uint32_t maxlen = __cfg_usb_serial_num_t_encoded_array_size(msg, 1);
    uint8_t *buf = (uint8_t *) malloc(sizeof(uint8_t) * maxlen);
    uint32_t len = __cfg_usb_serial_num_t_encode_array(buf, 0, maxlen, msg, 1);
    if(usb)  comms_publish_id( usb_comms, id, CHANNEL_CFG_USB_SN, buf, 0, len);
    if(xbee) comms_publish_id(xbee_comms, id, CHANNEL_CFG_USB_SN, buf, 0, len);
    if(loopback_mode) handler_cfg_usb_serial_num(NULL, id, CHANNEL_CFG_USB_SN, buf, len);
    free(buf);
}

static void handler_cfg_data_frequency(void *usr, uint16_t id, comms_channel_t channel,
                                       const uint8_t *msg, uint16_t len)
{
    char lcm_channel[40];
    snprintf(lcm_channel, 40, "CFG_DATA_FREQUENCY_%d_RX", id);

    verbose_printf("%s: ", lcm_channel);
    uint16_t i;
    for(i = 0; i < len; ++i)
    {
        verbose_printf("%x ", msg[i]);
    }
    verbose_printf("\n");

    cfg_data_frequency_t cfg_data_frequency;
    memset(&cfg_data_frequency, 0, sizeof(cfg_data_frequency_t));
    __cfg_data_frequency_t_decode_array(msg, 0, len, &cfg_data_frequency, 1);
    cfg_data_frequency_t_publish(lcm, lcm_channel, &cfg_data_frequency);
    __cfg_data_frequency_t_decode_array_cleanup(&cfg_data_frequency, 1);
}

static void handler_cfg_data_frequency_lcm(const lcm_recv_buf_t *rbuf, const char *channel,
                                           const cfg_data_frequency_t *msg, void *user)
{
    uint8_t id = parse_id(channel);

    verbose_printf("Received msg on lcm channel %s with id %d\n", channel, id);

    uint32_t maxlen = __cfg_data_frequency_t_encoded_array_size(msg, 1);
    uint8_t *buf = (uint8_t *) malloc(sizeof(uint8_t) * maxlen);
    uint32_t len = __cfg_data_frequency_t_encode_array(buf, 0, maxlen, msg, 1);
    if(usb)  comms_publish_id( usb_comms, id, CHANNEL_CFG_DATA_FREQUENCY, buf, 0, len);
    if(xbee) comms_publish_id(xbee_comms, id, CHANNEL_CFG_DATA_FREQUENCY, buf, 0, len);
    if(loopback_mode) handler_cfg_data_frequency(NULL, id, CHANNEL_CFG_DATA_FREQUENCY, buf, len);
    free(buf);
}
