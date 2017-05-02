#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

#include "serial.h"

#define DEBUG 0

struct serial
{
    int fd;
};

serial_t *serial_create(const char *port, int baud)
{
    serial_t *this = calloc(1, sizeof(serial_t));

    int flags = O_RDWR | O_NOCTTY | O_SYNC;
    this->fd = open(port, flags, 0);
    if(this->fd < 0) {
        if(DEBUG) fprintf(stderr, "Failed to open serial device: %s\n", strerror(errno));
        goto fail;
    }

    // attempt to reset the USB device
    // this call may fail: but we don't care
    // occasionally the USB bus can get in a weird state
    // this call will reset the USB in that situation
    ioctl(this->fd, USBDEVFS_RESET, 0);

    struct termios opts;

    // get the termios config
    if(tcgetattr(this->fd, &opts)) {
        fprintf(stderr, "failed to get termios options on fd (%i): %s\n", this->fd, strerror(errno));
        goto fail;
    }

    cfsetispeed(&opts, baud);
    cfsetospeed(&opts, baud);
    cfmakeraw(&opts);

    opts.c_cflag &= ~CSTOPB;
    opts.c_cflag |= CS8;
    opts.c_cflag &= ~PARENB;

    // set the new termios config
    if(tcsetattr(this->fd, TCSANOW, &opts)) {
        fprintf(stderr, "failed to set termios options on fd (%i): %s\n", this->fd, strerror(errno));
        goto fail;
    }

    tcflush(this->fd, TCIOFLUSH);

    return this;

 fail:
    free(this);
    return NULL;
}

// works analogous to unix-style write()
int serial_write(serial_t *this, const char *buf, size_t sz)
{
    assert(this != NULL);
    int ret = write(this->fd, buf, sz);
    if(ret == -1) {
        fprintf(stderr, "ERR: write failed: %s\n", strerror(errno));
        fflush(stderr);
        return -1;
    }
    fsync(this->fd);
    return ret;
}

// works analogous to unix-style read()
int serial_read(serial_t *this, char *buf, size_t bufsz)
{
    assert(this != NULL);
    int ret = read(this->fd, buf, bufsz);
    if(ret == -1) {
        fprintf(stderr, "ERR: write failed: %s\n", strerror(errno));
        fflush(stderr);
    }
    return ret;
}


void serial_destroy(serial_t *this)
{
    assert(this != NULL);
    close(this->fd);
}

