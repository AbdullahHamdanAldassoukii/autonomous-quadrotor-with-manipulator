#ifndef LINUX_SERIAL_H
#define LINUX_SERIAL_H

#include <stdlib.h>
#include <termios.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct serial serial_t;

// open a serial device using 8 bits, 1 stop, and no parity
// the "baud" parameter uses the parameters specified in termios.h
// example: for a baud rate of 38400 use "B38400"
serial_t *serial_create(const char *port, int baud);

// works analogous to unix-style write()
int serial_write(serial_t *this, const char *buf, size_t sz);

// works analogous to unix-style read()
// this call blocks until data is received
int serial_read(serial_t *this, char *buf, size_t bufsz);

void serial_destroy(serial_t *this);

#ifdef __cplusplus
}
#endif

#endif  /* LINUX_SERIAL_H */
