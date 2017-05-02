BINARIES = ../../bin/comms_driver ../../bin/comms_test
LCMTYPES := $(shell find ../lcmtypes/ -name '*.c')
LCMTYPES := $(LCMTYPES:$%.c=$%.o)
CFLAGS = -g -Wall -std=gnu99 `pkg-config --cflags lcm`
LDFLAGS = `pkg-config --libs lcm`
CC = gcc

.PHONY: all clean io lcmtypes

all: io $(BINARIES)

clean:
	rm -f *~ *.o io/*.o
	rm -f $(BINARIES)

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

io/%.o: io/%.c
	$(CC) $(CFLAGS) -c $^ -I. -o $@

../../bin/comms_driver: comms_driver.o io/comms.o io/serial.o io/circular.o $(LCMTYPES) 
	$(CC) -o $@ $^ -Iio $(LDFLAGS)

../../bin/comms_test: comms_test.o io/util.o
	$(CC) -o $@ $^ $(LDFLAGS) $(LCMTYPES) -lm


