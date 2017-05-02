LCMTYPES_NAMES := $(shell ls ../lcmtypes/*.lcm)

# Construct the pathnames to the C types
LCMTYPES_C := $(LCMTYPES_NAMES:%.lcm=%.c)
LCMTYPES_O = $(LCMTYPES_C:%.c=%.o)
CC = gcc
CFLAGS = -g `pkg-config --cflags lcm`

ALL = $(LCMTYPES_C) $(LCMTYPES_O)

all: $(ALL)

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

clean:
	rm -f *.c *.h *.o *~
