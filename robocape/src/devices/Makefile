#
# Generic subdirectory Makefile
#

SOURCES := $(shell ls *.c)
INCLUDES := $(shell ls *.h 2> /dev/null)
OBJECTS := $(SOURCES:$%.c=$%.o)
all: $(OBJECTS)

%.o: %.c $(INCLUDES)
	@$(CC) -c $(CFLAGS) -o $@ $<
	@echo "Compiled: " $<

clean:
	@$(RM) *.o
