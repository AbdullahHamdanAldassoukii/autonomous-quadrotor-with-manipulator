# Define and export variables used in all Make shells
LIBPATH = ../lib
TARGET = $(abspath $(LIBPATH))/librobocape.so
CC = gcc
LINKER   := gcc
TOUCH 	 := $(shell touch *)
CFLAGS := -Wall -fsingle-precision-constant -fpic -march=armv7-a -mtune=cortex-a8 `pkg-config --cflags lcm`
RM := rm -f
export LIBPATH TARGET CC LINKER TOUCH CFLAGS RM

# Now define variables used by only this Make file
LFLAGS	:= -lm -lrt -lpthread -shared -Wl,-soname,$(TARGET)
LDFLAGS := `pkg-config --libs lcm`
#SOURCES := $(shell ls *.c 2> /dev/null)
SOURCES	 := $(shell find ./ -name '*.c')
INCLUDES := $(shell ls *.h 2> /dev/null)
OBJECTS  := $(SOURCES:$%.c=$%.o)

# Make .so library $(TARGET)$
all: 
	@make -C devices
	@make -C math
	@make -C mmap
	@make -C other
	@make -C serial
	@make -C simple
	@$(CC) -c $(CFLAGS) -o robocape.o robocape.c
	@$(LINKER) $(LFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS)
	@echo $(TARGET) "\nLibrary Build Complete"
	@echo "\nWorking on Examples..."

# Default compile rule for this directory
#%.o: %.c $(INCLUDES)
#	@echo $@
#	@echo $<
#	@$(CC) -c $(CFLAGS) -o $@ $<
#
clean:
	@make -C devices -s clean
	@make -C math -s clean
	@make -C mmap -s clean
	@make -C other -s clean
	@make -C serial -s clean
	@make -C simple -s clean
	@$(RM) *.o
	@$(RM) $(TARGET)$
	@echo "$(TARGET) Clean Complete"


