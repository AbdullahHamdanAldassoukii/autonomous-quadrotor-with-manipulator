# This is a general use makefile for robotics cape projects written in C.
# Just change the target name to match your main source code filename.
TARGET = ../bin/project


TOUCH 	 := $(shell touch *)
CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g
LFLAGS	:= -lm -lrt -lpthread -L ../lib -lrobocape

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.c=$%.o)

RM := rm -f

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c
	@$(TOUCH) $(CC) $(CFLAGS) -c $< -o $(@)


all:
	$(TARGET)
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Clean Complete"
