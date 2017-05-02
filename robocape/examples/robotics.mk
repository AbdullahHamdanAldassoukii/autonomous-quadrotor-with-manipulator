TOUCH 	 := $(shell touch *)
CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g
LFLAGS	:= -L ../../lib -lrobocape -lm -lrt -lpthread

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
OBJECTS  := $(SOURCES:$%.c=$%.o)

PREFIX := /usr
RM := rm -f

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LFLAGS)

# compiling command
$(OBJECTS): %.o : %.c
	@$(CC) $(CFLAGS) -c $< -o $(@)
	@echo "Making Example: " $<

all: $(TARGET)
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
