TARGET	:=../bin/imu_lcm_example
#TOUCH 	 := $(shell touch *)

CC	:= gcc
LINKER   := gcc -o
CFLAGS	:= -c -Wall -g `pkg-config --cflags lcm`
LFLAGS	:= -L../robocape/lib -lrobocape -lm -lrt -lpthread -Wl,-soname,../robocape/lib/librobocape.so `pkg-config --libs lcm`

SOURCES  := $(wildcard *.c)
INCLUDES := $(wildcard *.h)
LCMTYPES := $(shell find ../lcmtypes/ -name '*.lcm')
LCMOBJS	 := $(LCMTYPES:$%.lcm=$%.o)
OBJECTS  := $(SOURCES:$%.c=$%.o)

RM := rm -f

# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) $(@) $(OBJECTS) $(LCMOBJS) $(LFLAGS)


# compiling command
$(OBJECTS): %.o : %.c
	$(CC) $(CFLAGS) -c $< -o $(@)

all: $(TARGET)
	
clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
