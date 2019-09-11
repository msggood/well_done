CC = cc

SRCS = $(wildcard *.c) $(wildcard src/*.c)
OBJS = $(patsubst %c, %o, $(SRCS))

INCLUDE = -I. -I./include

CFLAGS = -Wall -O2 $(INCLUDE)
CFLAGS += -g

LIB = -lpthread

TARGET = target

.PHONY:all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIB)
 
%o:%c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(OBJS) $(TARGET)

