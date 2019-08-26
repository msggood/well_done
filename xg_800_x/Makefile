CC = arm-linux-gnueabihf-gcc
AR = arm-linux-gnueabihf-ar

PLATFORM = platform_linux
CONSOLE_COMMANDS = console_commands
ENCODE = encode

SRCS = $(wildcard *.c) $(wildcard $(PLATFORM)/*.c) $(wildcard $(CONSOLE_COMMANDS)/*.c) $(wildcard $(ENCODE)/*.c)

OBJS = $(patsubst %c, %o, $(SRCS))
# -I指定头文件目录
INCLUDE = -I. -I$(PLATFORM) -I$(CONSOLE_COMMANDS) -Imatio/include

# 开启编译warning和设置优化等级
CFLAGS = -Wall -Wextra -Os -ffunction-sections -fdata-sections  $(INCLUDE)
CFLAGS += -g

LIB = -lpthread -lm -L./encode -lencode -Wl,--gc-sections
#LIB += -Lmatio/lib lmatio

TARGET = ad9361_linux

.PHONY:all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIB)
	-cp $(TARGET) ~/nfs
 
%o:%c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(OBJS) $(TARGET)

libad9361.a:$(filter-out main.c, $(SRCS))
	$(AR) rvs libad9361.a $+
