CC=gcc
SRC = $(wildcard *.c)
OBJ = $(patsubst %.c, %, $(SRC))

.PHONY:all clean
all:$(OBJ)

clean:  
	rm -f $(OBJ) *.o 

