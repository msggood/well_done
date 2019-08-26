#CC=gcc
CC=arm-linux-gnueabihf-gcc
FLAGS=
all:
	$(CC) $(FLAGS) -o encode main.c -L. -lencode
	cp encode ~/nfs

clean:
		rm *.o
