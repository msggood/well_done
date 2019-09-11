#CFLAGS=`pkg-config libgvc --cflags` -Wall -g -O2
#LDFLAGS=`pkg-config libgvc --libs`
CFLAGS=-I/usr/include/graphviz -Wall -g -O2
LDFLAGS=-L/usr/lib/x86_64-linux-gnu -lgvc -lcgraph -lcdt
CC=gcc

all: dot demo simple example

dot: dot.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
demo: demo.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
simple: simple.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
example: example.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

clean:
	rm -rf *.o dot demo simple example *.png
