# Makefile - Software Tools (Kernighan & Plauger)

CC=cc 
CFLAGS=-Weverything 
PROGS=test

all: $(PROGS)

docs: doxygen_config
	doxygen doxygen_config

test: test.c mb_transform.o
	$(CC) $(CFLAGS) -o test test.c mb_transform.o

mb_transform.o: mb_transform.c mb_transform.h
	$(CC) $(CFLAGS) -c mb_transform.c
	
	
clean:
	rm -rf *.o $(PROGS)	
