CC=gcc

CFLAGS  = -W -Wall
LDFLAGS = -lpthread -lm -lncurses
EXEC    = HexDMserver
LIB     = testlib.so
OBJECTS = HexDMserver.o ImageStreamIO.o

all: $(EXEC)


HexDMserver: $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o *.so
	rm -rf *~

mrproper: clean
	rm -rf $(EXEC)
