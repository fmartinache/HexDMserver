CC=gcc

CFLAGS  = -W -Wall
LDFLAGS = -lpthread -lm -lncurses
EXEC    = HexDMserver
LIB     = testlib.so
OBJECTS = HexDMserver.o ImageStreamIO.o

# adding the BMC specific things as they are setup on this machine

BMC_LIBDIR = $(HOME)/Progs/DM/lib/
BMC_INCDIR = $(HOME)/Progs/DM/src/inc/

CFLAGS  += -I $(BMC_INCDIR)
LDFLAGS += -L $(BMC_LIBDIR) -lbmcmd

# the makefile instructions

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
