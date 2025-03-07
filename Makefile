CC=g++

CFLAGS  = -std=c++17 -W -Wall -Wextra
LDFLAGS = -lcommander -lpthread -lzmq -lImageStreamIO -lboost_program_options -lfmt
EXEC    = HexDM_server
OBJECTS = commander_HexDM_server.o

# PREFIX is environment variable, but if it is not set, then set default value
ifeq ($(PREFIX),)
    PREFIX := /usr/local
endif

# adding the BMC specific things as they are setup on this machine

BMC_LIBDIR = /opt/Boston\ Micromachines/lib
BMC_INCDIR = /opt/Boston\ Micromachines/include/

CFLAGS  += -I $(BMC_INCDIR)
LDFLAGS += -L $(BMC_LIBDIR) -lBMC -lBMC_PCIeAPI -lBMC_USBAPI

# the makefile instructions

all: $(EXEC)


HexDM_server: $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o *.so
	rm -rf *~

mrproper: clean
	rm -rf $(EXEC)

install:
	install -D $(EXEC) $(PREFIX)/bin/
	install -D 27BW007#051.dm /opt/Boston\ Micromachines/Profiles/
