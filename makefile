#Place this file into one directory together with Main.c

PROGRAM = openrex-blender-3d-position

C_FILES := $(wildcard *.c)
OBJS := $(patsubst %.c, %.o, $(C_FILES))
CC = cc
CFLAGS = -Wall -pedantic -std=gnu99
LDFLAGS =

all: $(PROGRAM)

$(PROGRAM): .depend $(OBJS)
        $(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) -o $(PROGRAM) -lm

depend: .depend

.depend: cmd = gcc -MM -MF depend $(var); cat depend >> .depend;
.depend:
        @echo "Generating dependencies..."
        @$(foreach var, $(C_FILES), $(cmd))
        @rm -f depend

-include .depend

# These are the pattern matching rules. In addition to the automatic
# variables used here, the variable $* that matches whatever % stands for
# can be useful in special cases.
%.o: %.c
        $(CC) $(CFLAGS) -c $< -o $@

%: %.c
        $(CC) $(CFLAGS) -o $@ $<

clean:
        rm -f .depend *.o

.PHONY: clean depend
