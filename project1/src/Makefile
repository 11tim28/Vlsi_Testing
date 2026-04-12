#####################################################################
# Makefile for the BLIF file parser for different machines
# 
#
#
#####################################################################




#DEBUG_NETLIST		= -DNETLIST_DUMP
#DEBUG_BUILD_CKT	= -DDEBUG_BUILD_CKT
#DEBUG_MESSAGES		= -DDEBUG_MESSAGES
#DEBUG_MODE		= -DSTAND_ALONE
DEBUG_FLAGS		= $(DEBUG_NETLIST) $(YACCDEBUG) $(DEBUG_BUILD_CKT) $(DEBUG_MESSAGES) $(DEBUG_MODE)

CFLAGS			= -O $(INCLUDE) -DSIS
LIBS			= -lm #-ll

CC			= gcc
TARGET			= 3fsim

BUILDCKT_SRC		= build_ckt.c

PSRC			= main.c project.c \
			  build_ckt.c
POBJ			= main.o project.o \
			  build_ckt.o lex.yy.o y.tab.o 
PHDR			= project.h read_ckt.h


#----------------------------------------------------------------------

all:		$(TARGET)

main.o: main.c project.h
build_ckt.o: build_ckt.c  project.h
project.o: project.c project.h

$(TARGET):	$(POBJ)
		$(CC) -o $(TARGET) $(POBJ) $(LIBS)

.c.o:		
		$(CC) $(CFLAGS) $(DEBUG_FLAGS) -c $< -o $@

clean:
		rm -rf $(TARGET) project.o main.o read_ckt.o $(YACC_CPROG) $(YACC_HEADER) out.log




