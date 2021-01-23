######################################################################
#  Project
######################################################################

BINARY		= main
SRCFILES	= main.c lib/pcd8544/pcd8544.c lib/printf/printf.c lib/ugui/ugui.c


all: elf bin

include ../../Makefile.incl

CFLAGS += -DPRINTF_INCLUDE_CONFIG_H

# End
