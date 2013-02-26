# Example Makefile for Contiki projects
#     Author Marcus Lunden

# this defines where to find the Contiki root folder compared with this folder
# I prefer to have my Contiki projects under eg /contiki/projects/myproject/
# in which case CONTIKI = ../.. is a proper setting
CONTIKI = ../../..

# this sets the compilation target (hardware) to be Launchpad
ifndef TARGET
TARGET=launchpad
endif

# Change this to the main file of your project (NB without file ending)
PROJECT = blink-rtimer

# if you have more than one source file to include, list them here
#PROJECT_SOURCEFILES += example.c other.c

# if files are in several folders, list them here
#PROJECT_FOLDERS     += ./example ./example/deep/deeper

# any overridden Contiki source file (in this project) is listed here
#CONTIKI_SOURCEFILES += 

CFLAGS += -Wl,--oihex

all: $(PROJECT)
	@msp430-size $(PROJECT).launchpad

# must be here, it makes sure Contiki is included :)
include $(CONTIKI)/Makefile.include
