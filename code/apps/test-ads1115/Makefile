CONTIKI_PROJECT = test-ads1115
CONTIKI_WITH_RIME = 1
CONTIKI = ../../contiki

DEFINES += PROJECT_CONF_H=\"project-conf.h\"

# Include ads1115.c
PROJECTDIRS += $(CONTIKI)/../dev/cc2538/dev/ads1115

PROJECT_SOURCEFILES += ads1115.c

# Based on the crystal Makefile:
LDFLAGS += -specs=nosys.specs

# General contiki piece
all: $(CONTIKI_PROJECT)
include $(CONTIKI)/Makefile.include
