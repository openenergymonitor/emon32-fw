CC = cc
CFLAGS = -Wall -Wpedantic -g
INCLUDES = ../src/ -I./
DEFINES = -DHOSTED
LIBS = -lm

cm: OBJS = test_cm.c ../src/emon_CM.c ../src/board_def.c
eeprom: OBJS = test_eeprom.c ../src/eeprom.c
util: OBJS = test_util.c ../src/util.c

.PHONY: clean all

all: cm eeprom util

eeprom:
	${CC} ${CFLAGS} $(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
cm:
	${CC} ${CFLAGS} $(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
util:
	${CC} ${CFLAGS} $(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
clean:
	rm -f *.test
	rm -rf *dSYM/
