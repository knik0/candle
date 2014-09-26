CC= avr-gcc
CFLAGS= -O2 -Wall -mmcu=attiny13
LDFLAGS= -Wl,-Ttiny13flash.x -nostdlib -g
DEFS=-DTEST=0
INCLUDE=-I.
TARGET=main.elf
OBJS=
LIBS := $(LIBS) -lgcc

all: $(TARGET)
CANDLE=candle

test: DEFS=-DTEST=3
test: $(TARGET)

clean:
	rm -f *.dis *.bin *.hex *.elf

%.elf: %.c $(OBJS)
	$(CC) -S $(DEFS) $(INCLUDE) ${CFLAGS} $<
	$(CC) -o $@ $(DEFS) $(INCLUDE) ${CFLAGS} $(LDFLAGS) $< $(OBJS) $(LIBS)
	avr-objcopy -O binary $@ $(@:.elf=.bin)
	avr-objcopy -O ihex $@ $(@:.elf=.hex)
	avr-objdump $@ -dS >$(@:.elf=.dis)

candle: $(TARGET)
	rm -rf $(CANDLE)
	mkdir $(CANDLE)
	cp -avi *.bin *.hex *.c *.h *.x Makefile *.png $(CANDLE)/
	7z a -r candle.7z $(CANDLE)/

%.o: %.c
	${CC} -c -o $@ $(DEFS) $(INCLUDE) ${CFLAGS} $<

dep:
	makedepend $(DEFS) $(INCLUDE) -o.elf -Y *.c

# DO NOT DELETE

main.elf: tiny13a.h stdint.h
