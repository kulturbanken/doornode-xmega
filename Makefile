CC	= avr-gcc
OBJCOPY	= avr-objcopy
OBJDUMP	= avr-objdump
SIZE	= avr-size
HIDSPX	= hidspx
MCU	= atxmega32d4
CFLAGS	= -Os -Wall -mmcu=$(MCU) -DF_CPU=2000000
LIBS	=
LDFLAGS	= -Wl,-Map,$(TARGET).map
SFLAGS	=
HFLAGS	= -d2
TARGET	= $(MCU)
OBJS	= main.o serial.o adc.o timer.o twi_slave_driver.o i2c.o usart_driver.o

PROGRAMMER = dragon_pdi
PRG_PORT   = usb

.PHONY: all size spx clean install check-syntax

all: $(TARGET).hex

size: $(TARGET).elf
	$(SIZE) $(SFLAGS) $(TARGET).elf

spx: all
	$(HIDSPX) $(HFLAGS) $(TARGET).hex

%.hex: %.elf
#	$(OBJCOPY) -j .text -j .data -O ihex $< $@
	$(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

%.o: %.s
	$(CC) -c -o $@ $<

%.o: %.S
	$(CC) -c -o $@ $<

$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o *.hex *.elf *.map

install: $(TARGET).hex
	avrdude -p $(MCU) -c $(PROGRAMMER) -P $(PRG_PORT) -U flash:w:$(TARGET).hex 

check-syntax:
	$(CC) -o nul -S ${CHK_SOURCES}
