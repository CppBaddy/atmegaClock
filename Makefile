CC=avr-g++

DEVICE=328p

EFUSE=0xff
HFUSE=0xd9
LFUSE=0x62

# Device Signature
# ATmega8a 0x1E 0x93 0x07

CPPFLAGS += -std=c++11
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=atmega$(DEVICE) -DF_CPU=1000000
#CFLAGS=-g -std=c11 -Os -Wall -mcall-prologues -mmcu=atmega$(DEVICE) -DF_CPU=1000000
## Use short (8-bit) data types
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums

LDFLAGS = -Wl,-Map,$(TARGET).map
## Optional, but often ends up with smaller code
LDFLAGS += -Wl,--gc-sections

OBJ2HEX=avr-objcopy

SIZE=avr-size

UISP=avrdude

TARGET=atmegaClock

C_FILES = $(wildcard *.c)

OBJS = $(C_FILES:.c=.o)

all : build

#program : flash eeprom
program : flash

fuse :
	$(UISP) -p m$(DEVICE) -c USBasp -v -U hfuse:w:${HFUSE}:m -U lfuse:w:${LFUSE}:m 

flash : $(TARGET).hex
	$(UISP) -p m$(DEVICE) -c USBasp -v -U flash:w:$(TARGET).hex:i

eeprom : $(TARGET).eep
	$(UISP) -p m$(DEVICE) -c USBasp -v -U eeprom:w:$(TARGET).eep:i

build : $(TARGET).hex $(TARGET).eep
	ls -l $(TARGET).*

%.hex : %.elf
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

%.eep : %.elf
	$(OBJ2HEX) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@


%.elf : $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@
	$(SIZE) -t $@

%.o : %.c Makefile
	$(CC) $(CFLAGS) -c $< -o $@

clean :
	@rm -f *.hex *.eep *.elf *.o

help :
	@echo "make [help | clean | build | eeprom | flash | program | fuse]"
