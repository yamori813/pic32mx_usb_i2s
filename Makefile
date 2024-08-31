# PIC32MX usbhost build make file
# with PinguinoX.4 (gcc 4.6)
#

MCS=../microchip_solutions_v2013-06-15

# https://github.com/biomurph/chipKIT-core-prebuilt
MP=../chipKIT-core-prebuilt/windows/chipkit-core/pic32/compiler/pic32-tools/pic32mx
# https://github.com/PinguinoIDE/pinguino-compilers
PINPATH=../pinguino-compilers/

LKRSCRIPT=selfboot.ld
LKRSCRIPT=selfboot.ld

PICLIBS=$(MP)/lib/no-float/libmchp_peripheral_32MX220F032B.a
PROCESSOR_O=processor.o
HEAP_SIZE=512

LDFLAGS=-msoft-float -Wl,--gc-sections $(MIPS16) \
	-L. -L$(MP)/lib/proc/32MX220F032B/ \
	-Wl,--defsym,_min_heap_size=$(HEAP_SIZE) \
	-Wl,-Map=output.map \
	-T$(LKRSCRIPT) \
	-Telf32pic32mx.x

BOARD=PIC32_PINGUINO_220
PROC=32MX220F032B
#MIPS16=-mips16

CC=$(PINPATH)/macosx/p32/bin/mips-elf-gcc
OBJC=$(PINPATH)/macosx/p32/bin/avr-objcopy
OBJDUMP=$(PINPATH)/macosx/p32/bin/mips-elf-objdump
SIZE=$(PINPATH)/macosx/p32/bin/mips-elf-size
PROG=../pic32prog/pic32prog

ELF_FLAGS=-EL -Os -ffunction-sections -fdata-sections -march=24kc 

INCLUDEDIRS=-I. -I$(MCS)/Microchip/Include -I$(MCS)/Microchip/USB -I$(MP)/include

include ./Objs.mak

CFLAGS=-fdollars-in-identifiers $(INCLUDEDIRS) -G0
CFLAGS+=-D__PIC32MX__ -D__$(PROC)__ -D$(BOARD)
CFLAGS+=-D__PIC32_FEATURE_SET__=220
CFLAGS+=-DCONFIG_EC12MHz
CFLAGS+=-DPUSHSW
CFLAGS+=-D__XC32
# 24 bit support is not work corrctly. Only work at 32KHz
#CFLAGS+=-DSAMPLE24

all: $(OBJS)
	cp $(MP)/lib/proc/32MX220F032B/processor.o .
	$(CC) $(ELF_FLAGS) $(CFLAGS) -o main32.elf \
		$(PROCESSOR_O) \
		$(OBJS) \
		$(PICLIBS) \
		$(LDFLAGS) \
		-lm -lgcc -lc
	$(OBJC) -O ihex main32.elf main32.hex

%.o : %.c
	$(CC) $(ELF_FLAGS) $(CFLAGS) $(MIPS16) -c $< -o $@

crt.o : crt0.S
	$(CC) $(ELF_FLAGS) -I$(MP)/include -c $< -o $@

# Microchip Libraries for Applications (MLA) code

usb_device.o : $(MCS)/Microchip/USB/usb_device.c
	$(CC) $(ELF_FLAGS) $(CFLAGS) $(MIPS16) -c "$<" -o $@

size:
	$(SIZE) main32.elf

objdump:
	$(OBJDUMP) -m mips:isa32r2 -b ihex -D main32.hex

flash:
	$(PROG) main32.hex

clean:
	rm -f *.o *.elf *.hex *.map


