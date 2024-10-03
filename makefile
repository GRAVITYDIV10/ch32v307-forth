FW_NAME ?= ch32v307-forth

CROSS_COMPILE ?= riscv64-unknown-elf-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OD = $(CROSS_COMPILE)objdump
OC = $(CROSS_COMPILE)objcopy
SZ = $(CROSS_COMPILE)size

LINK_SCRIPT ?= link.ld

CFLAGS += \
	-march=rv32imac_zicsr -mabi=ilp32 \
	-mno-relax -nostdlib \
	-x assembler-with-cpp -ggdb \
	-T $(LINK_SCRIPT)

LDFLAGS += \
	-b elf32-littleriscv \
	--print-memory-usage \
	--no-relax-gp \
	-T $(LINK_SCRIPT)

#CFLAGS += -DTEST

all: $(FW_NAME).elf $(FW_NAME).bin $(FW_NAME).hex $(FW_NAME).dis
	$(SZ) $(FW_NAME).elf

clean:
	rm -f *.out *.o $(FW_NAME).dis $(FW_NAME).elf $(FW_NAME).bin $(FW_NAME).hex

$(FW_NAME).hex:
	$(OC) -O ihex $(FW_NAME).elf $(FW_NAME).hex

$(FW_NAME).bin:
	$(OC) -O binary $(FW_NAME).elf $(FW_NAME).bin

$(FW_NAME).elf:
	$(CC) $(CFLAGS) forth.s -c -o forth.o
	unix2dos motd.txt
	$(OC) -I binary -O elf32-littleriscv motd.txt _motd.o
	$(OC) --rename-section .data=.rodata _motd.o motd.o
	$(LD) $(LDFLAGS) motd.o forth.o -o $(FW_NAME).elf

$(FW_NAME).dis: $(FW_NAME).elf
	$(OD) -d -s $(FW_NAME).elf > $(FW_NAME).dis
