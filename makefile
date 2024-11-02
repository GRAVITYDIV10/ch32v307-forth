FW_NAME ?= ch32v307-forth

SRCS += forth.s

CROSS_COMPILE ?= riscv64-elf-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OD = $(CROSS_COMPILE)objdump
OC = $(CROSS_COMPILE)objcopy
SZ = $(CROSS_COMPILE)size
NM = $(CROSS_COMPILE)nm

LINK_SCRIPT ?= link.ld

CFLAGS += \
	-march=rv32imaf_zicsr -mabi=ilp32f \
	-nostdlib  -x assembler-with-cpp -ggdb \
	-T $(LINK_SCRIPT)

all: $(FW_NAME).elf $(FW_NAME).bin $(FW_NAME).hex $(FW_NAME).dis $(FW_NAME).sym

info: all
	$(SZ) $(FW_NAME).elf
	ls -lh $(FW_NAME).bin

rebuild: clean all

reflash: rebuild flash info

OPENOCD_CMD ?= openocd-wch-riscv

flash: all
	$(OPENOCD_CMD) -c init -c halt -c 'flash erase_sector wch_riscv 0 last' -c exit
	$(OPENOCD_CMD) -c init -c halt -c 'program $(FW_NAME).elf' -c exit
	$(OPENOCD_CMD) -c init -c halt -c 'verify_image $(FW_NAME).elf' -c exit
	$(OPENOCD_CMD) -c init -c halt -c 'wlink_reset_resume' -c exit

reset:
	$(OPENOCD_CMD) -c init -c halt -c 'wlink_reset_resume' -c exit

clean:
	rm -f *.out *.o $(FW_NAME).dis $(FW_NAME).elf \
		$(FW_NAME).bin $(FW_NAME).hex $(FW_NAME).map \
		$(FW_NAME).sym

$(FW_NAME).bin: $(FW_NAME).elf
	$(OC) -O binary $(FW_NAME).elf $(FW_NAME).bin

$(FW_NAME).hex:
	$(OC) -I binary -O ihex $(FW_NAME).bin $(FW_NAME).hex

$(FW_NAME).elf:
	$(CC) $(CFLAGS) $(SRCS) -o $(FW_NAME).elf

$(FW_NAME).dis: $(FW_NAME).elf
	$(OD) -d -s $(FW_NAME).elf > $(FW_NAME).dis

$(FW_NAME).sym: $(FW_NAME).elf
	$(NM) $(FW_NAME).elf > $(FW_NAME).sym
