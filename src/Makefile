CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -mcpu=cortex-m0 -mthumb -Os -g -ffreestanding -Wall -Werror
LDFLAGS = -T linker.ld -nostartfiles -nodefaultlibs -lgcc

SRCS = main.c spi_rc.c sfhss.c mpu6500.c orientation.c stabilize.c

all: firmware.bin

firmware.elf: $(SRCS) spi_rc.h sfhss.h mpu6500.h orientation.h stabilize.h stm32f031.h linker.ld
	$(CC) $(CFLAGS) -o firmware.elf $(SRCS) $(LDFLAGS)

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary firmware.elf firmware.bin

clean:
	rm -f *.elf *.bin
