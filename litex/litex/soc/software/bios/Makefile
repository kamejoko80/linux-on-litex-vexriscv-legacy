include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# Permit TFTP_SERVER_PORT override from shell environment / command line
ifdef TFTP_SERVER_PORT
CFLAGS += -DTFTP_SERVER_PORT=$(TFTP_SERVER_PORT)
endif

CPUFLAGS += -D$(PLATFORM) 

OBJECTS=isr.o sdram.o main.o boot-helper-$(CPU).o boot.o

# Soucre including
ifneq ($(PLATFORM), $(filter $(PLATFORM), PLATFORM_QMATECH \
                                          PLATFORM_ACCEL_TEST \
                                          PLATFORM_AT7CORE \
                                          ))
OBJECTS += fatfs_sd.o app.o spi.o can.o
endif

# Soucre including
ifeq ($(PLATFORM), PLATFORM_ACCEL_TEST)
OBJECTS += accel_test.o spi.o
endif

# Soucre including
ifeq ($(PLATFORM), PLATFORM_AT7CORE)
OBJECTS += sf_programmer.o
endif

all: bios.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif
ifeq ($(CPUENDIANNESS),little)
	$(PYTHON) -m litex.soc.software.mkmscimg $@ --little
else
	$(PYTHON) -m litex.soc.software.mkmscimg $@
endif

bios.elf: $(BIOS_DIRECTORY)/linker.ld $(OBJECTS)

%.elf: ../libbase/crt0-$(CPU)-ctr.o ../libnet/libnet.a ../libfatfs/libfatfs.a ../libbase/libbase-nofloat.a ../libcompiler_rt/libcompiler_rt.a
	$(LD) $(LDFLAGS) -T $(BIOS_DIRECTORY)/linker.ld -N -o $@ \
		../libbase/crt0-$(CPU)-ctr.o \
		$(OBJECTS) \
		-L../libnet \
		-L../libfatfs \
		-L../libbase \
		-L../libcompiler_rt \
		-lnet -lfatfs -lbase-nofloat -lcompiler_rt -Map=bios.map
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.o: $(BIOS_DIRECTORY)/%.c
	$(compile)

%.o: $(BIOS_DIRECTORY)/%.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) bios.elf bios.bin .*~ *~

.PHONY: all clean
