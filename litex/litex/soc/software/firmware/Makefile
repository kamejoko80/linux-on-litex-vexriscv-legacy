include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

# Permit TFTP_SERVER_PORT override from shell environment / command line
ifdef TFTP_SERVER_PORT
CFLAGS += -DTFTP_SERVER_PORT=$(TFTP_SERVER_PORT)
endif

OBJECTS = isr.o main.o spi.o boot-helper-$(CPU).o
OBJECTS += fatfs_sd.o app.o

all: firmware.bin

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

firmware.elf: $(FIRMWARE_DIRECTORY)/linker.ld $(OBJECTS)

%.elf: ../libbase/crt0-$(CPU)-ctr.o ../libnet/libnet.a ../libfatfs/libfatfs.a ../libbase/libbase-nofloat.a ../libcompiler_rt/libcompiler_rt.a
	$(LD) $(LDFLAGS) -T $(FIRMWARE_DIRECTORY)/linker.ld -N -o $@ \
		../libbase/crt0-$(CPU)-ctr.o \
		$(OBJECTS) \
		-L../libnet \
		-L../libfatfs \
		-L../libbase \
		-L../libcompiler_rt \
		-lnet -lfatfs -lbase-nofloat -lcompiler_rt -Map=firmware.map
ifneq ($(OS),Windows_NT)
	chmod -x $@
endif

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.o: $(FIRMWARE_DIRECTORY)/%.c
	$(compile)

%.o: $(FIRMWARE_DIRECTORY)/%.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) firmware.elf firmware.bin .*~ *~

.PHONY: all clean
