include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

OBJECTS = diskio.o fatfs.o ff.o ff_gen_drv.o user_diskio.o ccsbcs.o

all: libfatfs.a

libfatfs.a: $(OBJECTS)
	$(AR) crs libfatfs.a $(OBJECTS)

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.o: $(LIBFATFS_DIRECTORY)/%.c
	$(compile)

%.o: %.S
	$(assemble)

.PHONY: all clean

clean:
	$(RM) $(OBJECTS) libfatfs.a .*~ *~
