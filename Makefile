obj-m += comedi.o

obj-m	+= kcomedilib/
obj-m	+= drivers/

comedi-objs :=		\
	comedi_fops.o	\
	proc.o		\
	range.o		\
	drivers.o	\
	comedi_compat32.o \

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

default:
	        $(MAKE) -C $(KERNELDIR) M=$(PWD) 

clean:
	        rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
		rm -rf ./drivers/*.o ./drivers/*.ko ./drivers/*.mod.c ./drivers/.*.cmd
		rm -rf ./kcomedilib/*.o ./kcomedilib/*.ko ./kcomedilib/*.mod.c ./kcomedilib/.*.cmd

copy:
	sudo cp drivers/ni_usb6008.ko /lib/modules/$(shell uname -r)/kernel/drivers/staging/comedi/drivers/ni_usb6008.ko
	sudo cp comedi.ko /lib/modules/$(shell uname -r)/kernel/drivers/staging/comedi/comedi.ko
	sudo depmod -a

depend .depend dep:
	        $(CC) $(CFLAGS) -M *.c > .depend

ifeq (.depend,$(wildcard .depend))
	include .depend
endif 

