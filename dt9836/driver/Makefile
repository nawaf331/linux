

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
    obj-m := dt9836.o 
    dt9836-objs :=  dt9836_device.o \
		    dt9836_driver.o \
		    dt9836_sysfs.o \
		    dt9836_fileop.o

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else

    KERNELDIR := /lib/modules/$(shell uname -r)/build
    PWD  := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
	depmod -a

.PHONY : clean install
	
endif
