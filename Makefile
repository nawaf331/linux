
# If your target Linux distro does not have ezusb.ko then
# define environment variable BUILD_EZUSB as follows
# export BUILD_EZUSB=m. In addition, create the soft links
# described in the README
#
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
	obj-$(BUILD_EZUSB) := ezusb/
    	obj-y += dt9836/driver/

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else

    KERNELDIR := /lib/modules/$(shell uname -r)/build
    PWD  := $(shell pwd)
    FIRMWAREDIR := /lib/firmware/$(shell uname -r)/datx

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
	depmod -a

firmware:
	mkdir -p $(FIRMWAREDIR)
	cp -r ./firmware/*.fw $(FIRMWAREDIR)

.PHONY : clean install firmware	
endif
