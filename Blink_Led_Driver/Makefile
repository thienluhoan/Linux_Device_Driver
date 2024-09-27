EXTRA_CFLAGS = -Wall
obj-m += blink_led.o

KDIR := /home/luhthien/beagle_bone/kernelbuildscripts/bb-kernel/KERNEL
TOOLS := /home/luhthien/beagle_bone/kernelbuildscripts/bb-kernel/dl/gcc-11.5.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-

all:
	make ARCH=arm CROSS_COMPILE=$(TOOLS) -C $(KDIR) M=$(shell pwd) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean

cp:
	scp blink_led.ko ubuntu@192.168.9.9:/home/ubuntu/