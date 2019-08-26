#obj-m += gpio.o
obj-m += pl_interrupts.o

#KERN_DIR = /lib/modules/`uname -r`/build
KERN_DIR = /home/msg/linux-xlnx/

all:
	make -C $(KERN_DIR) M=`pwd` modules
	cp pl_interrupts.ko ~/nfs
clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order

