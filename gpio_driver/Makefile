obj-m += gpio.o

#KERN_DIR = /lib/modules/`uname -r`/build
KERN_DIR = /home/msg/linux-xlnx/

all:
	make -C $(KERN_DIR) M=`pwd` modules
clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order

