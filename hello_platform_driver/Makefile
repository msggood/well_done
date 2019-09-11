obj-m += hello.o
#hello_test-objs := hello.o ./lib/libtest.a
#hello_test-objs := hello.o

#KERN_DIR = /lib/modules/`uname -r`/build
KERN_DIR = /home/msg/linux-xlnx

all:
	make -C $(KERN_DIR) M=`pwd` modules
	cp hello.ko ~/nfs/
clean:
	make -C $(KERN_DIR) M=`pwd` modules clean
	rm -rf modules.order

