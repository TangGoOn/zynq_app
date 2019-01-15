# Cross compiler makefile for my_gpio
# By Nightmare @ EEFOCUS 2012-10-10
KERN_SRC=/home/tom/zynq/kernel/linux-digilent-digilent-v3.6
obj-m := my_axi.o

all:
	make -C $(KERN_SRC) ARCH=arm M=`pwd` modules
clean:
	make -C $(KERN_SRC) ARCH=arm M=`pwd=` clean
