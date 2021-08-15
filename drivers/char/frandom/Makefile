# Makefile for kernel module

ifneq ($(KERNELRELEASE),)
obj-m	:= frandom.o

else
KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
endif

