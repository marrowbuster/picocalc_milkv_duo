
# local kernel build dir
# KERN_DIR ?= /lib/modules/$(shell uname -r)/build

# kernel dir
ARCH := riscv
CROSS_COMPILE := ../../duo-buildroot-sdk-v2/host-tools/gcc/riscv64-linux-musl-x86_64/bin/riscv64-unknown-linux-musl-
KERN_DIR := ../../linux_5.10

MODULE_NAME = picocalc_snd_pwm

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=`pwd` modules
	$(CROSS_COMPILE)strip --strip-debug picocalc_snd_pwm.ko
clean:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERN_DIR) M=`pwd` modules clean

clena: clean

#CFLAGS_$(MODULE_NAME).o := -DDEBUG
obj-m += picocalc_snd_pwm.o
