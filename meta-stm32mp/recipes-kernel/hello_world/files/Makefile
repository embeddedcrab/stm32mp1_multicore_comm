# Object file(s) to be built
obj-m := hello_world.o

SRC := $(shell pwd)

all:
		$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

module_install:
		$(MAKE) -C $(KERNEL_SRC) M=(SRC) modules_install

.PHONY:	modules clean

clean:
		rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
		rm -f Module.markers Module.symvers modules.order
		rm -rf .tmp_versions Modules.symvers