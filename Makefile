# Makefile for ac020-thermal out-of-tree kernel module
# Targets the Raspberry Pi CM5 kernel (Linux 6.6.y)
#
# Build:
#   make
# Build for a specific kernel:
#   make KDIR=/path/to/kernel/source
# Cross-compile for Pi (from x86):
#   make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- KDIR=/path/to/pi/kernel

obj-m := ac020-thermal.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

# ---------------------------------------------------------------
# DTS overlay targets
# Requires the device tree compiler: sudo apt install device-tree-compiler
# ---------------------------------------------------------------

DTC      ?= dtc
DTS_DIR  := dts
OVERLAYS := $(patsubst $(DTS_DIR)/%.dts,$(DTS_DIR)/%.dtbo,$(wildcard $(DTS_DIR)/*.dts))

dtbo: $(OVERLAYS)

$(DTS_DIR)/%.dtbo: $(DTS_DIR)/%.dts
	$(DTC) -@ -I dts -O dtb -o $@ $<

install_dtbo: dtbo
	sudo cp $(DTS_DIR)/*.dtbo /boot/overlays/
	@echo "Overlays installed to /boot/overlays/"
	@echo "Add the appropriate dtoverlay= line to /boot/config.txt"

.PHONY: all clean install dtbo install_dtbo
