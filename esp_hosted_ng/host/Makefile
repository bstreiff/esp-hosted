# Toolchain and kernel
CROSS_COMPILE :=
KERNEL := /lib/modules/$(shell uname -r)/build

include $(KERNEL)/scripts/subarch.include
ARCH ?= $(SUBARCH)

MODULE_PREFIX := esp32

$(eval $(shell grep 'CONFIG_MMC=' $(KERNEL)/include/config/auto.conf))
$(eval $(shell grep 'CONFIG_SPI_MASTER=' $(KERNEL)/include/config/auto.conf))

# Debug and AP support
CONFIG_DEBUG_LOGS ?= y
ifeq ($(CONFIG_AP_SUPPORT), y)
    EXTRA_CFLAGS += -DCONFIG_AP_MODE
endif

# Source and include paths
PWD := $(shell pwd)
ccflags-y += -I$(src) -I$(src)/include -I$(CURDIR) -I$(CURDIR)/include
EXTRA_CFLAGS += -I$(M) -I$(M)/include

# Interface-specific sources
ifeq ($(CONFIG_MMC), y)
    ccflags-y += -I$(src)/sdio -I$(CURDIR)/sdio
    EXTRA_CFLAGS += -I$(M)/sdio
    $(MODULE_PREFIX)_sdio-objs += sdio/esp_sdio.o sdio/esp_sdio_api.o
    obj-m += $(MODULE_PREFIX)_sdio.o
endif

ifeq ($(CONFIG_SPI_MASTER), y)
    ccflags-y += -I$(src)/spi -I$(CURDIR)/spi
    EXTRA_CFLAGS += -I$(M)/spi
    $(MODULE_PREFIX)_spi-objs += spi/esp_spi.o
    obj-m += $(MODULE_PREFIX)_spi.o
endif

# Core module build rules
$(MODULE_PREFIX)-objs += esp_bt.o main.o esp_cmd.o esp_utils.o esp_cfg80211.o esp_stats.o esp_debugfs.o esp_log.o
CFLAGS_esp_log.o = -DDEBUG
obj-m += $(MODULE_PREFIX).o

# Build targets
all: clean
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o */*.o *.ko *.mod.c *.symvers *.order .*.cmd .tmp_versions
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean

check:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) $(module_objects)

