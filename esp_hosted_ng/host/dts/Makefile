DTC ?= dtc
CPP := cpp
KERNELVER ?= $(shell uname -r)
KERNEL_INCLUDES ?= /lib/modules/$(KERNELVER)/build/include /lib/modules/$(KERNELVER)/source/include

DTCVERSION = $(shell $(DTC) --version | grep ^Version | sed 's/^.* //g')

ifeq ($(findstring 1,$(DTC_VERBOSE)),)
DTC_FLAGS += -Wno-unit_address_vs_reg -Wno-unit_address_format \
	-Wno-avoid_unnecessary_addr_size -Wno-alias_paths \
	-Wno-graph_child_address -Wno-simple_bus_reg \
	-Wno-unique_unit_address -Wno-pci_device_reg
endif

ifneq ($(findstring 2,$(DTC_VERBOSE)),)
DTC_FLAGS += -Wnode_name_chars_strict -Wproperty_name_chars_strict
endif

ifeq "$(DTCVERSION)" "1.6.1"
DTC_FLAGS += -Wno-interrupt_provider
endif

dtc_cpp_flags  = -nostdinc \
                 $(foreach item,$(KERNEL_INCLUDES),-I$(item)) -Itestcase-data \
                 -undef -D__DTS__

ALL_DTS_OVERLAYS := $(shell find */* -name \*.dts)
ALL_DTB_OVERLAYS := $(patsubst %.dts,%.dtbo,$(ALL_DTS_OVERLAYS))

all: $(ALL_DTB_OVERLAYS)

%.dtbo: %.dts
	$(CPP) $(dtc_cpp_flags) -x assembler-with-cpp -o $<.tmp $< ; \
	$(DTC) -O dtb -o $@ -b 0 -@ -i $< $(DTC_FLAGS) $<.tmp

clean:
	rm */*.dtbo */*.dts.tmp

help:
	@echo "Targets:"
	@echo "  all:                   Build all device tree binaries"
	@echo "  clean:                 Clean all generated files"
	@echo ""
	@echo "  <PLATFORM>/<DTS>.dtb   Build a single device tree binary"

PHONY += FORCE
FORCE:

.PHONY: $(PHONY)
