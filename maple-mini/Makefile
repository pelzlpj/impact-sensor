
ifndef LIBMAPLE_REPO
    $(error LIBMAPLE_REPO must be set to a local libmaple git repository.)
endif

ifndef TOOLCHAIN_PATH
    $(error TOOLCHAIN_PATH must be set to the location for the ARM GCC binaries.)
endif

.DEFAULT_GOAL := sketch

SHELL := /bin/bash
export PATH  := $(TOOLCHAIN_PATH):$(PATH)
export BOARD := maple_mini

BUILD_PATH := build
LDDIR   := $(LIBMAPLE_REPO)/support/ld
MAKEDIR := $(LIBMAPLE_REPO)/support/make
BOARD_INCLUDE_DIR := $(MAKEDIR)/board-includes

MEMORY_TARGET := flash

# $(BOARD)- and $(MEMORY_TARGET)-specific configuration
include $(MAKEDIR)/target-config.mk

# Build rules and useful templates
include make/build-rules.mk
include $(MAKEDIR)/build-templates.mk

GLOBAL_CFLAGS   := -I$(LIBMAPLE_REPO)/libmaple/include -I$(LIBMAPLE_REPO)/wirish/include \
	               -I$(LIBMAPLE_REPO)/libmaple/$(MCU_SERIES)/include \
				   -I$(LIBMAPLE_REPO)/wirish/boards/$(BOARD)/include \
                   -Os -g3 -gdwarf-2 -nostdlib -std=gnu++0x \
                   -ffunction-sections -fdata-sections \
                   -Wl,--gc-sections $(TARGET_FLAGS)
GLOBAL_CXXFLAGS := -fno-rtti -fno-exceptions -Wall $(TARGET_FLAGS)
GLOBAL_ASFLAGS  := -x assembler-with-cpp $(TARGET_FLAGS)
LDFLAGS  = $(TARGET_LDFLAGS) $(TOOLCHAIN_LDFLAGS) -mcpu=cortex-m3 -mthumb \
           -Xlinker --gc-sections \
           -Xassembler --march=armv7-m -Wall


ACCEL_SRC := util.cpp serial_util.cpp RN42.cpp main.cpp

ACCEL_OBJ := $(addprefix $(BUILD_PATH)/,$(ACCEL_SRC:.cpp=.o))

BUILDDIRS += $(BUILD_PATH)

##
## Targets
##

.PHONY: sketch clean

BOARD_USB_VENDOR_ID  := 1EAF
BOARD_USB_PRODUCT_ID := 0003
UPLOAD_flash := $(LIBMAPLE_REPO)/support/scripts/reset.py && \
                sleep 1                  && \
                $(DFU) -a1 -d $(BOARD_USB_VENDOR_ID):$(BOARD_USB_PRODUCT_ID) -D $(BUILD_PATH)/$(BOARD).bin -R

# Conditionally upload to whatever the last build was
install: INSTALL_TARGET = $(shell cat $(BUILD_PATH)/build-type 2>/dev/null)
install: $(BUILD_PATH)/$(BOARD).bin
	@echo "Install target:" $(INSTALL_TARGET)
	$(UPLOAD_$(INSTALL_TARGET))

clean:
	rm -rf $(BUILD_PATH)
	$(MAKE) -C $(LIBMAPLE_REPO) clean

sketch: $(BUILD_PATH)/$(BOARD).bin

$(LIBMAPLE_REPO)/build/libmaple.a:
	echo $(PATH)
	$(MAKE) -C $(LIBMAPLE_REPO) library

$(BUILD_PATH)/$(BOARD).elf: $(BUILDDIRS) $(TGT_BIN) $(ACCEL_OBJ) $(LIBMAPLE_REPO)/build/libmaple.a
	$(SILENT_LD) $(CXX) $(LDFLAGS) -o $@ $(TGT_BIN) $(ACCEL_OBJ) $(LIBMAPLE_REPO)/build/libmaple.a \
	   	-Wl,-Map,$(BUILD_PATH)/$(BOARD).map

$(BUILD_PATH)/$(BOARD).bin: $(BUILD_PATH)/$(BOARD).elf
	$(SILENT_OBJCOPY) $(OBJCOPY) -v -Obinary $(BUILD_PATH)/$(BOARD).elf $@ 1>/dev/null
	$(SILENT_DISAS) $(DISAS) -d $(BUILD_PATH)/$(BOARD).elf > $(BUILD_PATH)/$(BOARD).disas
	@echo " "
	@echo "Object file sizes:"
	@find $(BUILD_PATH) -iname *.o | xargs $(SIZE) -t > $(BUILD_PATH)/$(BOARD).sizes
	@cat $(BUILD_PATH)/$(BOARD).sizes
	@echo " "
	@echo "Final Size:"
	@$(SIZE) $<
	@echo $(MEMORY_TARGET) > $(BUILD_PATH)/build-type

$(BUILDDIRS):
	@mkdir -p $@

MSG_INFO:
	@echo "================================================================================"
	@echo ""
	@echo "  Build info:"
	@echo "     BOARD:          " $(BOARD)
	@echo "     MCU:            " $(MCU)
	@echo "     MEMORY_TARGET:  " $(MEMORY_TARGET)
	@echo ""
	@echo "  See 'make help' for all possible targets"
	@echo ""
	@echo "================================================================================"
	@echo ""

