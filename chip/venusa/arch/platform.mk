# include by rules.mk
# define platfrom specification

CROSS_COMPILE := $(subst \,/,$(NUCLEI_TOOLCHAIN_PATH))/bin/riscv64-unknown-elf-

COREFLAGS := -march=rv32imac_zba_zbb_zbc_zbs -mabi=ilp32 -mcmodel=medlow -mtune=nuclei-300-series --specs=nosys.specs

AFLAGS    += -x assembler-with-cpp

CFLAGS    += -I $(TOPDIR)/chip/${CHIP}/bsp \
             -I $(TOPDIR)/chip/${CHIP}/include \
             -I $(TOPDIR)/chip/${CHIP}/include/register \
             -I $(TOPDIR)/modules/debug/include \
             -I $(TOPDIR)/include/NMSIS/Core/Include \
             -I $(TOPDIR)/include/bsp

ifeq ($(CONFIG_HARTID),1)
COREFLAGS  += -DBOOT_HARTID=1
LDFLAGS    += -DBOOT_HARTID=1
else
COREFLAGS  += -DBOOT_HARTID=0
LDFLAGS    += -DBOOT_HARTID=0
endif


ifneq ($(strip $(CONFIG_USE_SMP)),)
AFLAGS    += -DSMP_CPU_CNT=2
CFLAGS    += -DSMP_CPU_CNT=2
LDFLAGS   += -Wl,-defsym=__SMP_CPU_CNT=2
AFLAGS    += -DconfigNUMBER_OF_CORES=2
endif

# rtos module customization -TARGET.
# Difference: TARGET has MODULES, LIB does not.
ifneq ($(strip $(MODULES)),)
# check if rtos module is included
ifneq ($(findstring rtos,$(MODULES)),)

ifneq ($(findstring -lrtos,$(LIBS)),-lrtos)
$(error "rtos module is included, but -lrtos is not found in LIBS")
endif

# filter out the rtos module
MODULES := $(filter-out rtos, $(MODULES))

# define rtos target
.PHONY: rtos_custom
mk_libs : rtos_custom

rtos_custom :
	if [ -f "$(TOPDIR)/modules/rtos/Makefile" ]; then $(MAKE) -C $(TOPDIR)/modules/rtos $(MKDEFS) libs ||exit 1; fi;

# rtos module customization - TARGET & modules
# rtos include path & FreeRTOSConfig.h path
include $(TOPDIR)/modules/rtos/module_interface.mk

endif # ifneq ($(findstring rtos,$(MODULES)),)
endif # ifneq ($(strip $(MODULES)),)


# rtos module customization
ifeq (${CFG_RTOS},1)

ifeq ($(CFG_RTOS_SMP),1)
AFLAGS    += -DSMP_CPU_CNT=2
CFLAGS    += -DSMP_CPU_CNT=2
LDFLAGS   += -Wl,-defsym=__SMP_CPU_CNT=2

AFLAGS    += -DconfigNUMBER_OF_CORES=2
# CFLAGS    += -DconfigNUMBER_OF_CORES=2
endif


#decide to include the application specific config or default one
ifneq ($(strip $(CONFIG_FREERTOSCONFIG_H_PATH)),)
$(info "building FreeRTOS library: configured as $(CONFIG_FREERTOSCONFIG_H_PATH)")
CFLAGS  += -I $(TOPDIR)/$(CONFIG_FREERTOSCONFIG_H_PATH) \
                   -I $(TOPDIR)/modules/rtos/include
else
CFLAGS  += -I $(TOPDIR)/chip/${CHIP}/rtos_default_config \
           -I $(TOPDIR)/modules/rtos/include
endif

endif #CFG_RTOS

ifeq ($(CONFIG_TRACE), rtt)
COREFLAGS += -DCONFIG_TRACE

CFLAGS    += -I $(TOPDIR)/chip/${CHIP}/TraceRecorder/include \
             -I $(TOPDIR)/chip/${CHIP}/TraceRecorder/config \
             -I $(TOPDIR)/chip/${CHIP}/TraceRecorder/streamports/Jlink_RTT/include \
             -I $(TOPDIR)/chip/${CHIP}/TraceRecorder/streamports/Jlink_RTT/config

# Nessessary only when compiling TARGET. Difference: TARGET has MODULES, LIB does not.
ifneq ($(strip $(MODULES)),)
MODULES   += TraceRecorder
LIBS      += -ltrace
endif

endif

CFLAGS    += \
	-Wall -ffunction-sections -fdata-sections -static -ffast-math -fno-common -fno-builtin-printf
	
LDFLAGS   += \
	-march=rv32imac_zba_zbb_zbc_zbs -mabi=ilp32 -mtune=nuclei-300-series -static -Wl,-gc-sections -nostartfiles -Wl,--start-group -Wl,--end-group #--specs=nosys.specs -lstdc++ 
#remove --specs=nosys.specs for warning when link libc.a
	
ifeq ($(BUILD_NANO),1)
EXTLIBS   += -lc_nano -lg_nano 
else
EXTLIBS   += -lc -lgcc #-lc_nano -lg_nano #-lstdc++_nano -lsupc++_nano #-lm 
endif


LDSCRIPT  ?= $(TOPDIR)/chip/${CHIP}/arch/ram.ld


ifneq ($(strip $(CONFIG_EXT_RAM)),)
CFLAGS    += -DCONFIG_EXT_RAM
endif

ifneq ($(strip $(CONFIG_USE_RTT)),)
CFLAGS  += -DCONFIG_USE_RTT
CFLAGS  += -I $(TOPDIR)/modules/segger_rtt/include
MODULES += segger_rtt
LIBS    += -lsegger_rtt
endif
