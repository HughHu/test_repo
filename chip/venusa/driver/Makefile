TOPDIR=../../..

#lib or target you want to build: eg. LIB=libmisc.a  or TARGET=wlan
ifeq ($(IC_BOARD), 0)
LIB		= libdrv_fpga.a
else
LIB		= libdrv.a
endif

TARGET	=

#libraries depend: eg. LIBS = -lbsp -ldrv -lrtos
LIBS	= 
ifeq ($(IC_BOARD), 0)
LIBS	+= 
else
LIBS	+= 
endif

#exclude subdirs
exclude_dirs= $(patsubst ./%,%,$(shell find . -maxdepth 3 -type d))

#files want to build: eg. CSRCS=a.c  CPPSRCS=b.cpp  SSRCS=c.S
CSRCS	= $(shell find . -name "*.c")
CPPSRCS	=
SSRCS	=

#includings and flags
CFLAGS =  -I $(TOPDIR)/include/NMSIS/Core/Include \
          -I $(TOPDIR)/chip/${CHIP}/bsp \
          -I $(TOPDIR)/chip/${CHIP}/include \
          -I $(TOPDIR)/chip/${CHIP}/include/register \
          -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys \
          -I $(TOPDIR)/chip/${CHIP}/driver/isp/hal \
          -I $(TOPDIR)/include/bsp \
          -I $(TOPDIR)/include/rtos \
          -I $(TOPDIR)/include/wifi \
          -I $(TOPDIR)/bt/plf/api \
          -I $(TOPDIR)/include/debug \
          -I $(TOPDIR)/wifi/macsw/modules/bufpool/src \
          -I $(TOPDIR)/wifi/macsw/modules/hif/src \
          -I $(TOPDIR)/chip/${CHIP}/rtos/rtos_al \
          -I $(TOPDIR)/modules/rtos/portable/${BUILDTOOL}/non_secure \
          -I $(TOPDIR)/modules/ota/include

#decide to include the application specific tusb_config.h or default one
ifneq ($(strip $(CFG_TUSB_CONFIG_H_PATHNAME)),)
$(info "PRIVATE tusb_config.h @ $(CFG_TUSB_CONFIG_H_PATHNAME)")
#absolute path
CFLAGS  += -DCFG_TUSB_CONFIG_FILE=\"$(CFG_TUSB_CONFIG_H_PATHNAME)\"
else
$(info "PUBLIC tusb_config.h @ $(TOPDIR)/chip/${CHIP}/driver/usb_subsys/tusb_config.h")
endif



ifeq ($(CFG_AMP_IPC), 1)
CFLAGS += -DCFG_AMP_IPC
endif


include $(TOPDIR)/rules.mk

