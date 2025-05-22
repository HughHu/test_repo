TOPDIR=../../..

#lib or target you want to build: eg. LIB=libmisc.a  or TARGET=wlan
LIB		=
TARGET	= gimbal_demo

# modules depend by this module
MODULES = bsp driver rtos ls_jpeg

#libraries depend: eg. LIBS = -lbsp -ldrv -lrtos
LIBS	= -lbsp -lrtos -lls_jpeg
ifeq ($(IC_BOARD), 0)
LIBS	+= -ldrv_fpga
else
LIBS	+= -ldrv
endif

#exclude subdirs
exclude_dirs=

#files want to build: eg. CSRCS=a.c  CPPSRCS=b.cpp  SSRCS=c.S
CSRCS	= $(wildcard *.c)
CSRCS	+= $(shell find ./uvc -name "*.c")
CSRCS	+= $(shell find ./resource -name "*.c")
CPPSRCS	=
SSRCS	= $(wildcard *.S)

#includings and flags
CFLAGS = -I . \
		 -I ./uvc \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys/class/cdc \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys/class/vendor \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys/class/video \
		 -I $(TOPDIR)/chip/${CHIP}/ls_jpeg 

export CFG_TUSB_CONFIG_H_PATHNAME = $(abspath .)/uvc/tusb_config.h
CFLAGS  += -DCFG_TUSB_CONFIG_FILE=\"$(CFG_TUSB_CONFIG_H_PATHNAME)\" 

export CONFIG_FREERTOSCONFIG_H_PATH=src/venusa/gimbal_demo
export CFG_RTOS=1
export CFG_RTOS_SMP=1
IC_BOARD=0
export IC_BOARD

include $(TOPDIR)/rules.mk


LDSCRIPT = ./ram.ld
# LDSCRIPT = ./flash.ld


# ext:
# 	@sh $(TOPDIR)/tools/mkhdr.sh $(TGTOUT)/$(TARGET).bin	
