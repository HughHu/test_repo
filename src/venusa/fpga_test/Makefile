TOPDIR=../../..

#lib or target you want to build: eg. LIB=libmisc.a  or TARGET=wlan
LIB		=
TARGET	= fpga_test

# modules depend by this module
MODULES = bsp driver rtos segger_rtt ls_jpeg

#libraries depend: eg. LIBS = -lbsp -ldrv -lrtos
LIBS	= -lbsp -lrtos -lsegger_rtt -lls_jpeg
ifeq ($(IC_BOARD), 0)
LIBS	+= -ldrv_fpga
else
LIBS	+= -ldrv
endif

#exclude subdirs
exclude_dirs=$(patsubst ./%,%,$(shell find . -maxdepth 1 -type d))

#files want to build: eg. CSRCS=a.c  CPPSRCS=b.cpp  SSRCS=c.S
CSRCS  = $(shell find ./resource -name "*.c")
CSRCS += $(shell find ./uvc -name "*.c")
CSRCS += ./jpeg_dvp.c
CSRCS += ./main.c
CPPSRCS	=
SSRCS	= $(wildcard *.S)

#includings and flags
CFLAGS = -I $(TOPDIR)/include/CMSIS \
         -I $(TOPDIR)/include/CMSIS/core \
         -I $(TOPDIR)/include/NMSIS/Core/Include \
		 -I $(TOPDIR)/include/bsp \
		 -I $(TOPDIR)/modules/segger_rtt/include \
		 -I $(TOPDIR)/include \
		 -I $(TOPDIR)/chip/${CHIP}/bsp \
		 -I $(TOPDIR)/chip/${CHIP}/include/register \
		 -I $(TOPDIR)/chip/${CHIP}/include \
		 -I $(TOPDIR)/chip/${CHIP}/ls_jpeg \
		 -I ../../drv_demo/drv_demo_venusa/video/config \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys/class/cdc \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys/class/vendor \
		 -I $(TOPDIR)/chip/${CHIP}/driver/usb_subsys/class/video \
		 -I ./ \
		 -I ./uvc \
		 
export CFG_TUSB_CONFIG_H_PATHNAME = $(abspath .)/uvc/tusb_config.h
CFLAGS  += -DCFG_TUSB_CONFIG_FILE=\"$(CFG_TUSB_CONFIG_H_PATHNAME)\" 

#export the FreeRTOSConfig.h path 
#defined as CONFIG_FREERTOSCONFIG_H_PATH for rtos.a compiling
export CONFIG_FREERTOSCONFIG_H_PATH=src/venusa/fpga_test
export CFG_RTOS=1


include $(TOPDIR)/rules.mk
LDSCRIPT  = ./ram.ld
