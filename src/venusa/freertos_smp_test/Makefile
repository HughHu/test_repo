TOPDIR=../../..

#lib or target you want to build: eg. LIB=libmisc.a  or TARGET=wlan
LIB		=
TARGET	= freertos_smp_test

# modules depend by this module
MODULES = bsp driver rtos

#libraries depend: eg. LIBS = -lbsp -ldrv -lrtos
LIBS	= -lbsp -lrtos
ifeq ($(IC_BOARD), 0)
LIBS	+= -ldrv_fpga
else
LIBS	+= -ldrv
endif

#exclude subdirs
exclude_dirs=

#files want to build: eg. CSRCS=a.c  CPPSRCS=b.cpp  SSRCS=c.S
CSRCS	= $(wildcard *.c)
CPPSRCS	=
SSRCS	= $(wildcard *.S)

#includings and flags
CFLAGS = -I $(TOPDIR)/chip/${CHIP}/include


export CONFIG_FREERTOSCONFIG_H_PATH = src/venusa/freertos_smp_test
export CFG_RTOS=1
export CFG_RTOS_SMP=1

include $(TOPDIR)/rules.mk


# LDSCRIPT = ./ram.ld
LDSCRIPT = ./flash.ld


# ext:
# 	@sh $(TOPDIR)/tools/mkhdr.sh $(TGTOUT)/$(TARGET).bin	
