TOPDIR=../../..

#lib or target you want to build: eg. LIB=libmisc.a  or TARGET=wlan
LIB		= 
TARGET = baremetal_smp_test

# modules depend by this module
MODULES = bsp driver

#libraries depend: eg. LIBS = -lbsp -ldrv -lrtos
LIBS	= -lbsp

ifeq ($(IC_BOARD), 0)
LIBS	+= -ldrv_fpga
else
LIBS	+= -ldrv
endif

#exclude subdirs
#exclude_dirs=$(patsubst ./%,%,$(shell find . -maxdepth 1 -type d))

#files want to build: eg. CSRCS=a.c  CPPSRCS=b.cpp  SSRCS=c.S
CSRCS	= $(wildcard *.c)
CPPSRCS	= 
SSRCS	= 

#includings and flags
CFLAGS = -I .

export CONFIG_USE_SMP=1

include $(TOPDIR)/rules.mk

LDSCRIPT  = $(TOPDIR)/chip/${CHIP}/arch/flash.ld

