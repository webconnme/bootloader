#
# (C) Copyright 2010
# jung hyun kim, Nexell Co, <jhkim@nexell.co.kr>
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk
include libproto.mk

DEVPATH  = devices
LIBDEV   = $(obj)lib$(SOC)-devices.o

COMPATH  = common
LIBCOM   = $(obj)lib$(SOC)-common.o

LIB		 = $(obj)lib$(SOC).a

COBJS-y	+= cpu.o
COBJS-y	+= nxp2120.o
COBJS-y	+= reset.o
COBJS-y	+= timer.o

COBJS-$(CONFIG_PM_WAKEUP) 	+= wakeup.o
SOBJS-$(CONFIG_PM_WAKEUP) 	+= pm.o

SRCS    := $(SOBJS-y:.o=.S) $(COBJS-y:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS-y) $(SOBJS-y))

all:	$(obj).depend $(LIB)

LIBDEVOBJS = $(shell $(AR) t  $(DEVPATH)/$(LIBDEV))
COMDEVOBJS = $(shell $(AR) t  $(COMPATH)/$(LIBCOM))

$(LIB):	$(obj).depend $(LIBDEV) $(LIBCOM) $(OBJS)
	$(AR) x $(DEVPATH)/$(LIBDEV)
	$(AR) $(ARFLAGS) $@ $(OBJS) $(LIBDEVOBJS)
	rm $(LIBDEVOBJS)
	$(AR) x $(COMPATH)/$(LIBCOM)
	$(AR) $(ARFLAGS) $@ $(OBJS) $(COMDEVOBJS)
	rm $(COMDEVOBJS)

$(LIBDEV):
	$(MAKE) -C $(DEVPATH)

$(LIBCOM):
	$(MAKE) -C $(COMPATH)

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
