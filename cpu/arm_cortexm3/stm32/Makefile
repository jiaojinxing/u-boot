#
# (C) Copyright 2000-2003
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# (C) Copyright 2011
# Port to STM32
# Yuri Tikhonov, Emcraft Systems, yur@emcraft.com.
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

LIB	= $(obj)lib$(SOC).a

COBJS	:= clock.o cpu.o envm.o wdt.o fsmc.o \
Src/stm32f4xx_hal.o \
Src/stm32f4xx_hal_adc.o \
Src/stm32f4xx_hal_adc_ex.o \
Src/stm32f4xx_hal_can.o \
Src/stm32f4xx_hal_cec.o \
Src/stm32f4xx_hal_cortex.o \
Src/stm32f4xx_hal_crc.o \
Src/stm32f4xx_hal_cryp.o \
Src/stm32f4xx_hal_cryp_ex.o \
Src/stm32f4xx_hal_dac.o \
Src/stm32f4xx_hal_dac_ex.o \
Src/stm32f4xx_hal_dcmi.o \
Src/stm32f4xx_hal_dcmi_ex.o \
Src/stm32f4xx_hal_dma.o \
Src/stm32f4xx_hal_dma2d.o \
Src/stm32f4xx_hal_dma_ex.o \
Src/stm32f4xx_hal_eth.o \
Src/stm32f4xx_hal_flash.o \
Src/stm32f4xx_hal_flash_ex.o \
Src/stm32f4xx_hal_flash_ramfunc.o \
Src/stm32f4xx_hal_fmpi2c.o \
Src/stm32f4xx_hal_fmpi2c_ex.o \
Src/stm32f4xx_hal_gpio.o \
Src/stm32f4xx_hal_hash.o \
Src/stm32f4xx_hal_hash_ex.o \
Src/stm32f4xx_hal_hcd.o \
Src/stm32f4xx_hal_i2c.o \
Src/stm32f4xx_hal_i2c_ex.o \
Src/stm32f4xx_hal_i2s.o \
Src/stm32f4xx_hal_i2s_ex.o \
Src/stm32f4xx_hal_irda.o \
Src/stm32f4xx_hal_iwdg.o \
Src/stm32f4xx_hal_ltdc.o \
Src/stm32f4xx_hal_msp_template.o \
Src/stm32f4xx_hal_nand.o \
Src/stm32f4xx_hal_nor.o \
Src/stm32f4xx_hal_pccard.o \
Src/stm32f4xx_hal_pcd.o \
Src/stm32f4xx_hal_pcd_ex.o \
Src/stm32f4xx_hal_pwr.o \
Src/stm32f4xx_hal_pwr_ex.o \
Src/stm32f4xx_hal_qspi.o \
Src/stm32f4xx_hal_rcc.o \
Src/stm32f4xx_hal_rcc_ex.o \
Src/stm32f4xx_hal_rng.o \
Src/stm32f4xx_hal_rtc.o \
Src/stm32f4xx_hal_rtc_ex.o \
Src/stm32f4xx_hal_sai.o \
Src/stm32f4xx_hal_sai_ex.o \
Src/stm32f4xx_hal_sd.o \
Src/stm32f4xx_hal_sdram.o \
Src/stm32f4xx_hal_smartcard.o \
Src/stm32f4xx_hal_spdifrx.o \
Src/stm32f4xx_hal_spi.o \
Src/stm32f4xx_hal_sram.o \
Src/stm32f4xx_hal_tim.o \
Src/stm32f4xx_hal_tim_ex.o \
Src/stm32f4xx_hal_uart.o \
Src/stm32f4xx_hal_usart.o \
Src/stm32f4xx_hal_wwdg.o \
Src/stm32f4xx_ll_fmc.o \
Src/stm32f4xx_ll_fsmc.o \
Src/stm32f4xx_ll_sdmmc.o \
Src/stm32f4xx_ll_usb.o \
Src/system_stm32f4xx.o

ifeq ($(CONFIG_CMD_BUFCOPY),y)
COBJS	+= cmd_bufcopy.o
endif
SOBJS	:=

SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))
SOBJS	:= $(addprefix $(obj),$(SOBJS))

$(LIB):	$(obj).depend $(OBJS) $(SOBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

clean:
	rm -f $(SOBJS) $(OBJS)

distclean:
	rm -f $(LIB) core *.bak $(obj).depend

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
