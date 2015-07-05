/*
    This file defines the interface to the lpc22xx SPI module.
    Copyright (C) 2006  Embedded Artists AB (www.embeddedartists.com)

    This file may be included in software not adhering to the GPL.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef SPI_H
#define SPI_H

#include <config.h>
#include <common.h>
#include <asm/errno.h>
#include <asm/arch/hardware.h>

#define spi_lock()   disable_interrupts();
#define spi_unlock() enable_interrupts();

int spi_init(void);

unsigned char spi_read(void);

void spi_write(unsigned char b);

void spi_set_clock(unsigned char clk_value);

void spi_set_cfg(unsigned char phase,
                 unsigned char polarity,
                 unsigned char lsbf);

#define ENC28J60_CS	1

void spi_select(int cs);
void spi_deselect(int cs);

#endif /* SPI_H */
