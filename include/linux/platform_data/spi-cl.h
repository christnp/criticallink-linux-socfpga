/*
 * Copyright 2014 Critical Link LLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __SPI_CL_PLATFORM_H
#define __SPI_CL_PLATFORM_H

/**
 * spi_cl_platform_data - Platform data for SPI master device on Critical
 *                        Link SOC FPGA module.
 *
 * @num_chipselect: number of chipselects supported by this SPI master
 * @master-ref-clk: reference clock used for generating devided SCLK
 * @bus-num: bus number to use.
 */
struct spi_cl_platform_data {
	u8	num_chipselect;
	u32	master_ref_clk_hz;
	u8	bus_num;
};

#endif	/* __SPI_CL_PLATFORM_H */
