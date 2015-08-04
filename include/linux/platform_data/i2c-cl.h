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

#ifndef __I2C_CL_PLATFORM_H
#define __I2C_CL_PLATFORM_H

/**
 * i2c_cl_platform_data - Platform data for I2C master device on Critical
 *                        Link SOC FPGA module.
 *
 * @master-ref-clk: reference clock used for generating devided SCLK
 * @requested_speed: speed of the I2C clock to use.
 */
struct i2c_cl_platform_data {
	u32	master_ref_clk_hz;
	u32	requested_speed_hz;
};

#endif	/* __I2C_CL_PLATFORM_H */
