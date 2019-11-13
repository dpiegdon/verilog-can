/* vim: colorcolumn=80
 *
 * This file is part of a verilog CAN controller that is SJA1000 compatible.
 *
 * Authors:
 *   * Igor Mohor <igorm@opencores.org>
 *       Author of the original version at
 *       http://www.opencores.org/projects/can/
 *       (which has been unmaintained since about 2009)
 *
 *   * David Piegdon <dgit@piegdon.de>
 *       Picked up project for cleanup and bugfixes in 2019
 *
 * Any additional information is available in the LICENSE file.
 *
 * Copyright (C) 2002, 2003, 2004, 2019 Authors
 *
 * This source file may be used and distributed without restriction provided
 * that this copyright statement is not removed from the file and that any
 * derivative work contains the original copyright notice and the associated
 * disclaimer.
 *
 * This source file is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 *
 * This source is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this source; if not, download it from
 * http://www.opencores.org/lgpl.shtml
 *
 * The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
 * Anybody who wants to implement this CAN IP core on silicon has to obtain
 * a CAN protocol license from Bosch.
 */

`default_nettype none

/*
 * Uncomment following line if you want to use CAN in Actel APA devices
 * (embedded memory used)
 */
//`define   ACTEL_APA_RAM

/*
 * Uncomment following line if you want to use CAN in Altera devices
 * (embedded memory used)
 */
//`define   ALTERA_RAM

/*
 * Uncomment following line if you want to use CAN in Xilinx devices
 * (embedded memory used)
 */
//`define   XILINX_RAM

/*
 * Uncomment the line for the ram used in ASIC implementation
 */
//`define   VIRTUALSILICON_RAM
//`define   ARTISAN_RAM

/*
 * Uncomment the following line when RAM BIST is needed
 * (for ASIC implementations)
 */
//`define CAN_BIST

/*
 * width of MBIST control bus
 */
//`define CAN_MBIST_CTRL_WIDTH 3
