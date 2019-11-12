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

`timescale 1ns/10ps

/* Mode register */
`define CAN_MODE_RESET                  1'h1    /* Reset mode */

/* Bit Timing 0 register value */
//`define CAN_TIMING0_BRP                 6'h0    /* Baud rate prescaler (2*(value+1)) */
//`define CAN_TIMING0_SJW                 2'h2    /* SJW (value+1) */

`define CAN_TIMING0_BRP                 6'h3    /* Baud rate prescaler (2*(value+1)) */
`define CAN_TIMING0_SJW                 2'h1    /* SJW (value+1) */

/* Bit Timing 1 register value */
//`define CAN_TIMING1_TSEG1               4'h4    /* TSEG1 segment (value+1) */
//`define CAN_TIMING1_TSEG2               3'h3    /* TSEG2 segment (value+1) */
//`define CAN_TIMING1_SAM                 1'h0    /* Triple sampling */

`define CAN_TIMING1_TSEG1               4'hf    /* TSEG1 segment (value+1) */
`define CAN_TIMING1_TSEG2               3'h2    /* TSEG2 segment (value+1) */
`define CAN_TIMING1_SAM                 1'h0    /* Triple sampling */
