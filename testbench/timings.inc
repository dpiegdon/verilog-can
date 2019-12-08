/* vim: colorcolumn=80 syntax=verilog
 *
 * This file is part of a verilog CAN controller that is SJA1000 compatible.
 *
 * Authors:
 *   * David Piegdon <dgit@piegdon.de>
 *       Picked up project for cleanup and bugfixes in 2019
 *
 * Any additional information is available in the LICENSE file.
 *
 * Copyright (C) 2019 Authors
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

/*
 * This file defines timing constants.
 *
 * Just include this file *inside* your testbench module and add the test code.
 */

// maximum frame lengths
localparam maximum_bits_per_basic_frame = 1+11+1+2+4+8*8+15+1+2+7;
localparam maximum_bits_per_extended_frame = 1+29+1+2+4+8*8+15+1+2+7;
localparam maximum_bits_per_stuffed_basic_frame = maximum_bits_per_basic_frame + maximum_bits_per_basic_frame/5;
localparam maximum_bits_per_stuffed_extended_frame = maximum_bits_per_extended_frame + maximum_bits_per_extended_frame/5;

// maximum time (in bits) between an acknowledge and the corresponding interrupt being triggered
localparam maximum_bits_between_ack_and_interrupt = 9;

// this yields the length of a single bit on the CAN bus, as
// configured by the given parameters, in terms of a single master clocks
function automatic [15:0] clocks_per_bit(input [5:0] baudrate_prescaler, input [2:0] tseg2, input [3:0] tseg1);
	begin
		clocks_per_bit = 2 * (baudrate_prescaler+1) * (1 + tseg1+1 + tseg2+1);
	end
endfunction

