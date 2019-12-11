/* vim: colorcolumn=80
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

`default_nettype none

module test_acceptance_filter(output reg finished, output reg [15:0] errors);
	/*
	 * test acceptance filter when receiving frames.
	 */

	`include "testbench/fixture.inc"

	localparam baudrate_prescaler = 6'h0;
	localparam sync_jump_width = 2'h1;
	localparam tseg1 = 4'h0;
	localparam tseg2 = 3'h0;
	localparam bitclocks = clocks_per_bit(baudrate_prescaler, tseg2, tseg1);

	localparam triple_sampling = 0;
	localparam extended_mode = 1;
	localparam remote_transmission_request = 0;

	reg [31:0] filter_id;
	reg [31:0] filter_mask;

	initial begin
		errors = 0;
		finished = 0;
		wait(start_test);

		// transmitting device
		setup_device(1, sync_jump_width, baudrate_prescaler,
			triple_sampling, tseg2, tseg1, 1);

		// basic mode, single short filter:
		// filter checks against id[10:3]
		setup_device_with_filters(2, sync_jump_width, baudrate_prescaler,
			triple_sampling, tseg2, tseg1, 0,
			0,
			filter_id[ : ], 8'h00, 8'h00, 8'h00,
			filter_mask[ : ], 8'h00, 8'h00, 8'h00,
		);

		// extended mode, single long filter
		setup_device_with_filters(3, sync_jump_width, baudrate_prescaler,
			triple_sampling, tseg2, tseg1, 1,
			1,
			:
			:
		);

		// extended mode, double short filter, first slot
		setup_device_with_filters(4, sync_jump_width, baudrate_prescaler,
			triple_sampling, tseg2, tseg1, 1,
			0,
			:
			:
		);

		// extended mode, double short filter, second slot
		setup_device_with_filters(5, sync_jump_width, baudrate_prescaler,
			triple_sampling, tseg2, tseg1, 1,
			0,
			:
			:
		);

		$warning("FIXME: implement test_acceptance_filter()");
		errors = 1;
		finished = 1;
	end
endmodule

