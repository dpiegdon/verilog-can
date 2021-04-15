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
	 * primary goals:
	 *	test acceptance filter when receiving frames.
	 * secondary goals:
	 *	test basic frame transmission and receiption,
	 *	test controller in basic mode (dut 2),
	 *	test transmit/receive of different frames length.
	 */

	`include "testbench/fixture.inc"

	task automatic setup_devices(input reg [31:0] filter_id, input reg [31:0] filter_mask);
		begin
			// transmitting device
			setup_device(1, sync_jump_width, baudrate_prescaler,
				triple_sampling, tseg2, tseg1, 1);

			// basic mode, single short filter:
			// filter checks against id[10:3]
			setup_device_with_filters(2, sync_jump_width, baudrate_prescaler,
				triple_sampling, tseg2, tseg1, 0,
				0,
				filter_id[10:3], 8'h00, 8'h00, 8'h00,
				filter_mask[10:3], 8'h00, 8'h00, 8'h00
			);

			// extended mode, single long filter
			setup_device_with_filters(3, sync_jump_width, baudrate_prescaler,
				triple_sampling, tseg2, tseg1, 1,
				1,
				filter_id[31:24], filter_id[23:16], filter_id[15:8], filter_id[7:0],
				filter_mask[31:24], filter_mask[23:16], filter_mask[15:8], filter_mask[7:0]
			);

			// extended mode, double short filter, first slot
			setup_device_with_filters(4, sync_jump_width, baudrate_prescaler,
				triple_sampling, tseg2, tseg1, 1,
				0,
				filter_id[31:24], filter_id[23:16], 8'h00, 8'h00,
				filter_mask[31:24], filter_mask[23:16], 8'h00, 8'h00
			);

			// extended mode, double short filter, second slot
			setup_device_with_filters(5, sync_jump_width, baudrate_prescaler,
				triple_sampling, tseg2, tseg1, 1,
				0,
				8'h00, 8'h00, filter_id[31:24], filter_id[23:16],
				8'h00, 8'h00, filter_mask[31:24], filter_mask[23:16]
			);

			// BRP clocks are needed before we can do work with the bus.
			repeat (8 * bitclocks) @(posedge clk);
		end
	endtask

	task automatic receiption_ok(input integer dut_receiver);
		begin
			if(irqline(dut_receiver) != 0) begin
				errors += 1;
				$error("DUT%d did not trigger interrupt line", dut_receiver);
			end

			get_interrupt_register(dut_receiver, value);
			if(dut_receiver == 2) begin
				expect = 8'he1; // basic mode controller has always 0xe0 set.
			end else begin
				expect = 8'h01;
			end
			if(value != expect) begin
				errors += 1;
				$error("DUT%d interrupt should be 'RX complete' (0x%02X) but is 0x%02X",
					dut_receiver, expect, value);
			end

			value = get_rx_fifo_framecount(dut_receiver);
			expect = 1;
			if(value == expect) begin
				release_receive_buffer(dut_receiver);
			end else begin
				errors += 1;
				$error("DUT%d has %d frames in fifo, but there should be %d.",
					dut_receiver, value, expect);
			end

			repeat (5) @(posedge clk);

			expect = 0;
			value = get_rx_fifo_framecount(dut_receiver);
			if(value != 0) begin
				errors += 1;
				$error("DUT%d did not properly release buffer. Has %d in queue, should be %d",
					dut_receiver, value, expect);
			end
		end
	endtask

	localparam baudrate_prescaler = 6'h0;
	localparam sync_jump_width = 2'h1;
	localparam tseg1 = 4'h1;
	localparam tseg2 = 3'h0;
	localparam bitclocks = clocks_per_bit(baudrate_prescaler, tseg2, tseg1);

	localparam triple_sampling = 0;
	localparam extended_mode = 1;
	localparam remote_transmission_request = 0;

	integer dut_sender = 1;
	integer dut_receiver;
	integer value = 0;
	integer expect = 0;

	reg [31:0] filter_id;
	reg [31:0] bad_id;
	reg [31:0] filter_mask;
	reg [63:0] payload;
	reg [3:0] dlc;

	initial begin
		errors = 0;
		finished = 0;
		wait(start_test);

		filter_id = 32'h00000123;
		filter_mask = 32'h00000000;
		payload = 64'h0000_0000_0000_0000;
		dlc = 0;
		setup_devices(filter_id, filter_mask);
		send_frame(dut_sender, 0, 0, dlc, filter_id[28:0], payload);
		verify_transmit_finished(dut_sender);

		// check reception on other devices
		repeat (20) @(posedge clk);
		for(dut_receiver=2; dut_receiver<=5; dut_receiver=dut_receiver+1) begin
			receiption_ok(dut_receiver);
		end



		// FIXME
		finished = 1;
	end
endmodule

