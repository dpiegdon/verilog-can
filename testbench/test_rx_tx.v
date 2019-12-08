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

module test_tx_rx(output reg finished, output reg [15:0] errors);
	/*
	 * test transmit and receive of extended-id can frames on a bus.
	 * also tests basic data bus connectivity for both wishbone
	 * and 8051 bus types.
	 */

	`include "testbench/fixture.inc"

	localparam baudrate_prescaler = 6'h0;
	localparam sync_jump_width = 2'h1;
	localparam tseg1 = 4'hf;
	localparam tseg2 = 3'h2;
	localparam bitclocks = clocks_per_bit(baudrate_prescaler, tseg2, tseg1);

	localparam triple_sampling = 0;
	localparam extended_mode = 1;
	localparam remote_transmission_request = 0;

	integer i = 0;
	integer countdown = 0;
	integer dut_sender = 0;
	integer dut_receiver = 0;

	integer value = 0;
	integer expect = 0;

	reg [28:0] tx_id;
	reg [3:0] tx_data_length;
	reg [63:0] tx_data;
	reg [1:0] rx_errors;

	initial begin
		finished = 0;
		errors = 0;
		wait(start_test);

		for(i=1; i<=4; i=i+1) begin
			setup_device(i,
				sync_jump_width, baudrate_prescaler,
				triple_sampling, tseg2, tseg1, extended_mode);
		end

		// BRP clocks are needed before we can do work with the bus.
		repeat (8 * bitclocks) @(posedge clk);

		// send from all DUTs and check that all other DUTs received.
		for(dut_sender=1; dut_sender<=4; dut_sender=dut_sender+1) begin
			// make sure bus is idle
			if(1!=canbus_tap_rx) begin
				errors += 1;
				$error("CAN bus should be idle but is not.");
			end

			// transmit frame from one device
			tx_id = 28'h0123456+dut_sender;
			tx_data_length = 8;
			tx_data = 64'hdead_beef_badc_0ffe+dut_sender;
			send_frame(dut_sender, remote_transmission_request, extended_mode, tx_data_length, tx_id, tx_data);

			// check device actually starts a transmit
			countdown = 2 * bitclocks;
			while((countdown!=0) && (canbus_tap_rx!=0)) begin
				@(posedge clk);
				countdown = countdown-1;
			end
			if(0==countdown) begin
				errors += 1;
				$error("DUT%d did not send data on can bus.", dut_sender);
			end

			// check tx-complete interrupt
			countdown = (maximum_bits_between_ack_and_interrupt + maximum_bits_per_stuffed_extended_frame) * bitclocks;
			fork;
				begin : timeout_check
					repeat (countdown) @(posedge clk);
					errors += 1;
					$error("DUT%d TX timed out", dut_sender);
					disable tx_check;
				end
				begin : tx_check
					wait4irq(dut_sender);
					get_interrupt_register(dut_sender, value);
					expect = 8'h02;
					if(value != expect) begin
						errors += 1;
						$error("DUT%d interrupt should be 'TX complete' (0x%02X) but is 0x%02X",
							dut_sender, expect, value);
					end
					disable timeout_check;
				end
			join

			// check reception on other devices
			repeat (100) @(posedge clk);
			for(dut_receiver=1; dut_receiver<=4; dut_receiver=dut_receiver+1) begin
				if(dut_receiver != dut_sender) begin
					get_interrupt_register(dut_receiver, value);
					expect = 8'h01;
					if(value != expect) begin
						errors += 1;
						$error("DUT%d interrupt should be 'RX complete' (0x%02X) but is 0x%02X",
							dut_receiver, expect, value);
					end

					if(dut_receiver==1) begin
						value = `rx_fifo_frames(dut1);
					end else if(dut_receiver==2) begin
						value = `rx_fifo_frames(dut2);
					end else if(dut_receiver==3) begin
						value = `rx_fifo_frames(dut3.can_8051);
					end else if(dut_receiver==4) begin
						value = `rx_fifo_frames(dut4.can_8051);
					end

					expect = 1;
					if(value == expect) begin
						receive_and_verify_frame(dut_receiver, remote_transmission_request, extended_mode, tx_id, tx_data_length, tx_data, rx_errors);
						errors = errors + rx_errors;
					end else begin
						errors += 1;
						$error("DUT%d has %d frames in fifo, but there should be %d.",
							dut_receiver, value, expect);
					end
				end
			end

			// check that interrupts have not reappeared.
			for(i = 1; i <= 4; i=i+1) begin
				get_interrupt_register(i, value);
				expect = 8'h0;
				if(value != expect) begin
					errors += 1;
					$error("DUT%d interrupt should be 'NONE' (0x%02X) but is 0x%02X",
						i, expect, value);
				end
			end
			repeat (100) @(posedge clk);

		end

		finished = 1;
	end

endmodule

