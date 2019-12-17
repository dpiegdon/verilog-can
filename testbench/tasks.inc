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
 * This file defines helper tasks for the can controller.
 *
 * Just include this file *inside* your testbench module and add the test code.
 */

// Forcing bus-off by writinf to tx_err_cnt register
task automatic forced_bus_off(input integer dut);
	begin
		// Switch-on reset mode
		write_register(dut, 8'd0, {7'h0, 1'h1});
		// Set Clock Divider register
		write_register(dut, 8'd31, {1'b1, 7'h0});    // Setting the extended mode (not normal)
		// Write 255 to tx_err_cnt register - Forcing bus-off
		write_register(dut, 8'd15, 255);
		// Switch-off reset mode
		write_register(dut, 8'd0, {7'h0, ~(1'h1)});

		//    #1000000;
		#2500000;

		// Switch-on reset mode
		write_register(dut, 8'd0, {7'h0, 1'h1});
		// Write 245 to tx_err_cnt register
		write_register(dut, 8'd15, 245);
		// Switch-off reset mode
		write_register(dut, 8'd0, {7'h0, ~(1'h1)});

		#1000000;
	end
endtask

// Return number of bytes in RX fifo.
// Not working when wr_info_pointer is smaller than rd_info_pointer.
`define rx_fifo_bytes(dut_interna) \
	(dut_interna.can_controller.i_can_bsp.i_can_fifo.fifo_cnt)

// Return number of frames in RX fifo.
// Not working when wr_info_pointer is smaller than rd_info_pointer.
`define rx_fifo_frames(dut_interna) \
	(dut_interna.can_controller.i_can_bsp.i_can_fifo.wr_info_pointer - \
		dut_interna.can_controller.i_can_bsp.i_can_fifo.rd_info_pointer)

function automatic integer get_rx_fifo_framecount(input integer dut);
	begin
		if(dut==1) begin
			get_rx_fifo_framecount = `rx_fifo_frames(dut1);
		end else if(dut==2) begin
			get_rx_fifo_framecount = `rx_fifo_frames(dut2);
		end else if(dut==3) begin
			get_rx_fifo_framecount = `rx_fifo_frames(dut3.can_8051);
		end else if(dut==4) begin
			get_rx_fifo_framecount = `rx_fifo_frames(dut4.can_8051);
		end else if(dut==5) begin
			get_rx_fifo_framecount = `rx_fifo_frames(dut5.can_8051);
		end
	end
endfunction

// Wait for interrupt on specified device. Returns immediately if IRQ active.
task automatic wait4irq(input integer dut);
	begin
		if(dut==1) begin
			if(dut1_irq) begin
				@(negedge dut1_irq);
			end
		end else if(dut==2) begin
			if(dut2_irq) begin
				@(negedge dut2_irq);
			end
		end else if(dut==3) begin
			if(dut3_irq) begin
				@(negedge dut3_irq);
			end
		end else if(dut==4) begin
			if(dut4_irq) begin
				@(negedge dut4_irq);
			end
		end else if(dut==5) begin
			if(dut5_irq) begin
				@(negedge dut5_irq);
			end
		end
	end
endtask

// Return value of interrupt line of dut
function automatic irqline(input integer dut);
	begin
		if(dut==1) begin
			irqline = dut1_irq;
		end else if(dut==2) begin
			irqline = dut2_irq;
		end else if(dut==3) begin
			irqline = dut3_irq;
		end else if(dut==4) begin
			irqline = dut4_irq;
		end else if(dut==5) begin
			irqline = dut5_irq;
		end
	end
endfunction

task automatic set_timing_registers(input integer dut,
	input [1:0] sync_jump_width, input [5:0] baudrate_prescaler,
	input triple_sampling, input [2:0] tseg2, input [3:0] tseg1);

	begin
		// BRT0
		write_register(dut, 8'd6, {sync_jump_width, baudrate_prescaler});
		// BTR1
		write_register(dut, 8'd7, {triple_sampling, tseg2, tseg1});
	end
endtask

task automatic set_clockdivider_register(input integer dut,
	input extended_mode, // (PeliCAN mode)
	// CBP is always 0
	// RXINTEN is always 0
	// BIT4 is always 0
	input clock_off,
	input [2:0] clock_div);

	begin
		write_register(dut, 8'd31, {
				extended_mode, 3'b000,
				clock_off, clock_div});
	end
endtask

task automatic set_mode_register(input integer dut,
	input sleep, acceptance_filter, self_test, listen_only, reset);
	// (only sensible for extended mode / PeliCAN mode)

	begin
		write_register(dut, 8'd0, {
				3'b000, sleep,
				acceptance_filter, self_test,
				listen_only, reset});
	end
endtask

task automatic set_interrupt_enable_register(input integer dut,
	input bus_error, arbitration_lost, error_passive, wake_up, data_overrun,
	error_warning, transmitted, received);

	begin
		write_register(dut, 8'd4, {
				bus_error, arbitration_lost,
				error_passive, wake_up,
				data_overrun, error_warning,
				transmitted, received});
	end
endtask

task automatic get_interrupt_register(input integer dut, output [7:0] interrupts);
	begin
		read_register(dut, 8'd3, interrupts);
	end
endtask

task automatic set_command_register(input integer dut, input [7:0] commands);
	begin
		write_register(dut, 8'd1, commands);
	end
endtask

task automatic setup_device_with_filters(
	input integer dut, 
	input [1:0] sync_jump_width,
	input [5:0] baudrate_prescaler,
	input triple_sampling,
	input [2:0] tseg2,
	input [3:0] tseg1,
	input extended_mode,
	input acceptance_filter_mode,
	input [7:0] acode0, acode1, acode2, acode3,
	input [7:0] amask0, amask1, amask2, amask3);

	reg [7:0] tmp;
	begin
		// force into reset mode
		set_mode_register(dut, 0, acceptance_filter_mode, 0, 0, 1);

		set_clockdivider_register(dut, extended_mode, 0, 0);

		set_timing_registers(dut, sync_jump_width, baudrate_prescaler,
			triple_sampling, tseg2, tseg1);

		// send all interrupts
		set_interrupt_enable_register(dut, 1,1,1,1,1,1,1,1);

		// set acceptance filter to accept anything
		if(extended_mode) begin
			write_register(dut, 8'd16, acode0);
			write_register(dut, 8'd17, acode1);
			write_register(dut, 8'd18, acode2);
			write_register(dut, 8'd19, acode3);
			write_register(dut, 8'd20, amask0);
			write_register(dut, 8'd21, amask1);
			write_register(dut, 8'd22, amask2);
			write_register(dut, 8'd23, amask3);
		end else begin
			write_register(dut, 8'd04, acode0);
			write_register(dut, 8'd05, amask0);
		end

		// clear error counter
		write_register(dut, 8'd14, 8'h00); // RX error counter
		write_register(dut, 8'd15, 8'h00); // TX error counter

		// clear error code capture register
		read_register(dut, 8'd12, tmp);

		// clear any interrupts
		get_interrupt_register(dut, tmp);

		// leave reset mode
		set_mode_register(dut, 0, acceptance_filter_mode, 0, 0, 0);
	end
endtask

task automatic setup_device(
	input integer dut, 
	input [1:0] sync_jump_width,
	input [5:0] baudrate_prescaler,
	input triple_sampling,
	input [2:0] tseg2,
	input [3:0] tseg1,
	input extended_mode);
	begin
		setup_device_with_filters(dut, sync_jump_width, baudrate_prescaler,
			triple_sampling, tseg2, tseg1, extended_mode,
			0,
			8'h00, 8'h00, 8'h00, 8'h00, 
			8'hff, 8'hff, 8'hff, 8'hff);
	end
endtask

task automatic send_frame(input integer dut,
	input remote_transmission_request,
	input extended_format,
	input [3:0] data_length,
	input [28:0] id,
	input [63:0] data);

	reg [63:0] data_dlc;

	begin
		write_register(dut, 8'd16, {
				extended_format, remote_transmission_request,
				2'b00,
				data_length});

		data_dlc = data << (8-data_length);

		if(extended_format) begin
			write_register(dut, 8'd17, id[28:21]);
			write_register(dut, 8'd18, id[20:13]);
			write_register(dut, 8'd19, id[12:5]);
			write_register(dut, 8'd20, {id[4:0], 3'b000});
			write_register(dut, 8'd21, data_dlc[63:56]);
			write_register(dut, 8'd22, data_dlc[55:48]);
			write_register(dut, 8'd23, data_dlc[47:40]);
			write_register(dut, 8'd24, data_dlc[39:32]);
			write_register(dut, 8'd25, data_dlc[31:24]);
			write_register(dut, 8'd26, data_dlc[23:16]);
			write_register(dut, 8'd27, data_dlc[15:8]);
			write_register(dut, 8'd28, data_dlc[7:0]);
		end else begin
			write_register(dut, 8'd17, id[10:3]);
			write_register(dut, 8'd18, {id[2:0], 5'b00000});
			write_register(dut, 8'd19, data_dlc[63:56]);
			write_register(dut, 8'd20, data_dlc[55:48]);
			write_register(dut, 8'd21, data_dlc[47:40]);
			write_register(dut, 8'd22, data_dlc[39:32]);
			write_register(dut, 8'd23, data_dlc[31:24]);
			write_register(dut, 8'd24, data_dlc[23:16]);
			write_register(dut, 8'd25, data_dlc[15:8]);
			write_register(dut, 8'd26, data_dlc[7:0]);
		end

		set_command_register(dut, 8'h01); // start transmit
	end
endtask

// verify that transmission on CAN bus has started
// uses upstream `countdown` `bitclocks` `canbus_tap_rx`
task automatic verify_transmit_started(dut);
	begin
		// check device actually starts a transmit
		countdown = 2 * bitclocks;
		while((countdown!=0) && (canbus_tap_rx!=0)) begin
			@(posedge clk);
		end
		if(0==countdown) begin
			errors += 1;
			$error("DUT%d did not send data on can bus.", dut);
		end else begin
			countdown = 0;
		end
	end
endtask

// verify that transmission starts on CANbus, device triggers an interrupt,
// and only the tx-complete interrupt is set.
task automatic verify_transmit_finished(dut);
	begin
		verify_transmit_started(dut);

		// check tx-complete interrupt
		countdown = (maximum_bits_between_ack_and_interrupt + maximum_bits_per_stuffed_extended_frame) * bitclocks;
		while((countdown != 0) && (irqline(dut_sender) != 0)) begin
			@(posedge clk);
		end
		if(0 == countdown) begin
			$error("DUT%d TX timeout", dut_sender);
		end else begin
			countdown = 0;
			get_interrupt_register(dut_sender, value);
			expect = 8'h02;
			if(value != expect) begin
				errors += 1;
				$error("DUT%d interrupt should be 'TX complete' (0x%02X) but is 0x%02X",
					dut_sender, expect, value);
			end
		end
	end
endtask


task automatic release_receive_buffer(dut);
	begin
		set_command_register(dut, 8'h04);
	end
endtask


task automatic receive_frame(input integer dut,
	output remote_transmission_request,
	output received_extended_id,
	output [28:0] received_id,
	output [3:0] received_data_length,
	output [63:0] received_data);

	// NOTE this does not check if there actually is a frame queued in the buffer.
	// you need to do that before calling.

	reg [7:0] value;
	integer datareg_off;
	integer i;
	integer databyte;

	begin
		read_register(dut, 8'd16, value);
		received_extended_id = value[7];
		received_data_length = value[3:0];
		if(received_data_length[3] & |(received_data_length[2:0])) begin
			$error("insane DLC while receiving frame: %d. truncating to 8.\n", received_data_length);
			//received_data_length = 8;
		end

		received_id = 0;
		if(received_extended_id) begin
			read_register(dut, 8'd17, value);
			received_id[28:21] = value;
			read_register(dut, 8'd18, value);
			received_id[20:13] = value;
			read_register(dut, 8'd19, value);
			received_id[12:5] = value;
			read_register(dut, 8'd20, value);
			received_id[4:0] = value[7:3];
			datareg_off = 21;
		end else begin
			read_register(dut, 8'd17, value);
			received_id[10:3] = value;
			read_register(dut, 8'd18, value);
			received_id[2:0] = value[7:5];
			datareg_off = 19;
		end

		received_data = 0;
		for(i = 0; i < received_data_length; i=i+1) begin
			read_register(dut, datareg_off + i, value);
			// circumvent use of variable array index...
			databyte = received_data_length - i - 1;
			if(7 == databyte) begin
				received_data[63:56] = value;
			end else if(6 == databyte) begin
				received_data[55:48] = value;
			end else if(5 == databyte) begin
				received_data[47:40] = value;
			end else if(4 == databyte) begin
				received_data[39:32] = value;
			end else if(3 == databyte) begin
				received_data[31:24] = value;
			end else if(2 == databyte) begin
				received_data[23:16] = value;
			end else if(1 == databyte) begin
				received_data[15:8] = value;
			end else if(0 == databyte) begin
				received_data[7:0] = value;
			end
		end

		release_receive_buffer(dut);
	end
endtask

task automatic receive_and_verify_frame(input integer dut,
	input expected_remote_transmission_request,
	input expected_extended_id,
	input [28:0] expected_id,
	input [3:0] expected_data_length,
	input [63:0] expected_data,
	output [1:0] errors);

	reg received_remote_transmission_request;
	reg received_extended_id;
	reg [28:0] received_id;
	reg [3:0] received_data_length;
	reg [63:0] received_data;

	begin
		errors = 2'h0;

		receive_frame(dut, received_remote_transmission_request, received_extended_id, received_id, received_data_length, received_data);

		if(received_remote_transmission_request != expected_remote_transmission_request) begin
			$error("DUT%d RTR flag: got %d should be %d",
				dut, received_remote_transmission_request, expected_remote_transmission_request);
			errors = errors + 1;
		end

		if(received_extended_id != expected_extended_id) begin
			$error("DUT%d extended ID flag: got %d should be %d",
				dut, received_extended_id, expected_extended_id);
			errors = errors + 1;
		end

		if(received_id != expected_id) begin
			$error("DUT%d ID: got 0x%08x should be 0x%08x\n",
				dut, received_id, expected_id);
			errors = errors + 1;
		end

		if(received_data_length != expected_data_length) begin
			$error("DUT%d DLC: got %d should be %d\n",
				dut, received_data_length, expected_data_length);
			errors = errors + 1;
		end

		if(received_data != expected_data) begin
			$error("DUT%d data: got 0x%016x should be 0x%016x\n",
				dut, received_data, expected_data);
			errors = errors + 1;
		end
	end
endtask
