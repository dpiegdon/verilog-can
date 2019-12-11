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
 * This file contains a basic test fixture for the can controller.
 * Five devices are connected on a can bus:
 * two (1&2) partially share a wishbone bus,
 * and the other three (3&4&5) share a 8051 bus.
 * Tasks are provided to read and write registers of all of these.
 *
 * Just include this file *inside* your testbench module and add the test code
 * and wait(start_test) in your code so the fixture is initialized properly.
 */

`include "testbench/tasks.inc"
`include "testbench/timings.inc"

// to enable logging of all register accesses
//`define LOG_REGISTER_ACCESS 1

// main clock at 16 MHz
reg clk = 0;
initial begin
	forever #31.25 clk = ~clk;
end

// wishbone clock at 10 MHz
reg wb_clk = 0;
initial begin
	forever #50 wb_clk = ~wb_clk;
end

reg reset = 1;


// wishbone bus
reg [7:0] wb_di = 8'hx;
wire [7:0] wb_do1;
wire [7:0] wb_do2;
reg wb_cyc1 = 0;
reg wb_cyc2 = 0;
reg wb_stb = 0;
reg wb_we = 0;
reg [7:0] wb_adr = 8'hx;
wire wb_ack1;
wire wb_ack2;
reg wb_free = 1; // bus-free indicator for wishbone bus

// can device 1
wire dut1_rx;
wire dut1_tx;
wire dut1_irq;
wire dut1_clkout;
wire dut1_bus_off_on;
can_wishbone_top dut1(wb_clk, reset,
	wb_di, wb_do1, wb_cyc1, wb_stb, wb_we, wb_adr, wb_ack1,
	clk,
	dut1_rx, dut1_tx,
	dut1_bus_off_on, dut1_irq, dut1_clkout);

// can device 2
wire dut2_rx;
wire dut2_tx;
wire dut2_irq;
wire dut2_clkout;
wire dut2_bus_off_on;
can_wishbone_top dut2(wb_clk, reset,
	wb_di, wb_do2, wb_cyc2, wb_stb, wb_we, wb_adr, wb_ack2,
	clk,
	dut2_rx, dut2_tx,
	dut2_bus_off_on, dut2_irq, dut2_clkout);


// tristatable 8051 bus
reg t8b_cs3 = 0;
reg t8b_cs4 = 0;
reg t8b_cs5 = 0;
reg t8b_ale = 0;
reg t8b_rd = 0;
reg t8b_wr = 0;
wire [7:0] t8b_port;
reg t8b_free = 1; // bus free indicator for 8051 bus
reg t8b_port_inject_now = 0; // trigger to inject word onto data port
reg [7:0] t8b_port_inject_value = 0; // word to inject onto data port
assign t8b_port = t8b_port_inject_now ? t8b_port_inject_value : 8'hz;

// can device 3
wire dut3_rx;
wire dut3_tx;
wire dut3_irq;
wire dut3_clkout;
wire dut3_bus_off_on;
can_8051_tristate_top dut3(reset,
		t8b_ale, t8b_rd, t8b_wr, t8b_port, t8b_cs3,
		clk,
		dut3_rx, dut3_tx,
		dut3_bus_off_on, dut3_irq, dut3_clkout);

// can device 4
wire dut4_rx;
wire dut4_tx;
wire dut4_irq;
wire dut4_clkout;
wire dut4_bus_off_on;
can_8051_tristate_top dut4(reset,
		t8b_ale, t8b_rd, t8b_wr, t8b_port, t8b_cs4,
		clk,
		dut4_rx, dut4_tx,
		dut4_bus_off_on, dut4_irq, dut4_clkout);

// can device 5
wire dut5_rx;
wire dut5_tx;
wire dut5_irq;
wire dut5_clkout;
wire dut5_bus_off_on;
can_8051_tristate_top dut5(reset,
		t8b_ale, t8b_rd, t8b_wr, t8b_port, t8b_cs5,
		clk,
		dut5_rx, dut5_tx,
		dut5_bus_off_on, dut5_irq, dut5_clkout);


// bus helper tasks
task automatic read_register_wb;
	input integer dut;
	input [7:0] reg_addr;
	output [7:0] reg_data;

	begin
		wait (wb_free);
		wb_free = 0;

		@(posedge wb_clk);
		#1;
		wb_adr = reg_addr;
		if(dut == 1) begin
			wb_cyc1 = 1;
		end else if(dut == 2) begin
			wb_cyc2 = 1;
		end
		wb_stb = 1;
		wb_we = 0;
		if(dut == 1) begin
			wait (wb_ack1);
			reg_data = wb_do1;
		end else if(dut == 2) begin
			wait (wb_ack2);
			reg_data = wb_do2;
		end
		@(posedge wb_clk);
		#1;
		wb_adr = 8'hz;
		if(dut == 1) begin
			wb_cyc1 = 0;
		end else if(dut == 2) begin
			wb_cyc2 = 0;
		end
		wb_stb = 0;
		wb_we = 0;

		wb_free = 1;
	end
endtask

task automatic write_register_wb;
	input integer dut;
	input [7:0] reg_addr;
	input [7:0] reg_data;

	begin
		wait (wb_free);
		wb_free = 0;

		@(posedge wb_clk);
		#1;
		wb_adr = reg_addr;
		wb_di = reg_data;
		if(dut == 1) begin
			wb_cyc1 = 1;
		end else if(dut == 2) begin
			wb_cyc2 = 1;
		end
		wb_stb = 1;
		wb_we = 1;
		if(dut == 1) begin
			wait (wb_ack1);
		end else if(dut == 2) begin
			wait (wb_ack2);
		end
		@(posedge wb_clk);
		#1;
		wb_adr = 8'hx;
		if(dut == 1) begin
			wb_cyc1 = 0;
		end else if(dut == 2) begin
			wb_cyc2 = 0;
		end
		wb_stb = 0;
		wb_we = 0;
		wb_di = 8'hx;

		wb_free = 1;
	end
endtask

task automatic read_register_t8b;
	input integer dut;
	input [7:0] reg_addr;
	output [7:0] reg_data;

	begin
		wait (t8b_free);
		t8b_free = 0;

		@(posedge clk);
		#1;
		if(dut == 3) begin
			t8b_cs3 = 1;
		end else if(dut == 4) begin
			t8b_cs4 = 1;
		end else if(dut == 5) begin
			t8b_cs5 = 1;
		end
		@(negedge clk);
		#1;
		t8b_ale = 1;
		t8b_port_inject_value = reg_addr;
		t8b_port_inject_now = 1;
		@(negedge clk);
		#1;
		t8b_ale = 0;
		@(negedge clk);
		#1;
		t8b_port_inject_now = 0;
		t8b_rd = 1;
		@(negedge clk);
		#1;
		reg_data = t8b_port;
		#1;
		t8b_rd = 0;
		if(dut == 3) begin
			t8b_cs3 = 0;
		end else if(dut == 4) begin
			t8b_cs4 = 0;
		end else if(dut == 5) begin
			t8b_cs5 = 0;
		end

		t8b_free = 1;
	end
endtask

task automatic write_register_t8b;
	input integer dut;
	input [7:0] reg_addr;
	input [7:0] reg_data;

	begin
		wait (t8b_free);
		t8b_free = 0;

		@(posedge clk);
		#1;
		if(dut == 3) begin
			t8b_cs3 = 1;
		end else if(dut == 4) begin
			t8b_cs4 = 1;
		end else if(dut == 5) begin
			t8b_cs5 = 1;
		end
		@(negedge clk);
		#1;
		t8b_ale = 1;
		t8b_port_inject_now = 1;
		t8b_port_inject_value = reg_addr;
		@(negedge clk);
		#1;
		t8b_ale = 0;
		@(negedge clk);
		#1;
		t8b_port_inject_value = reg_data;
		t8b_wr = 1;
		@(negedge clk);
		#1;
		t8b_wr = 0;
		t8b_port_inject_now = 0;
		if(dut == 3) begin
			t8b_cs3 = 0;
		end else if(dut == 4) begin
			t8b_cs4 = 0;
		end else if(dut == 5) begin
			t8b_cs5 = 0;
		end

		t8b_free = 1;
	end
endtask


/*
 * task to read a register via the connected bus.
 * @dut: which device to interface with
 * @reg_addr: address of register to read
 * @reg_data: output value
 */
task automatic read_register;
	input integer dut;
	input [7:0] reg_addr;
	output [7:0] reg_data;

	begin
		if((dut == 1) || (dut == 2)) begin
			read_register_wb(dut, reg_addr, reg_data);
		end else if((dut == 3) || (dut == 4) || (dut == 5)) begin
			read_register_t8b(dut, reg_addr, reg_data);
		end

		`ifdef LOG_REGISTER_ACCESS
			$display("(%012t) DUT%1d GET register %0d == 0x%0x",
				$time, dut, reg_addr, reg_data);
		`endif
	end
endtask

/*
 * task to write a register via the connected bus.
 * @dut: which device to interface with
 * @reg_addr: address of register to write
 * @reg_data: output value
 */
task automatic write_register;
	input integer dut;
	input [7:0] reg_addr;
	input [7:0] reg_data;

	begin
		`ifdef LOG_REGISTER_ACCESS
			$display("(%012t) DUT%1d SET register %0d := 0x%0x",
				$time, dut, reg_addr, reg_data);
		`endif

		if((dut == 1) || (dut == 2)) begin
			write_register_wb(dut, reg_addr, reg_data);
		end else if((dut == 3) || (dut == 4) || (dut == 5)) begin
			write_register_t8b(dut, reg_addr, reg_data);
		end
	end
endtask


// manual bus tap
reg canbus_tap_tx = 1;
wire canbus_tap_rx;


// can bus
simulated_can_bus #(.N(6), .delay(25)) bus(
		{ canbus_tap_tx, dut1_tx, dut2_tx, dut3_tx, dut4_tx, dut5_tx },
		{ canbus_tap_rx, dut1_rx, dut2_rx, dut3_rx, dut4_rx, dut5_rx });


// test trigger
reg start_test = 0;
initial begin
	#250;
	reset = 0;
	#250;
	start_test = 1;
end

