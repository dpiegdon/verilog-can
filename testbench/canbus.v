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

module simulated_can_bus(input wire [N-1:0] tx, output wire [N-1:0] rx);
	/*
	 * tx bit '1' equals to 0V = can_hi - can_low (recessive)
	 * tx bit '0' equals to 5V = can_hi - can_low (dominant)
	 *
	 * this is a simple CAN bus simulator that merges N devices on a bus,
	 * where each device has a transmission delay of #delay from to each
	 * other device.
	 */

	parameter N=2; // number of devices on bus
	parameter delay=0; // delay from any one to another node

	wire [N-1:0] delayed_tx;
	wand [N-1:0] merged_tx;

	generate
		genvar rxn;
		genvar txn;
		for(rxn = 0; rxn < N; rxn=rxn+1) begin

			assign #delay delayed_tx[rxn] = tx[rxn];

			for(txn = 0; txn < N; txn=txn+1) begin
				if(rxn == txn) begin
					assign merged_tx[rxn] = tx[txn];
				end else begin
					assign merged_tx[rxn] = delayed_tx[txn];
				end
			end

			assign rx[rxn] = merged_tx[rxn];
		end
	endgenerate

endmodule

