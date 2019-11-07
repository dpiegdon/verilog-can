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

`include "can_top_defines.v"

module can_acf(
	input wire clk,
	input wire rst,
	input wire [28:0] id,

	/* Mode register */
	input wire reset_mode,
	input wire acceptance_filter_mode,
	input wire extended_mode,

	input wire [7:0] acceptance_code_0,
	input wire [7:0] acceptance_code_1,
	input wire [7:0] acceptance_code_2,
	input wire [7:0] acceptance_code_3,
	input wire [7:0] acceptance_mask_0,
	input wire [7:0] acceptance_mask_1,
	input wire [7:0] acceptance_mask_2,
	input wire [7:0] acceptance_mask_3,

	input wire go_rx_crc_lim,
	input wire go_rx_inter,
	input wire go_error_frame,

	input wire [7:0] data0,
	input wire [7:0] data1,
	input wire rtr1,
	input wire rtr2,
	input wire ide,
	input wire no_byte0,
	input wire no_byte1,

	output reg id_ok
	);

	parameter Tp = 1;


	wire          match;
	wire          match_sf_std;
	wire          match_sf_ext;
	wire          match_df_std;
	wire          match_df_ext;

	// Working in basic mode. ID match for standard format (11-bit ID).
	assign match =
		(	(id[3]  == acceptance_code_0[0] | acceptance_mask_0[0] )
		&	(id[4]  == acceptance_code_0[1] | acceptance_mask_0[1] )
		&	(id[5]  == acceptance_code_0[2] | acceptance_mask_0[2] )
		&	(id[6]  == acceptance_code_0[3] | acceptance_mask_0[3] )
		&	(id[7]  == acceptance_code_0[4] | acceptance_mask_0[4] )
		&	(id[8]  == acceptance_code_0[5] | acceptance_mask_0[5] )
		&	(id[9]  == acceptance_code_0[6] | acceptance_mask_0[6] )
		&	(id[10] == acceptance_code_0[7] | acceptance_mask_0[7] )
		);

// Working in extended mode. ID match for standard format (11-bit ID). Using single filter.
	assign match_sf_std =
		(	(id[3]  == acceptance_code_0[0] | acceptance_mask_0[0] )
		&	(id[4]  == acceptance_code_0[1] | acceptance_mask_0[1] )
		&	(id[5]  == acceptance_code_0[2] | acceptance_mask_0[2] )
		&	(id[6]  == acceptance_code_0[3] | acceptance_mask_0[3] )
		&	(id[7]  == acceptance_code_0[4] | acceptance_mask_0[4] )
		&	(id[8]  == acceptance_code_0[5] | acceptance_mask_0[5] )
		&	(id[9]  == acceptance_code_0[6] | acceptance_mask_0[6] )
		&	(id[10] == acceptance_code_0[7] | acceptance_mask_0[7] )

		&	(rtr1   == acceptance_code_1[4] | acceptance_mask_1[4] )
		&	(id[0]  == acceptance_code_1[5] | acceptance_mask_1[5] )
		&	(id[1]  == acceptance_code_1[6] | acceptance_mask_1[6] )
		&	(id[2]  == acceptance_code_1[7] | acceptance_mask_1[7] )

		&	(data0[0]  == acceptance_code_2[0] | acceptance_mask_2[0] | no_byte0)
		&	(data0[1]  == acceptance_code_2[1] | acceptance_mask_2[1] | no_byte0)
		&	(data0[2]  == acceptance_code_2[2] | acceptance_mask_2[2] | no_byte0)
		&	(data0[3]  == acceptance_code_2[3] | acceptance_mask_2[3] | no_byte0)
		&	(data0[4]  == acceptance_code_2[4] | acceptance_mask_2[4] | no_byte0)
		&	(data0[5]  == acceptance_code_2[5] | acceptance_mask_2[5] | no_byte0)
		&	(data0[6]  == acceptance_code_2[6] | acceptance_mask_2[6] | no_byte0)
		&	(data0[7]  == acceptance_code_2[7] | acceptance_mask_2[7] | no_byte0)

		&	(data1[0]  == acceptance_code_3[0] | acceptance_mask_3[0] | no_byte1)
		&	(data1[1]  == acceptance_code_3[1] | acceptance_mask_3[1] | no_byte1)
		&	(data1[2]  == acceptance_code_3[2] | acceptance_mask_3[2] | no_byte1)
		&	(data1[3]  == acceptance_code_3[3] | acceptance_mask_3[3] | no_byte1)
		&	(data1[4]  == acceptance_code_3[4] | acceptance_mask_3[4] | no_byte1)
		&	(data1[5]  == acceptance_code_3[5] | acceptance_mask_3[5] | no_byte1)
		&	(data1[6]  == acceptance_code_3[6] | acceptance_mask_3[6] | no_byte1)
		&	(data1[7]  == acceptance_code_3[7] | acceptance_mask_3[7] | no_byte1)
		);

	// Working in extended mode. ID match for extended format (29-bit ID). Using single filter.
	assign match_sf_ext =
		(	(id[21]  == acceptance_code_0[0] | acceptance_mask_0[0] )
		&	(id[22]  == acceptance_code_0[1] | acceptance_mask_0[1] )
		&	(id[23]  == acceptance_code_0[2] | acceptance_mask_0[2] )
		&	(id[24]  == acceptance_code_0[3] | acceptance_mask_0[3] )
		&	(id[25]  == acceptance_code_0[4] | acceptance_mask_0[4] )
		&	(id[26]  == acceptance_code_0[5] | acceptance_mask_0[5] )
		&	(id[27]  == acceptance_code_0[6] | acceptance_mask_0[6] )
		&	(id[28]  == acceptance_code_0[7] | acceptance_mask_0[7] )

		&	(id[13]  == acceptance_code_1[0] | acceptance_mask_1[0] )
		&	(id[14]  == acceptance_code_1[1] | acceptance_mask_1[1] )
		&	(id[15]  == acceptance_code_1[2] | acceptance_mask_1[2] )
		&	(id[16]  == acceptance_code_1[3] | acceptance_mask_1[3] )
		&	(id[17]  == acceptance_code_1[4] | acceptance_mask_1[4] )
		&	(id[18]  == acceptance_code_1[5] | acceptance_mask_1[5] )
		&	(id[19]  == acceptance_code_1[6] | acceptance_mask_1[6] )
		&	(id[20]  == acceptance_code_1[7] | acceptance_mask_1[7] )

		&	(id[5]  == acceptance_code_2[0] | acceptance_mask_2[0] )
		&	(id[6]  == acceptance_code_2[1] | acceptance_mask_2[1] )
		&	(id[7]  == acceptance_code_2[2] | acceptance_mask_2[2] )
		&	(id[8]  == acceptance_code_2[3] | acceptance_mask_2[3] )
		&	(id[9]  == acceptance_code_2[4] | acceptance_mask_2[4] )
		&	(id[10] == acceptance_code_2[5] | acceptance_mask_2[5] )
		&	(id[11] == acceptance_code_2[6] | acceptance_mask_2[6] )
		&	(id[12] == acceptance_code_2[7] | acceptance_mask_2[7] )

		&	(rtr2   == acceptance_code_3[2] | acceptance_mask_3[2] )
		&	(id[0]  == acceptance_code_3[3] | acceptance_mask_3[3] )
		&	(id[1]  == acceptance_code_3[4] | acceptance_mask_3[4] )
		&	(id[2]  == acceptance_code_3[5] | acceptance_mask_3[5] )
		&	(id[3]  == acceptance_code_3[6] | acceptance_mask_3[6] )
		&	(id[4]  == acceptance_code_3[7] | acceptance_mask_3[7] )
		);

	// Working in extended mode. ID match for standard format (11-bit ID). Using double filter.
	assign match_df_std =
		(	(	(id[3]  == acceptance_code_0[0] | acceptance_mask_0[0] )
			&	(id[4]  == acceptance_code_0[1] | acceptance_mask_0[1] )
			&	(id[5]  == acceptance_code_0[2] | acceptance_mask_0[2] )
			&	(id[6]  == acceptance_code_0[3] | acceptance_mask_0[3] )
			&	(id[7]  == acceptance_code_0[4] | acceptance_mask_0[4] )
			&	(id[8]  == acceptance_code_0[5] | acceptance_mask_0[5] )
			&	(id[9]  == acceptance_code_0[6] | acceptance_mask_0[6] )
			&	(id[10] == acceptance_code_0[7] | acceptance_mask_0[7] )

			&	(rtr1   == acceptance_code_1[4] | acceptance_mask_1[4] )
			&	(id[0]  == acceptance_code_1[5] | acceptance_mask_1[5] )
			&	(id[1]  == acceptance_code_1[6] | acceptance_mask_1[6] )
			&	(id[2]  == acceptance_code_1[7] | acceptance_mask_1[7] )

			&	(data0[0] == acceptance_code_3[0] | acceptance_mask_3[0] | no_byte0)
			&	(data0[1] == acceptance_code_3[1] | acceptance_mask_3[1] | no_byte0)
			&	(data0[2] == acceptance_code_3[2] | acceptance_mask_3[2] | no_byte0)
			&	(data0[3] == acceptance_code_3[3] | acceptance_mask_3[3] | no_byte0)
			&	(data0[4] == acceptance_code_1[0] | acceptance_mask_1[0] | no_byte0)
			&	(data0[5] == acceptance_code_1[1] | acceptance_mask_1[1] | no_byte0)
			&	(data0[6] == acceptance_code_1[2] | acceptance_mask_1[2] | no_byte0)
			&	(data0[7] == acceptance_code_1[3] | acceptance_mask_1[3] | no_byte0)
			)
		|	(	(id[3]  == acceptance_code_2[0] | acceptance_mask_2[0] )
			&	(id[4]  == acceptance_code_2[1] | acceptance_mask_2[1] )
			&	(id[5]  == acceptance_code_2[2] | acceptance_mask_2[2] )
			&	(id[6]  == acceptance_code_2[3] | acceptance_mask_2[3] )
			&	(id[7]  == acceptance_code_2[4] | acceptance_mask_2[4] )
			&	(id[8]  == acceptance_code_2[5] | acceptance_mask_2[5] )
			&	(id[9]  == acceptance_code_2[6] | acceptance_mask_2[6] )
			&	(id[10] == acceptance_code_2[7] | acceptance_mask_2[7] )

			&	(rtr1   == acceptance_code_3[4] | acceptance_mask_3[4] )
			&	(id[0]  == acceptance_code_3[5] | acceptance_mask_3[5] )
			&	(id[1]  == acceptance_code_3[6] | acceptance_mask_3[6] )
			&	(id[2]  == acceptance_code_3[7] | acceptance_mask_3[7] )
			)
		);

// Working in extended mode. ID match for extended format (29-bit ID). Using double filter.
	assign match_df_ext =
		(	(	(id[21]  == acceptance_code_0[0] | acceptance_mask_0[0] )
			&	(id[22]  == acceptance_code_0[1] | acceptance_mask_0[1] )
			&	(id[23]  == acceptance_code_0[2] | acceptance_mask_0[2] )
			&	(id[24]  == acceptance_code_0[3] | acceptance_mask_0[3] )
			&	(id[25]  == acceptance_code_0[4] | acceptance_mask_0[4] )
			&	(id[26]  == acceptance_code_0[5] | acceptance_mask_0[5] )
			&	(id[27]  == acceptance_code_0[6] | acceptance_mask_0[6] )
			&	(id[28]  == acceptance_code_0[7] | acceptance_mask_0[7] )

			&	(id[13]  == acceptance_code_1[0] | acceptance_mask_1[0] )
			&	(id[14]  == acceptance_code_1[1] | acceptance_mask_1[1] )
			&	(id[15]  == acceptance_code_1[2] | acceptance_mask_1[2] )
			&	(id[16]  == acceptance_code_1[3] | acceptance_mask_1[3] )
			&	(id[17]  == acceptance_code_1[4] | acceptance_mask_1[4] )
			&	(id[18]  == acceptance_code_1[5] | acceptance_mask_1[5] )
			&	(id[19]  == acceptance_code_1[6] | acceptance_mask_1[6] )
			&	(id[20]  == acceptance_code_1[7] | acceptance_mask_1[7] )
			)
		|	(	(id[21]  == acceptance_code_2[0] | acceptance_mask_2[0] )
			&	(id[22]  == acceptance_code_2[1] | acceptance_mask_2[1] )
			&	(id[23]  == acceptance_code_2[2] | acceptance_mask_2[2] )
			&	(id[24]  == acceptance_code_2[3] | acceptance_mask_2[3] )
			&	(id[25]  == acceptance_code_2[4] | acceptance_mask_2[4] )
			&	(id[26]  == acceptance_code_2[5] | acceptance_mask_2[5] )
			&	(id[27]  == acceptance_code_2[6] | acceptance_mask_2[6] )
			&	(id[28]  == acceptance_code_2[7] | acceptance_mask_2[7] )

			&	(id[13]  == acceptance_code_3[0] | acceptance_mask_3[0] )
			&	(id[14]  == acceptance_code_3[1] | acceptance_mask_3[1] )
			&	(id[15]  == acceptance_code_3[2] | acceptance_mask_3[2] )
			&	(id[16]  == acceptance_code_3[3] | acceptance_mask_3[3] )
			&	(id[17]  == acceptance_code_3[4] | acceptance_mask_3[4] )
			&	(id[18]  == acceptance_code_3[5] | acceptance_mask_3[5] )
			&	(id[19]  == acceptance_code_3[6] | acceptance_mask_3[6] )
			&	(id[20]  == acceptance_code_3[7] | acceptance_mask_3[7] )
			)
		);

	// ID ok signal generation
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			id_ok <= 1'b0;
		end else if(go_rx_crc_lim) begin
			// sample_point is already included in go_rx_crc_lim
			if(extended_mode) begin
				if(~acceptance_filter_mode) begin
					// dual filter
					if(ide) begin
						// extended frame message
						id_ok <=#Tp match_df_ext;
					end else begin
						// standard frame message
						id_ok <=#Tp match_df_std;
					end
				end else begin
					// single filter
					if(ide) begin
						// extended frame message
						id_ok <=#Tp match_sf_ext;
					end else begin
						// standard frame message
						id_ok <=#Tp match_sf_std;
					end
				end
			end else begin
				id_ok <=#Tp match;
			end
		end else if(reset_mode | go_rx_inter | go_error_frame) begin
			// sample_point is already included in go_rx_inter
			id_ok <=#Tp 1'b0;
		end
	end

endmodule
