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

module can_btl( 
	input wire clk,
	input wire rst,
	input wire rx,
	input wire tx,

	/* Bus Timing 0 register */
	input wire [5:0] baud_r_presc,
	input wire [1:0] sync_jump_width,

	/* Bus Timing 1 register */
	input wire [3:0] time_segment1,
	input wire [2:0] time_segment2,
	input wire triple_sampling,

	/* Output signals from this module */
	output reg sample_point,
	output reg sampled_bit,
	output reg sampled_bit_q,
	output reg tx_point,
	output wire hard_sync,

	/* From can_bsp module */
	input wire rx_idle,
	input wire rx_inter,
	input wire transmitting,
	input wire transmitter,
	input wire go_rx_inter,
	input wire tx_next,

	input wire go_overload_frame,
	input wire go_error_frame,
	input wire go_tx,
	input wire send_ack,
	input wire node_error_passive);

	parameter Tp = 1;

	reg     [6:0] clk_cnt;
	reg           clk_en;
	reg           clk_en_q;
	reg           sync_blocked;
	reg           hard_sync_blocked;
	reg     [4:0] quant_cnt;
	reg     [3:0] delay;
	reg           sync;
	reg           seg1;
	reg           seg2;
	reg           resync_latched;
	reg     [1:0] sample;
	reg           tx_next_sp;

	wire          go_sync;
	wire          go_seg1;
	wire          go_seg2;
	wire [7:0]    preset_cnt;
	wire          sync_window;
	wire          resync;

	assign preset_cnt = (baud_r_presc + 1'b1)<<1;        // (BRP+1)*2
	assign hard_sync  =   (rx_idle | rx_inter)    & (~rx) & sampled_bit & (~hard_sync_blocked);  // Hard synchronization
	assign resync     =  (~rx_idle) & (~rx_inter) & (~rx) & sampled_bit & (~sync_blocked);       // Re-synchronization

	// Generating general enable signal that defines baud rate
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			clk_cnt <= 7'h0;
		end else if(clk_cnt >= (preset_cnt-1'b1)) begin
			clk_cnt <=#Tp 7'h0;
		end else begin
			clk_cnt <=#Tp clk_cnt + 1'b1;
		end
	end

	always @(posedge clk or posedge rst) begin
		if(rst) begin
			clk_en  <= 1'b0;
		end else if({1'b0, clk_cnt} == (preset_cnt-1'b1)) begin
			clk_en  <=#Tp 1'b1;
		end else begin
			clk_en  <=#Tp 1'b0;
		end
	end

	always @(posedge clk or posedge rst) begin
		if(rst) begin
			clk_en_q  <= 1'b0;
		end else begin
			clk_en_q  <=#Tp clk_en;
		end
	end

	// Changing states
	assign go_sync = clk_en_q & seg2 & (quant_cnt[2:0] == time_segment2) & (~hard_sync) & (~resync);
	assign go_seg1 = clk_en_q & (sync | hard_sync | (resync & seg2 & sync_window) | (resync_latched & sync_window));
	assign go_seg2 = clk_en_q & (seg1 & (~hard_sync) & (quant_cnt == (time_segment1 + delay)));

	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_point <= 1'b0;
		end else begin
			// When transmitter we should transmit as soon as possible.
			tx_point <=#Tp ~tx_point & seg2 & (
						clk_en & (quant_cnt[2:0] == time_segment2)
						| (clk_en | clk_en_q) & (resync | hard_sync)
					);
		end
	end

	/* When early edge is detected outside of the SJW field, synchronization request is latched and performed when
	SJW is reached */
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			resync_latched <= 1'b0;
		end else if(resync & seg2 & (~sync_window)) begin
			resync_latched <=#Tp 1'b1;
		end else if(go_seg1) begin
			resync_latched <= 1'b0;
		end
	end

	// Synchronization stage/segment
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			sync <= 1'b0;
		end else if(clk_en_q) begin
			sync <=#Tp go_sync;
		end
	end

	// Seg1 stage/segment (together with propagation segment which is 1 quant long)
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			seg1 <= 1'b1;
		end else if(go_seg1) begin
			seg1 <=#Tp 1'b1;
		end else if(go_seg2) begin
			seg1 <=#Tp 1'b0;
		end
	end

	// Seg2 stage/segment
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			seg2 <= 1'b0;
		end else if(go_seg2) begin
			seg2 <=#Tp 1'b1;
		end else if(go_sync | go_seg1) begin
			seg2 <=#Tp 1'b0;
		end
	end

	// Quant counter
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			quant_cnt <= 5'h0;
		end else if(go_sync | go_seg1 | go_seg2) begin
			quant_cnt <=#Tp 5'h0;
		end else if(clk_en_q) begin
			quant_cnt <=#Tp quant_cnt + 1'b1;
		end
	end

	// When late edge is detected (in seg1 stage), stage seg1 is prolonged.
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			delay <= 4'h0;
		end else if(resync & seg1 & (~transmitting | transmitting & (tx_next_sp | (tx & (~rx))))) begin
			// when transmitting 0 with positive error delay is set to 0
			delay <=#Tp (quant_cnt > {3'h0, sync_jump_width})? ({2'h0, sync_jump_width} + 1'b1) : (quant_cnt + 1'b1);
		end else if(go_sync | go_seg1) begin
			delay <=#Tp 4'h0;
		end
	end

	// If early edge appears within this window (in seg2 stage), phase error is fully compensated
	assign sync_window = ((time_segment2 - quant_cnt[2:0]) < ( sync_jump_width + 1'b1));

	// Sampling data (memorizing two samples all the time).
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			sample <= 2'b11;
		end else if(clk_en_q) begin
			sample <= {sample[0], rx};
		end
	end

	// When enabled, tripple sampling is done here.
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			sampled_bit <= 1'b1;
			sampled_bit_q <= 1'b1;
			sample_point <= 1'b0;
		end else if(go_error_frame) begin
			sampled_bit_q <=#Tp sampled_bit;
			sample_point <=#Tp 1'b0;
		end else if(clk_en_q & (~hard_sync)) begin
			if(seg1 & (quant_cnt == (time_segment1 + delay))) begin
				sample_point <=#Tp 1'b1;
				sampled_bit_q <=#Tp sampled_bit;
				if(triple_sampling) begin
					sampled_bit <=#Tp (sample[0] & sample[1]) | ( sample[0] & rx) | (sample[1] & rx);
				end else begin
					sampled_bit <=#Tp rx;
				end
			end
		end else begin
			sample_point <=#Tp 1'b0;
		end
	end

	// tx_next_sp shows next value that will be driven on the TX. When driving 1 and receiving 0 we
	// need to synchronize (even when we are a transmitter)
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_next_sp <= 1'b0;
		end else if(go_overload_frame | (go_error_frame & (~node_error_passive)) | go_tx | send_ack) begin
			tx_next_sp <=#Tp 1'b0;
		end else if(go_error_frame & node_error_passive) begin
			tx_next_sp <=#Tp 1'b1;
		end else if(sample_point) begin
			tx_next_sp <=#Tp tx_next;
		end
	end

	// Blocking synchronization (can occur only once in a bit time)
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			sync_blocked <=#Tp 1'b1;
		end else if(clk_en_q) begin
			if(resync) begin
				sync_blocked <=#Tp 1'b1;
			end else if(go_seg2) begin
				sync_blocked <=#Tp 1'b0;
			end
		end
	end

	// Blocking hard synchronization when occurs once or when we are transmitting a msg
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			hard_sync_blocked <=#Tp 1'b0;
		end else if(hard_sync & clk_en_q | (transmitting & transmitter | go_tx) & tx_point & (~tx_next)) begin
			hard_sync_blocked <=#Tp 1'b1;
		end else if(go_rx_inter | (rx_idle | rx_inter) & sample_point & sampled_bit) begin
			// When a glitch performed synchronization
			hard_sync_blocked <=#Tp 1'b0;
		end
	end

endmodule
