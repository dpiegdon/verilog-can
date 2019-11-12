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

`include "can_top_defines.v"
`include "can_crc.v"
`include "can_acf.v"
`include "can_fifo.v"
`include "can_ibo.v"

module can_bsp(
	input wire clk,
	input wire rst,

	input wire sample_point,
	input wire sampled_bit,
	input wire sampled_bit_q,
	input wire tx_point,
	input wire hard_sync,

	input wire [7:0] addr,
	input wire [7:0] data_in,
	output wire [7:0] data_out,
	input wire fifo_selected,

	/* Mode register */
	input wire reset_mode,
	input wire listen_only_mode,
	input wire acceptance_filter_mode,
	input wire self_test_mode,

	/* Command register */
	input wire release_buffer,
	input wire tx_request,
	input wire abort_tx,
	input wire self_rx_request,
	input wire single_shot_transmission,
	output reg tx_state,
	output reg tx_state_q,
	input wire overload_request, // When receiver is busy, it needs to send overload frame. Only 2 overload frames are allowed to
	output reg overload_frame,   // be send in a row. This is not implemented, yet,  because host can not send an overload request.

	/* Arbitration Lost Capture Register */
	input wire read_arbitration_lost_capture_reg,

	/* Error Code Capture Register */
	input wire read_error_code_capture_reg,
	output reg [7:0] error_capture_code,

	/* Error Warning Limit register */
	input wire [7:0] error_warning_limit,

	/* Rx Error Counter register */
	input wire we_rx_err_cnt,

	/* Tx Error Counter register */
	input wire we_tx_err_cnt,

	/* Clock Divider register */
	input wire extended_mode,

	output reg rx_idle,
	output reg transmitting,
	output reg transmitter,
	output wire go_rx_inter,
	output wire not_first_bit_of_inter,
	output reg rx_inter,
	output wire set_reset_mode,
	output reg node_bus_off,
	output wire error_status,
	output reg [8:0] rx_err_cnt,
	output reg [8:0] tx_err_cnt,
	output wire transmit_status,
	output wire receive_status,
	output wire tx_successful,
	output reg need_to_tx, // When the CAN core has something to transmit and a dominant bit is sampled at the third bit
	output wire overrun,
	output wire info_empty,
	output wire set_bus_error_irq,
	output wire set_arbitration_lost_irq,
	output reg [4:0] arbitration_lost_capture,
	output reg node_error_passive,
	output wire node_error_active,
	output wire [6:0] rx_message_counter,

	/* This section is for BASIC and EXTENDED mode */
	/* Acceptance code register */
	input wire [7:0] acceptance_code_0,

	/* Acceptance mask register */
	input wire [7:0] acceptance_mask_0,
	/* End: This section is for BASIC and EXTENDED mode */

	/* This section is for EXTENDED mode */
	/* Acceptance code register */
	input wire [7:0] acceptance_code_1,
	input wire [7:0] acceptance_code_2,
	input wire [7:0] acceptance_code_3,

	/* Acceptance mask register */
	input wire [7:0] acceptance_mask_1,
	input wire [7:0] acceptance_mask_2,
	input wire [7:0] acceptance_mask_3,
	/* End: This section is for EXTENDED mode */

	/* Tx data registers. Holding identifier (basic mode), tx frame information (extended mode) and data */
	input wire [7:0] tx_data_0,
	input wire [7:0] tx_data_1,
	input wire [7:0] tx_data_2,
	input wire [7:0] tx_data_3,
	input wire [7:0] tx_data_4,
	input wire [7:0] tx_data_5,
	input wire [7:0] tx_data_6,
	input wire [7:0] tx_data_7,
	input wire [7:0] tx_data_8,
	input wire [7:0] tx_data_9,
	input wire [7:0] tx_data_10,
	input wire [7:0] tx_data_11,
	input wire [7:0] tx_data_12,
	/* End: Tx data registers */

	/* Tx signal */
	output reg tx,
	output reg tx_next,
	output wire bus_off_on,

	output wire go_overload_frame,
	output wire go_error_frame,
	output wire go_tx,
	output wire send_ack

	`ifdef CAN_BIST
	,
	input wire mbist_si_i,
	output wire mbist_so_o,
	input wire [`CAN_MBIST_CTRL_WIDTH - 1:0] mbist_ctrl_i
	`endif
	);

	parameter Tp = 1;

	reg           reset_mode_q;
	reg     [5:0] bit_cnt;

	reg     [3:0] data_len;
	reg    [28:0] id;
	reg     [2:0] bit_stuff_cnt;
	reg     [2:0] bit_stuff_cnt_tx;
	reg           tx_point_q;

	reg           rx_id1;
	reg           rx_rtr1;
	reg           rx_ide;
	reg           rx_id2;
	reg           rx_rtr2;
	reg           rx_r1;
	reg           rx_r0;
	reg           rx_dlc;
	reg           rx_data;
	reg           rx_crc;
	reg           rx_crc_lim;
	reg           rx_ack;
	reg           rx_ack_lim;
	reg           rx_eof;
	reg           go_early_tx_latched;

	reg           rtr1;
	reg           ide;
	reg           rtr2;
	reg    [14:0] crc_in;

	reg     [7:0] tmp_data;
	reg     [7:0] tmp_fifo [0:7];
	reg           write_data_to_tmp_fifo;
	reg     [2:0] byte_cnt;
	reg           bit_stuff_cnt_en;
	reg           crc_enable;

	reg     [2:0] eof_cnt;
	reg     [2:0] passive_cnt;


	reg           error_frame;
	reg           enable_error_cnt2;
	reg     [2:0] error_cnt1;
	reg     [2:0] error_cnt2;
	reg     [2:0] delayed_dominant_cnt;
	reg           enable_overload_cnt2;
	reg           overload_frame_blocked;
	reg     [1:0] overload_request_cnt;
	reg     [2:0] overload_cnt1;
	reg     [2:0] overload_cnt2;
	reg           crc_err;

	reg           arbitration_lost;
	reg           arbitration_lost_q;
	reg           arbitration_field_d;
	reg     [4:0] arbitration_cnt;
	reg           arbitration_blocked;
	reg           tx_q;

	reg     [3:0] data_cnt;     // Counting the data bytes that are written to FIFO
	reg     [2:0] header_cnt;   // Counting header length
	reg           wr_fifo;      // Write data and header to 64-byte fifo
	reg     [7:0] data_for_fifo;// Multiplexed data that is stored to 64-byte fifo

	reg     [5:0] tx_pointer;
	reg           tx_bit;
	reg           finish_msg;

	reg     [3:0] bus_free_cnt;
	reg           bus_free_cnt_en;
	reg           bus_free;
	reg           waiting_for_bus_free;

	reg           node_bus_off_q;
	reg           ack_err_latched;
	reg           bit_err_latched;
	reg           stuff_err_latched;
	reg           form_err_latched;
	reg           rule3_exc1_1;
	reg           rule3_exc1_2;
	reg           suspend;
	reg           susp_cnt_en;
	reg     [2:0] susp_cnt;
	reg           error_flag_over_latched;

	reg     [7:6] error_capture_code_type;
	reg           error_capture_code_blocked;
	reg           first_compare_bit;


	wire    [4:0] error_capture_code_segment;
	wire          error_capture_code_direction;

	wire          bit_de_stuff;
	wire          bit_de_stuff_tx;

	wire          rule5;

	/* Rx state machine */
	wire          go_rx_idle;
	wire          go_rx_id1;
	wire          go_rx_rtr1;
	wire          go_rx_ide;
	wire          go_rx_id2;
	wire          go_rx_rtr2;
	wire          go_rx_r1;
	wire          go_rx_r0;
	wire          go_rx_dlc;
	wire          go_rx_data;
	wire          go_rx_crc;
	wire          go_rx_crc_lim;
	wire          go_rx_ack;
	wire          go_rx_ack_lim;
	wire          go_rx_eof;

	wire          last_bit_of_inter;

	wire          go_crc_enable;
	wire          rst_crc_enable;

	wire          bit_de_stuff_set;
	wire          bit_de_stuff_reset;

	wire          go_early_tx;

	wire   [14:0] calculated_crc;
	wire   [15:0] r_calculated_crc;
	wire          remote_rq;
	wire    [3:0] limited_data_len;
	wire          form_err;

	wire          error_frame_ended;
	wire          overload_frame_ended;
	wire          bit_err;
	wire          ack_err;
	wire          stuff_err;

	wire          id_ok;                // If received ID matches ID set in registers
	wire          no_byte0;             // There is no byte 0 (RTR bit set to 1 or DLC field equal to 0). Signal used for acceptance filter.
	wire          no_byte1;             // There is no byte 1 (RTR bit set to 1 or DLC field equal to 1). Signal used for acceptance filter.

	wire    [2:0] header_len;
	wire          storing_header;
	wire    [3:0] limited_data_len_minus1;
	wire          reset_wr_fifo;
	wire          err;

	wire          arbitration_field;

	wire   [18:0] basic_chain;
	wire   [63:0] basic_chain_data;
	wire   [18:0] extended_chain_std;
	wire   [38:0] extended_chain_ext;
	wire   [63:0] extended_chain_data_std;
	wire   [63:0] extended_chain_data_ext;

	wire          rst_tx_pointer;

	wire    [7:0] r_tx_data_0;
	wire    [7:0] r_tx_data_1;
	wire    [7:0] r_tx_data_2;
	wire    [7:0] r_tx_data_3;
	wire    [7:0] r_tx_data_4;
	wire    [7:0] r_tx_data_5;
	wire    [7:0] r_tx_data_6;
	wire    [7:0] r_tx_data_7;
	wire    [7:0] r_tx_data_8;
	wire    [7:0] r_tx_data_9;
	wire    [7:0] r_tx_data_10;
	wire    [7:0] r_tx_data_11;
	wire    [7:0] r_tx_data_12;

	wire          bit_err_exc1;
	wire          bit_err_exc2;
	wire          bit_err_exc3;
	wire          bit_err_exc4;
	wire          bit_err_exc5;
	wire          bit_err_exc6;
	wire          error_flag_over;
	wire          overload_flag_over;

	wire    [5:0] limited_tx_cnt_ext;
	wire    [5:0] limited_tx_cnt_std;

	assign go_rx_idle     =                   sample_point &  sampled_bit & last_bit_of_inter | bus_free & (~node_bus_off);
	assign go_rx_id1      =                   sample_point &  (~sampled_bit) & (rx_idle | last_bit_of_inter);
	assign go_rx_rtr1     = (~bit_de_stuff) & sample_point &  rx_id1  & (bit_cnt[3:0] == 4'd10);
	assign go_rx_ide      = (~bit_de_stuff) & sample_point &  rx_rtr1;
	assign go_rx_id2      = (~bit_de_stuff) & sample_point &  rx_ide  &   sampled_bit;
	assign go_rx_rtr2     = (~bit_de_stuff) & sample_point &  rx_id2  & (bit_cnt[4:0] == 5'd17);
	assign go_rx_r1       = (~bit_de_stuff) & sample_point &  rx_rtr2;
	assign go_rx_r0       = (~bit_de_stuff) & sample_point & (rx_ide  & (~sampled_bit) | rx_r1);
	assign go_rx_dlc      = (~bit_de_stuff) & sample_point &  rx_r0;
	assign go_rx_data     = (~bit_de_stuff) & sample_point &  rx_dlc  & (bit_cnt[1:0] == 2'd3) &  (sampled_bit   |   (|data_len[2:0])) & (~remote_rq);
	assign go_rx_crc      = (~bit_de_stuff) & sample_point & (rx_dlc  & (bit_cnt[1:0] == 2'd3) & ((~sampled_bit) & (~(|data_len[2:0])) | remote_rq) |
								  rx_data & (bit_cnt[5:0] == ((limited_data_len<<3) - 1'b1)));  // overflow works ok at max value (8<<3 = 64 = 0). 0-1 = 6'h3f
	assign go_rx_crc_lim  = (~bit_de_stuff) & sample_point &  rx_crc  & (bit_cnt[3:0] == 4'd14);
	assign go_rx_ack      = (~bit_de_stuff) & sample_point &  rx_crc_lim;
	assign go_rx_ack_lim  =                   sample_point &  rx_ack;
	assign go_rx_eof      =                   sample_point &  rx_ack_lim;
	assign go_rx_inter    =                 ((sample_point &  rx_eof  & (eof_cnt == 3'd6)) | error_frame_ended | overload_frame_ended) & (~overload_request);

	assign go_error_frame = (form_err | stuff_err | bit_err | ack_err | (crc_err & go_rx_eof));
	assign error_frame_ended = (error_cnt2 == 3'd7) & tx_point;
	assign overload_frame_ended = (overload_cnt2 == 3'd7) & tx_point;

	assign go_overload_frame = (     sample_point & ((~sampled_bit) | overload_request) & (rx_eof & (~transmitter) & (eof_cnt == 3'd6) | error_frame_ended | overload_frame_ended) |
					 sample_point & (~sampled_bit) & rx_inter & (bit_cnt[1:0] < 2'd2)                                                            |
					 sample_point & (~sampled_bit) & ((error_cnt2 == 3'd7) | (overload_cnt2 == 3'd7))
				   )
				   & (~overload_frame_blocked)
				   ;


	assign go_crc_enable  = hard_sync | go_tx;
	assign rst_crc_enable = go_rx_crc;

	assign bit_de_stuff_set   = go_rx_id1 & (~go_error_frame);
	assign bit_de_stuff_reset = go_rx_ack | go_error_frame | go_overload_frame;

	assign remote_rq = ((~ide) & rtr1) | (ide & rtr2);
	assign limited_data_len = (data_len < 4'h8)? data_len : 4'h8;

	assign ack_err = rx_ack & sample_point & sampled_bit & tx_state & (~self_test_mode);
	assign bit_err = (tx_state | error_frame | overload_frame | rx_ack) & sample_point & (tx != sampled_bit) & (~bit_err_exc1) & (~bit_err_exc2) & (~bit_err_exc3) & (~bit_err_exc4) & (~bit_err_exc5) & (~bit_err_exc6) & (~reset_mode);
	assign bit_err_exc1 = tx_state & arbitration_field & tx;
	assign bit_err_exc2 = rx_ack & tx;
	assign bit_err_exc3 = error_frame & node_error_passive & (error_cnt1 < 3'd7);
	assign bit_err_exc4 = (error_frame & (error_cnt1 == 3'd7) & (~enable_error_cnt2)) | (overload_frame & (overload_cnt1 == 3'd7) & (~enable_overload_cnt2));
	assign bit_err_exc5 = (error_frame & (error_cnt2 == 3'd7)) | (overload_frame & (overload_cnt2 == 3'd7));
	assign bit_err_exc6 = (eof_cnt == 3'd6) & rx_eof & (~transmitter);

	assign arbitration_field = rx_id1 | rx_rtr1 | rx_ide | rx_id2 | rx_rtr2;

	assign last_bit_of_inter = rx_inter & (bit_cnt[1:0] == 2'd2);
	assign not_first_bit_of_inter = rx_inter & (bit_cnt[1:0] != 2'd0);


	// Rx idle state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_idle <= 1'b0;
		end else if(go_rx_id1 | go_error_frame) begin
			rx_idle <=#Tp 1'b0;
		end else if(go_rx_idle) begin
			rx_idle <=#Tp 1'b1;
		end
	end


	// Rx id1 state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_id1 <= 1'b0;
		end else if(go_rx_rtr1 | go_error_frame) begin
			rx_id1 <=#Tp 1'b0;
		end else if(go_rx_id1) begin
			rx_id1 <=#Tp 1'b1;
		end
	end


	// Rx rtr1 state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_rtr1 <= 1'b0;
		end else if(go_rx_ide | go_error_frame) begin
			rx_rtr1 <=#Tp 1'b0;
		end else if(go_rx_rtr1) begin
			rx_rtr1 <=#Tp 1'b1;
		end
	end


	// Rx ide state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_ide <= 1'b0;
		end else if(go_rx_r0 | go_rx_id2 | go_error_frame) begin
			rx_ide <=#Tp 1'b0;
		end else if(go_rx_ide) begin
			rx_ide <=#Tp 1'b1;
		end
	end


	// Rx id2 state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_id2 <= 1'b0;
		end else if(go_rx_rtr2 | go_error_frame) begin
			rx_id2 <=#Tp 1'b0;
		end else if(go_rx_id2) begin
			rx_id2 <=#Tp 1'b1;
		end
	end


	// Rx rtr2 state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_rtr2 <= 1'b0;
		end else if(go_rx_r1 | go_error_frame) begin
			rx_rtr2 <=#Tp 1'b0;
		end else if(go_rx_rtr2) begin
			rx_rtr2 <=#Tp 1'b1;
		end
	end


	// Rx r0 state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_r1 <= 1'b0;
		end else if(go_rx_r0 | go_error_frame) begin
			rx_r1 <=#Tp 1'b0;
		end else if(go_rx_r1) begin
			rx_r1 <=#Tp 1'b1;
		end
	end


	// Rx r0 state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_r0 <= 1'b0;
		end else if(go_rx_dlc | go_error_frame) begin
			rx_r0 <=#Tp 1'b0;
		end else if(go_rx_r0) begin
			rx_r0 <=#Tp 1'b1;
		end
	end


	// Rx dlc state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_dlc <= 1'b0;
		end else if(go_rx_data | go_rx_crc | go_error_frame) begin
			rx_dlc <=#Tp 1'b0;
		end else if(go_rx_dlc) begin
			rx_dlc <=#Tp 1'b1;
		end
	end


	// Rx data state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_data <= 1'b0;
		end else if(go_rx_crc | go_error_frame) begin
			rx_data <=#Tp 1'b0;
		end else if(go_rx_data) begin
			rx_data <=#Tp 1'b1;
		end
	end


	// Rx crc state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_crc <= 1'b0;
		end else if(go_rx_crc_lim | go_error_frame) begin
			rx_crc <=#Tp 1'b0;
		end else if(go_rx_crc) begin
			rx_crc <=#Tp 1'b1;
		end
	end


	// Rx crc delimiter state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_crc_lim <= 1'b0;
		end else if(go_rx_ack | go_error_frame) begin
			rx_crc_lim <=#Tp 1'b0;
		end else if(go_rx_crc_lim) begin
			rx_crc_lim <=#Tp 1'b1;
		end
	end


	// Rx ack state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_ack <= 1'b0;
		end else if(go_rx_ack_lim | go_error_frame) begin
			rx_ack <=#Tp 1'b0;
		end else if(go_rx_ack) begin
			rx_ack <=#Tp 1'b1;
		end
	end


	// Rx ack delimiter state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_ack_lim <= 1'b0;
		end else if(go_rx_eof | go_error_frame) begin
			rx_ack_lim <=#Tp 1'b0;
		end else if(go_rx_ack_lim) begin
			rx_ack_lim <=#Tp 1'b1;
		end
	end


	// Rx eof state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_eof <= 1'b0;
		end else if(go_rx_inter | go_error_frame | go_overload_frame) begin
			rx_eof <=#Tp 1'b0;
		end else if(go_rx_eof) begin
			rx_eof <=#Tp 1'b1;
		end
	end



	// Interframe space
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_inter <= 1'b0;
		end else if(go_rx_idle | go_rx_id1 | go_overload_frame | go_error_frame) begin
			rx_inter <=#Tp 1'b0;
		end else if(go_rx_inter) begin
			rx_inter <=#Tp 1'b1;
		end
	end


	// ID register
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			id <= 29'h0;
		end else if(sample_point & (rx_id1 | rx_id2) & (~bit_de_stuff)) begin
			id <=#Tp {id[27:0], sampled_bit};
		end
	end


	// rtr1 bit
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rtr1 <= 1'b0;
		end else if(sample_point & rx_rtr1 & (~bit_de_stuff)) begin
			rtr1 <=#Tp sampled_bit;
		end
	end


	// rtr2 bit
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rtr2 <= 1'b0;
		end else if(sample_point & rx_rtr2 & (~bit_de_stuff)) begin
			rtr2 <=#Tp sampled_bit;
		end
	end


	// ide bit
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			ide <= 1'b0;
		end else if(sample_point & rx_ide & (~bit_de_stuff)) begin
			ide <=#Tp sampled_bit;
		end
	end


	// Data length
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			data_len <= 4'b0;
		end else if(sample_point & rx_dlc & (~bit_de_stuff)) begin
			data_len <=#Tp {data_len[2:0], sampled_bit};
		end
	end


	// Data
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tmp_data <= 8'h0;
		end else if(sample_point & rx_data & (~bit_de_stuff)) begin
			tmp_data <=#Tp {tmp_data[6:0], sampled_bit};
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			write_data_to_tmp_fifo <= 1'b0;
		end else if(sample_point & rx_data & (~bit_de_stuff) & (&bit_cnt[2:0])) begin
			write_data_to_tmp_fifo <=#Tp 1'b1;
		end else begin
			write_data_to_tmp_fifo <=#Tp 1'b0;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			byte_cnt <= 3'h0;
		end else if(write_data_to_tmp_fifo) begin
			byte_cnt <=#Tp byte_cnt + 1'b1;
		end else if(sample_point & go_rx_crc_lim) begin
			byte_cnt <=#Tp 3'h0;
		end
	end


	always @(posedge clk) begin
		if(write_data_to_tmp_fifo) begin
			tmp_fifo[byte_cnt] <=#Tp tmp_data;
		end
	end



	// CRC
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			crc_in <= 15'h0;
		end else if(sample_point & rx_crc & (~bit_de_stuff)) begin
			crc_in <=#Tp {crc_in[13:0], sampled_bit};
		end
	end


	// bit_cnt
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bit_cnt <= 6'd0;
		end else if(go_rx_id1 | go_rx_id2 | go_rx_dlc | go_rx_data | go_rx_crc | go_rx_ack | go_rx_eof | go_rx_inter | go_error_frame | go_overload_frame) begin
			bit_cnt <=#Tp 6'd0;
		end else if(sample_point & (~bit_de_stuff)) begin
			bit_cnt <=#Tp bit_cnt + 1'b1;
		end
	end


	// eof_cnt
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			eof_cnt <= 3'd0;
		end else if(sample_point) begin
			if(go_rx_inter | go_error_frame | go_overload_frame) begin
				eof_cnt <=#Tp 3'd0;
			end else if(rx_eof) begin
				eof_cnt <=#Tp eof_cnt + 1'b1;
			end
		end
	end


	// Enabling bit de-stuffing
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bit_stuff_cnt_en <= 1'b0;
		end else if(bit_de_stuff_set) begin
			bit_stuff_cnt_en <=#Tp 1'b1;
		end else if(bit_de_stuff_reset) begin
			bit_stuff_cnt_en <=#Tp 1'b0;
		end
	end


	// bit_stuff_cnt
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bit_stuff_cnt <= 3'h1;
		end else if(bit_de_stuff_reset) begin
			bit_stuff_cnt <=#Tp 3'h1;
		end else if(sample_point & bit_stuff_cnt_en) begin
			if(bit_stuff_cnt == 3'h5) begin
				bit_stuff_cnt <=#Tp 3'h1;
			end else if(sampled_bit == sampled_bit_q) begin
				bit_stuff_cnt <=#Tp bit_stuff_cnt + 1'b1;
			end else begin
				bit_stuff_cnt <=#Tp 3'h1;
			end
		end
	end


	// bit_stuff_cnt_tx
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bit_stuff_cnt_tx <= 3'h1;
		end else if(reset_mode || bit_de_stuff_reset) begin
			bit_stuff_cnt_tx <=#Tp 3'h1;
		end else if(tx_point_q & bit_stuff_cnt_en) begin
			if(bit_stuff_cnt_tx == 3'h5) begin
				bit_stuff_cnt_tx <=#Tp 3'h1;
			end else if(tx == tx_q) begin
				bit_stuff_cnt_tx <=#Tp bit_stuff_cnt_tx + 1'b1;
			end else begin
				bit_stuff_cnt_tx <=#Tp 3'h1;
			end
		end
	end


	assign bit_de_stuff = bit_stuff_cnt == 3'h5;
	assign bit_de_stuff_tx = bit_stuff_cnt_tx == 3'h5;



	// stuff_err
	assign stuff_err = sample_point & bit_stuff_cnt_en & bit_de_stuff & (sampled_bit == sampled_bit_q);



	// Generating delayed signals
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			reset_mode_q <=#Tp 1'b0;
			node_bus_off_q <=#Tp 1'b0;
		end else begin
			reset_mode_q <=#Tp reset_mode;
			node_bus_off_q <=#Tp node_bus_off;
		end
	end



	always @(posedge clk or posedge rst) begin
		if(rst) begin
			crc_enable <= 1'b0;
		end else if(rst_crc_enable) begin
			crc_enable <=#Tp 1'b0;
		end else if(go_crc_enable) begin
			crc_enable <=#Tp 1'b1;
		end
	end


	// CRC error generation
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			crc_err <= 1'b0;
		end else if(reset_mode | error_frame_ended) begin
			crc_err <=#Tp 1'b0;
		end else if(go_rx_ack) begin
			crc_err <=#Tp crc_in != calculated_crc;
		end
	end


	// Conditions for form error
	assign form_err = sample_point & ( ((~bit_de_stuff) & rx_crc_lim & (~sampled_bit)                  ) |
					   (                  rx_ack_lim & (~sampled_bit)                  ) |
					   ((eof_cnt < 3'd6)& rx_eof     & (~sampled_bit) & (~transmitter) ) |
					   (                & rx_eof     & (~sampled_bit) &   transmitter  )
					 );


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			ack_err_latched <= 1'b0;
		end else if(reset_mode | error_frame_ended | go_overload_frame) begin
			ack_err_latched <=#Tp 1'b0;
		end else if(ack_err) begin
			ack_err_latched <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bit_err_latched <= 1'b0;
		end else if(reset_mode | error_frame_ended | go_overload_frame) begin
			bit_err_latched <=#Tp 1'b0;
		end else if(bit_err) begin
			bit_err_latched <=#Tp 1'b1;
		end
	end



	// Rule 5 (Fault confinement).
	assign rule5 = bit_err &  ( (~node_error_passive) & error_frame    & (error_cnt1    < 3'd7)
				    | overload_frame & (overload_cnt1 < 3'd7)
				  );

	// Rule 3 exception 1 - first part (Fault confinement).
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rule3_exc1_1 <= 1'b0;
		end else if(error_flag_over | rule3_exc1_2) begin
			rule3_exc1_1 <=#Tp 1'b0;
		end else if(transmitter & node_error_passive & ack_err) begin
			rule3_exc1_1 <=#Tp 1'b1;
		end
	end


	// Rule 3 exception 1 - second part (Fault confinement).
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rule3_exc1_2 <= 1'b0;
		end else if(go_error_frame | rule3_exc1_2) begin
			rule3_exc1_2 <=#Tp 1'b0;
		end else if(rule3_exc1_1 & (error_cnt1 < 3'd7) & sample_point & (~sampled_bit)) begin
			rule3_exc1_2 <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			stuff_err_latched <= 1'b0;
		end else if(reset_mode | error_frame_ended | go_overload_frame) begin
			stuff_err_latched <=#Tp 1'b0;
		end else if(stuff_err) begin
			stuff_err_latched <=#Tp 1'b1;
		end
	end



	always @(posedge clk or posedge rst) begin
		if(rst) begin
			form_err_latched <= 1'b0;
		end else if(reset_mode | error_frame_ended | go_overload_frame) begin
			form_err_latched <=#Tp 1'b0;
		end else if(form_err) begin
			form_err_latched <=#Tp 1'b1;
		end
	end



	// Instantiation of the RX CRC module
	can_crc i_can_crc_rx
	(
		.clk(clk),
		.data(sampled_bit),
		.enable(crc_enable & sample_point & (~bit_de_stuff)),
		.initialize(go_crc_enable),
		.crc(calculated_crc)
	);




	assign no_byte0 = rtr1 | (data_len<4'h1);
	assign no_byte1 = rtr1 | (data_len<4'h2);

	can_acf i_can_acf
	(
		.clk(clk),
		.rst(rst),

		.id(id),

		/* Mode register */
		.reset_mode(reset_mode),
		.acceptance_filter_mode(acceptance_filter_mode),

		// Clock Divider register
		.extended_mode(extended_mode),

		/* This section is for BASIC and EXTENDED mode */
		/* Acceptance code register */
		.acceptance_code_0(acceptance_code_0),

		/* Acceptance mask register */
		.acceptance_mask_0(acceptance_mask_0),
		/* End: This section is for BASIC and EXTENDED mode */

		/* This section is for EXTENDED mode */
		/* Acceptance code register */
		.acceptance_code_1(acceptance_code_1),
		.acceptance_code_2(acceptance_code_2),
		.acceptance_code_3(acceptance_code_3),

		/* Acceptance mask register */
		.acceptance_mask_1(acceptance_mask_1),
		.acceptance_mask_2(acceptance_mask_2),
		.acceptance_mask_3(acceptance_mask_3),
		/* End: This section is for EXTENDED mode */

		.go_rx_crc_lim(go_rx_crc_lim),
		.go_rx_inter(go_rx_inter),
		.go_error_frame(go_error_frame),

		.data0(tmp_fifo[0]),
		.data1(tmp_fifo[1]),
		.rtr1(rtr1),
		.rtr2(rtr2),
		.ide(ide),
		.no_byte0(no_byte0),
		.no_byte1(no_byte1),

		.id_ok(id_ok)
	);




	assign header_len[2:0] = extended_mode ? (ide? (3'h5) : (3'h3)) : 3'h2;
	assign storing_header = header_cnt < header_len;
	assign limited_data_len_minus1[3:0] = remote_rq? 4'hf : ((data_len < 4'h8)? (data_len -1'b1) : 4'h7);   // - 1 because counter counts from 0
	assign reset_wr_fifo = (data_cnt == (limited_data_len_minus1 + {1'b0, header_len})) || reset_mode;

	assign err = form_err | stuff_err | bit_err | ack_err | form_err_latched | stuff_err_latched | bit_err_latched | ack_err_latched | crc_err;



	// Write enable signal for 64-byte rx fifo
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			wr_fifo <= 1'b0;
		end else if(reset_wr_fifo) begin
			wr_fifo <=#Tp 1'b0;
		end else if(go_rx_inter & id_ok & (~error_frame_ended) & ((~tx_state) | self_rx_request)) begin
			wr_fifo <=#Tp 1'b1;
		end
	end


	// Header counter. Header length depends on the mode of operation and frame format.
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			header_cnt <= 3'h0;
		end else if(reset_wr_fifo) begin
			header_cnt <=#Tp 3'h0;
		end else if(wr_fifo & storing_header) begin
			header_cnt <=#Tp header_cnt + 1'h1;
		end
	end


	// Data counter. Length of the data is limited to 8 bytes.
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			data_cnt <= 4'h0;
		end else if(reset_wr_fifo) begin
			data_cnt <=#Tp 4'h0;
		end else if(wr_fifo) begin
			data_cnt <=#Tp data_cnt + 4'h1;
		end
	end


	// Multiplexing data that is stored to 64-byte fifo depends on the mode of operation and frame format
	always @(extended_mode or ide or data_cnt or header_cnt or  header_len or
		storing_header or id or rtr1 or rtr2 or data_len or
		tmp_fifo[0] or tmp_fifo[2] or tmp_fifo[4] or tmp_fifo[6] or
		tmp_fifo[1] or tmp_fifo[3] or tmp_fifo[5] or tmp_fifo[7]) begin

		casex ({storing_header, extended_mode, ide, header_cnt}) /* synthesis parallel_case */
			6'b1_1_1_000  : data_for_fifo = {1'b1, rtr2, 2'h0, data_len};  // extended mode, extended format header
			6'b1_1_1_001  : data_for_fifo = id[28:21];                     // extended mode, extended format header
			6'b1_1_1_010  : data_for_fifo = id[20:13];                     // extended mode, extended format header
			6'b1_1_1_011  : data_for_fifo = id[12:5];                      // extended mode, extended format header
			6'b1_1_1_100  : data_for_fifo = {id[4:0], 3'h0};               // extended mode, extended format header
			6'b1_1_0_000  : data_for_fifo = {1'b0, rtr1, 2'h0, data_len};  // extended mode, standard format header
			6'b1_1_0_001  : data_for_fifo = id[10:3];                      // extended mode, standard format header
			6'b1_1_0_010  : data_for_fifo = {id[2:0], rtr1, 4'h0};         // extended mode, standard format header
			6'b1_0_x_000  : data_for_fifo = id[10:3];                      // normal mode                    header
			6'b1_0_x_001  : data_for_fifo = {id[2:0], rtr1, data_len};     // normal mode                    header
			default       : data_for_fifo = tmp_fifo[data_cnt - {1'b0, header_len}]; // data
		endcase
	end




	// Instantiation of the RX fifo module
	can_fifo i_can_fifo
	(
		.clk(clk),
		.rst(rst),

		.wr(wr_fifo),

		.data_in(data_for_fifo),
		.addr(addr[5:0]),
		.data_out(data_out),
		.fifo_selected(fifo_selected),

		.reset_mode(reset_mode),
		.release_buffer(release_buffer),
		.extended_mode(extended_mode),
		.overrun(overrun),
		.info_empty(info_empty),
		.info_cnt(rx_message_counter)

		`ifdef CAN_BIST
		,
		.mbist_si_i(mbist_si_i),
		.mbist_so_o(mbist_so_o),
		.mbist_ctrl_i(mbist_ctrl_i)
		`endif
	);


	// Transmitting error frame.
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			error_frame <= 1'b0;
		//end else if(reset_mode || error_frame_ended || go_overload_frame) begin
		end else if(set_reset_mode || error_frame_ended || go_overload_frame) begin
			error_frame <=#Tp 1'b0;
		end else if(go_error_frame) begin
			error_frame <=#Tp 1'b1;
		end
	end



	always @(posedge clk or posedge rst) begin
		if(rst) begin
			error_cnt1 <= 3'd0;
		end else if(error_frame_ended | go_error_frame | go_overload_frame) begin
			error_cnt1 <=#Tp 3'd0;
		end else if(error_frame & tx_point & (error_cnt1 < 3'd7)) begin
			error_cnt1 <=#Tp error_cnt1 + 1'b1;
		end
	end



	assign error_flag_over = ((~node_error_passive) & sample_point & (error_cnt1 == 3'd7) | node_error_passive  & sample_point & (passive_cnt == 3'h6)) & (~enable_error_cnt2);


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			error_flag_over_latched <= 1'b0;
		end else if(error_frame_ended | go_error_frame | go_overload_frame) begin
			error_flag_over_latched <=#Tp 1'b0;
		end else if(error_flag_over) begin
			error_flag_over_latched <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			enable_error_cnt2 <= 1'b0;
		end else if(error_frame_ended | go_error_frame | go_overload_frame) begin
			enable_error_cnt2 <=#Tp 1'b0;
		end else if(error_frame & (error_flag_over & sampled_bit)) begin
			enable_error_cnt2 <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			error_cnt2 <= 3'd0;
		end else if(error_frame_ended | go_error_frame | go_overload_frame) begin
			error_cnt2 <=#Tp 3'd0;
		end else if(enable_error_cnt2 & tx_point) begin
			error_cnt2 <=#Tp error_cnt2 + 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			delayed_dominant_cnt <= 3'h0;
		end else if(enable_error_cnt2 | go_error_frame | enable_overload_cnt2 | go_overload_frame) begin
			delayed_dominant_cnt <=#Tp 3'h0;
		end else if(sample_point & (~sampled_bit) & ((error_cnt1 == 3'd7) | (overload_cnt1 == 3'd7))) begin
			delayed_dominant_cnt <=#Tp delayed_dominant_cnt + 1'b1;
		end
	end


	// passive_cnt
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			passive_cnt <= 3'h1;
		end else if(error_frame_ended | go_error_frame | go_overload_frame | first_compare_bit) begin
			passive_cnt <=#Tp 3'h1;
		end else if(sample_point & (passive_cnt < 3'h6)) begin
			if(error_frame & (~enable_error_cnt2) & (sampled_bit == sampled_bit_q)) begin
				passive_cnt <=#Tp passive_cnt + 1'b1;
			end else begin
				passive_cnt <=#Tp 3'h1;
			end
		end
	end


	// When comparing 6 equal bits, first is always equal
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			first_compare_bit <= 1'b0;
		end else if(go_error_frame) begin
			first_compare_bit <=#Tp 1'b1;
		end else if(sample_point) begin
			first_compare_bit <= 1'b0;
		end
	end


	// Transmitting overload frame.
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			overload_frame <= 1'b0;
		end else if(overload_frame_ended | go_error_frame) begin
			overload_frame <=#Tp 1'b0;
		end else if(go_overload_frame) begin
			overload_frame <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			overload_cnt1 <= 3'd0;
		end else if(overload_frame_ended | go_error_frame | go_overload_frame) begin
			overload_cnt1 <=#Tp 3'd0;
		end else if(overload_frame & tx_point & (overload_cnt1 < 3'd7)) begin
			overload_cnt1 <=#Tp overload_cnt1 + 1'b1;
		end
	end


	assign overload_flag_over = sample_point & (overload_cnt1 == 3'd7) & (~enable_overload_cnt2);


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			enable_overload_cnt2 <= 1'b0;
		end else if(overload_frame_ended | go_error_frame | go_overload_frame) begin
			enable_overload_cnt2 <=#Tp 1'b0;
		end else if(overload_frame & (overload_flag_over & sampled_bit)) begin
			enable_overload_cnt2 <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			overload_cnt2 <= 3'd0;
		end else if(overload_frame_ended | go_error_frame | go_overload_frame) begin
			overload_cnt2 <=#Tp 3'd0;
		end else if(enable_overload_cnt2 & tx_point) begin
			overload_cnt2 <=#Tp overload_cnt2 + 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			overload_request_cnt <= 2'b0;
		end else if(go_error_frame | go_rx_id1) begin
			overload_request_cnt <=#Tp 2'b0;
		end else if(overload_request & overload_frame) begin
			overload_request_cnt <=#Tp overload_request_cnt + 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			overload_frame_blocked <= 1'b0;
		end else if(go_error_frame | go_rx_id1) begin
			overload_frame_blocked <=#Tp 1'b0;
		end else if(overload_request & overload_frame & overload_request_cnt == 2'h2) begin  // This is a second sequential overload_request
			overload_frame_blocked <=#Tp 1'b1;
		end
	end


	assign send_ack = (~tx_state) & rx_ack & (~err) & (~listen_only_mode);



	always @(reset_mode or node_bus_off or tx_state or go_tx or bit_de_stuff_tx or tx_bit or tx_q or
		send_ack or go_overload_frame or overload_frame or overload_cnt1 or
		go_error_frame or error_frame or error_cnt1 or node_error_passive) begin

		if(reset_mode | node_bus_off) begin
			// Reset or node_bus_off
			tx_next = 1'b1;
		end else begin
			if(go_error_frame | error_frame) begin
				// Transmitting error frame
				if(error_cnt1 < 3'd6) begin
					if(node_error_passive) begin
						tx_next = 1'b1;
					end else begin
						tx_next = 1'b0;
					end
				end else begin
					tx_next = 1'b1;
				end
			end else if(go_overload_frame | overload_frame) begin
				// Transmitting overload frame
				if(overload_cnt1 < 3'd6) begin
					tx_next = 1'b0;
				end else begin
					tx_next = 1'b1;
				end
			end else if(go_tx | tx_state) begin
				// Transmitting message
				tx_next = ((~bit_de_stuff_tx) & tx_bit) | (bit_de_stuff_tx & (~tx_q));
			end else if(send_ack) begin
				// Acknowledge
				tx_next = 1'b0;
			end else begin
				tx_next = 1'b1;
			end
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx <= 1'b1;
		end else if(reset_mode) begin
			tx <= 1'b1;
		end else if(tx_point) begin
			tx <=#Tp tx_next;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_q <=#Tp 1'b0;
		end else if(reset_mode) begin
			tx_q <=#Tp 1'b0;
		end else if(tx_point) begin
			tx_q <=#Tp tx & (~go_early_tx_latched);
		end
	end


	/* Delayed tx point */
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_point_q <=#Tp 1'b0;
		end else if(reset_mode) begin
			tx_point_q <=#Tp 1'b0;
		end else begin
			tx_point_q <=#Tp tx_point;
		end
	end


	/* Changing bit order from [7:0] to [0:7] */
	can_ibo i_ibo_tx_data_0  (.di(tx_data_0),  .do(r_tx_data_0));
	can_ibo i_ibo_tx_data_1  (.di(tx_data_1),  .do(r_tx_data_1));
	can_ibo i_ibo_tx_data_2  (.di(tx_data_2),  .do(r_tx_data_2));
	can_ibo i_ibo_tx_data_3  (.di(tx_data_3),  .do(r_tx_data_3));
	can_ibo i_ibo_tx_data_4  (.di(tx_data_4),  .do(r_tx_data_4));
	can_ibo i_ibo_tx_data_5  (.di(tx_data_5),  .do(r_tx_data_5));
	can_ibo i_ibo_tx_data_6  (.di(tx_data_6),  .do(r_tx_data_6));
	can_ibo i_ibo_tx_data_7  (.di(tx_data_7),  .do(r_tx_data_7));
	can_ibo i_ibo_tx_data_8  (.di(tx_data_8),  .do(r_tx_data_8));
	can_ibo i_ibo_tx_data_9  (.di(tx_data_9),  .do(r_tx_data_9));
	can_ibo i_ibo_tx_data_10 (.di(tx_data_10), .do(r_tx_data_10));
	can_ibo i_ibo_tx_data_11 (.di(tx_data_11), .do(r_tx_data_11));
	can_ibo i_ibo_tx_data_12 (.di(tx_data_12), .do(r_tx_data_12));

	/* Changing bit order from [14:0] to [0:14] */
	can_ibo i_calculated_crc0 (.di(calculated_crc[14:7]), .do(r_calculated_crc[7:0]));
	can_ibo i_calculated_crc1 (.di({calculated_crc[6:0], 1'b0}), .do(r_calculated_crc[15:8]));


	assign basic_chain = {r_tx_data_1[7:4], 2'h0, r_tx_data_1[3:0], r_tx_data_0[7:0], 1'b0};
	assign basic_chain_data = {r_tx_data_9, r_tx_data_8, r_tx_data_7, r_tx_data_6, r_tx_data_5, r_tx_data_4, r_tx_data_3, r_tx_data_2};
	assign extended_chain_std = {r_tx_data_0[7:4], 2'h0, r_tx_data_0[1], r_tx_data_2[2:0], r_tx_data_1[7:0], 1'b0};
	assign extended_chain_ext = {r_tx_data_0[7:4], 2'h0, r_tx_data_0[1], r_tx_data_4[4:0], r_tx_data_3[7:0], r_tx_data_2[7:3], 1'b1, 1'b1, r_tx_data_2[2:0], r_tx_data_1[7:0], 1'b0};
	assign extended_chain_data_std = {r_tx_data_10, r_tx_data_9, r_tx_data_8, r_tx_data_7, r_tx_data_6, r_tx_data_5, r_tx_data_4, r_tx_data_3};
	assign extended_chain_data_ext = {r_tx_data_12, r_tx_data_11, r_tx_data_10, r_tx_data_9, r_tx_data_8, r_tx_data_7, r_tx_data_6, r_tx_data_5};

	always @(extended_mode or rx_data or tx_pointer or extended_chain_data_std or extended_chain_data_ext or rx_crc or r_calculated_crc or
		r_tx_data_0   or extended_chain_ext or extended_chain_std or basic_chain_data or basic_chain or
		finish_msg) begin

		if(extended_mode) begin
			if(rx_data) begin
				// data stage
				if(r_tx_data_0[0]) begin
					// Extended frame
					tx_bit = extended_chain_data_ext[tx_pointer];
				end else begin
					tx_bit = extended_chain_data_std[tx_pointer];
				end
			end else if(rx_crc) begin
				tx_bit = r_calculated_crc[tx_pointer];
			end else if(finish_msg) begin
				tx_bit = 1'b1;
			end else begin
				if(r_tx_data_0[0]) begin
					// Extended frame
					tx_bit = extended_chain_ext[tx_pointer];
				end else begin
					tx_bit = extended_chain_std[tx_pointer];
				end
			end
		end else begin
			// Basic mode
			if(rx_data) begin  // data stage
				tx_bit = basic_chain_data[tx_pointer];
			end else if(rx_crc) begin
				tx_bit = r_calculated_crc[tx_pointer];
			end else if(finish_msg) begin
				tx_bit = 1'b1;
			end else begin
				tx_bit = basic_chain[tx_pointer];
			end
		end
	end


	assign limited_tx_cnt_ext = tx_data_0[3] ? 6'h3f : ((tx_data_0[2:0] <<3) - 1'b1);
	assign limited_tx_cnt_std = tx_data_1[3] ? 6'h3f : ((tx_data_1[2:0] <<3) - 1'b1);

	assign rst_tx_pointer = ((~bit_de_stuff_tx) & tx_point & (~rx_data) &   extended_mode  &   r_tx_data_0[0]   & tx_pointer == 6'd38             ) |   // arbitration + control for extended format
				((~bit_de_stuff_tx) & tx_point & (~rx_data) &   extended_mode  & (~r_tx_data_0[0])  & tx_pointer == 6'd18             ) |   // arbitration + control for extended format
				((~bit_de_stuff_tx) & tx_point & (~rx_data) & (~extended_mode)                      & tx_pointer == 6'd18             ) |   // arbitration + control for standard format
				((~bit_de_stuff_tx) & tx_point &   rx_data  &   extended_mode                       & tx_pointer == limited_tx_cnt_ext) |   // data       (overflow is OK here)
				((~bit_de_stuff_tx) & tx_point &   rx_data  & (~extended_mode)                      & tx_pointer == limited_tx_cnt_std) |   // data       (overflow is OK here)
				(                     tx_point &   rx_crc_lim                                                                         ) |   // crc
				(go_rx_idle                                                                                                           ) |   // at the end
				(reset_mode                                                                                                           ) |
				(overload_frame                                                                                                       ) |
				(error_frame                                                                                                          ) ;

	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_pointer <= 6'h0;
		end else if(rst_tx_pointer) begin
			tx_pointer <=#Tp 6'h0;
		end else if(go_early_tx | (tx_point & (tx_state | go_tx) & (~bit_de_stuff_tx))) begin
			tx_pointer <=#Tp tx_pointer + 1'b1;
		end
	end


	assign tx_successful = transmitter & go_rx_inter & (~go_error_frame) & (~error_frame_ended) & (~overload_frame_ended) & (~arbitration_lost);


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			need_to_tx <= 1'b0;
		end else if(tx_successful | reset_mode | (abort_tx & (~transmitting)) | ((~tx_state) & tx_state_q & single_shot_transmission)) begin
			need_to_tx <=#Tp 1'h0;
		end else if(tx_request & sample_point) begin
			need_to_tx <=#Tp 1'b1;
		end
	end



	assign go_early_tx = (~listen_only_mode) & need_to_tx & (~tx_state) & (~suspend | (susp_cnt == 3'h7)) & sample_point & (~sampled_bit) & (rx_idle | last_bit_of_inter);
	assign go_tx       = (~listen_only_mode) & need_to_tx & (~tx_state) & (~suspend | (sample_point & (susp_cnt == 3'h7))) & (go_early_tx | rx_idle);

	// go_early_tx latched (for proper bit_de_stuff generation)
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			go_early_tx_latched <= 1'b0;
		end else if(reset_mode || tx_point) begin
			go_early_tx_latched <=#Tp 1'b0;
		end else if(go_early_tx) begin
			go_early_tx_latched <=#Tp 1'b1;
		end
	end



	// Tx state
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_state <= 1'b0;
		end else if(reset_mode | go_rx_inter | error_frame | arbitration_lost) begin
			tx_state <=#Tp 1'b0;
		end else if(go_tx) begin
			tx_state <=#Tp 1'b1;
		end
	end

	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_state_q <=#Tp 1'b0;
		end else if(reset_mode) begin
			tx_state_q <=#Tp 1'b0;
		end else begin
			tx_state_q <=#Tp tx_state;
		end
	end



	// Node is a transmitter
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			transmitter <= 1'b0;
		end else if(go_tx) begin
			transmitter <=#Tp 1'b1;
		end else if(reset_mode | go_rx_idle | suspend & go_rx_id1) begin
			transmitter <=#Tp 1'b0;
		end
	end



	// Signal "transmitting" signals that the core is a transmitting (message, error frame or overload frame). No synchronization is done meanwhile.
	// Node might be both transmitter or receiver (sending error or overload frame)
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			transmitting <= 1'b0;
		end else if(go_error_frame | go_overload_frame | go_tx | send_ack) begin
			transmitting <=#Tp 1'b1;
		end else if(reset_mode | go_rx_idle | (go_rx_id1 & (~tx_state)) | (arbitration_lost & tx_state)) begin
			transmitting <=#Tp 1'b0;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			suspend <= 1'b0;
		end else if(reset_mode | (sample_point & (susp_cnt == 3'h7))) begin
			suspend <=#Tp 1'b0;
		end else if(not_first_bit_of_inter & transmitter & node_error_passive) begin
			suspend <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			susp_cnt_en <= 1'b0;
		end else if(reset_mode | (sample_point & (susp_cnt == 3'h7))) begin
			susp_cnt_en <=#Tp 1'b0;
		end else if(suspend & sample_point & last_bit_of_inter) begin
			susp_cnt_en <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			susp_cnt <= 3'h0;
		end else if(reset_mode | (sample_point & (susp_cnt == 3'h7))) begin
			susp_cnt <=#Tp 3'h0;
		end else if(susp_cnt_en & sample_point) begin
			susp_cnt <=#Tp susp_cnt + 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			finish_msg <= 1'b0;
		end else if(go_rx_idle | go_rx_id1 | error_frame | reset_mode) begin
			finish_msg <=#Tp 1'b0;
		end else if(go_rx_crc_lim) begin
			finish_msg <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			arbitration_lost <= 1'b0;
		end else if(go_rx_idle | error_frame_ended) begin
			arbitration_lost <=#Tp 1'b0;
		end else if(transmitter & sample_point & tx & arbitration_field & ~sampled_bit) begin
			arbitration_lost <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			arbitration_lost_q <=#Tp 1'b0;
		end else begin
			arbitration_lost_q <=#Tp arbitration_lost;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			arbitration_field_d <=#Tp 1'b0;
		end else if(sample_point) begin
			arbitration_field_d <=#Tp arbitration_field;
		end
	end


	assign set_arbitration_lost_irq = arbitration_lost & (~arbitration_lost_q) & (~arbitration_blocked);


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			arbitration_cnt <= 5'h0;
		end else if(sample_point && !bit_de_stuff) begin
			if(arbitration_field_d) begin
				arbitration_cnt <=#Tp arbitration_cnt + 1'b1;
			end else begin
				arbitration_cnt <=#Tp 5'h0;
			end
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			arbitration_lost_capture <= 5'h0;
		end else if(set_arbitration_lost_irq) begin
			arbitration_lost_capture <=#Tp arbitration_cnt;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			arbitration_blocked <= 1'b0;
		end else if(read_arbitration_lost_capture_reg) begin
			arbitration_blocked <=#Tp 1'b0;
		end else if(set_arbitration_lost_irq) begin
			arbitration_blocked <=#Tp 1'b1;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			rx_err_cnt <= 9'h0;
		end else if(we_rx_err_cnt & (~node_bus_off)) begin
			rx_err_cnt <=#Tp {1'b0, data_in};
		end else if(set_reset_mode) begin
			rx_err_cnt <=#Tp 9'h0;
		end else begin
			if((~listen_only_mode) & (~transmitter | arbitration_lost)) begin
				if(go_rx_ack_lim & (~go_error_frame) & (~crc_err) & (rx_err_cnt > 9'h0)) begin
					if(rx_err_cnt > 9'd127) begin
						rx_err_cnt <=#Tp 9'd127;
					end else begin
						rx_err_cnt <=#Tp rx_err_cnt - 1'b1;
					end
				end else if(rx_err_cnt < 9'd128) begin
					if(go_error_frame & (~rule5)) begin                                                                                    // 1  (rule 5 is just the opposite then rule 1 exception
						rx_err_cnt <=#Tp rx_err_cnt + 1'b1;
					end else if( (error_flag_over & (~error_flag_over_latched) & sample_point & (~sampled_bit) & (error_cnt1 == 3'd7)  ) |  // 2
						(go_error_frame & rule5                                                                                    ) |  // 5
						(sample_point & (~sampled_bit) & (delayed_dominant_cnt == 3'h7)) ) begin                                        // 6

						rx_err_cnt <=#Tp rx_err_cnt + 4'h8;
					end
				end
			end
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			tx_err_cnt <= 9'h0;
		end else if(we_tx_err_cnt) begin
			tx_err_cnt <=#Tp {1'b0, data_in};
		end else begin
			if(set_reset_mode) begin
				tx_err_cnt <=#Tp 9'd128;
			end else if((tx_err_cnt > 9'd0) & (tx_successful | bus_free)) begin
				tx_err_cnt <=#Tp tx_err_cnt - 1'h1;
			end else if(transmitter & (~arbitration_lost)) begin
				if( (sample_point & (~sampled_bit) & (delayed_dominant_cnt == 3'h7)                                             ) |       // 6
					(go_error_frame & rule5                                                                                 ) |       // 4  (rule 5 is the same as rule 4)
					(go_error_frame & (~(transmitter & node_error_passive & ack_err)) & (~(transmitter & stuff_err &
					arbitration_field & sample_point & tx & (~sampled_bit)))                                                ) |       // 3
					(error_frame & rule3_exc1_2) ) begin                                                                              // 3

					tx_err_cnt <=#Tp tx_err_cnt + 4'h8;
				end
			end
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			node_error_passive <= 1'b0;
		end else if((rx_err_cnt < 128) & (tx_err_cnt < 9'd128)) begin
			node_error_passive <=#Tp 1'b0;
		end else if(((rx_err_cnt >= 128) | (tx_err_cnt >= 9'd128)) & (error_frame_ended | go_error_frame | (~reset_mode) & reset_mode_q) & (~node_bus_off)) begin
			node_error_passive <=#Tp 1'b1;
		end
	end


	assign node_error_active = ~(node_error_passive | node_bus_off);


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			node_bus_off <= 1'b0;
		end else if((rx_err_cnt == 9'h0) & (tx_err_cnt == 9'd0) & (~reset_mode) | (we_tx_err_cnt & (data_in < 8'd255))) begin
			node_bus_off <=#Tp 1'b0;
		end else if((tx_err_cnt >= 9'd256) | (we_tx_err_cnt & (data_in == 8'd255))) begin
			node_bus_off <=#Tp 1'b1;
		end
	end



	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bus_free_cnt <= 4'h0;
		end else if(sample_point) begin
			if(sampled_bit & bus_free_cnt_en & (bus_free_cnt < 4'd10)) begin
				bus_free_cnt <=#Tp bus_free_cnt + 1'b1;
			end else begin
				bus_free_cnt <=#Tp 4'h0;
			end
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bus_free_cnt_en <= 1'b0;
		end else if((~reset_mode) & reset_mode_q | node_bus_off_q & (~reset_mode)) begin
			bus_free_cnt_en <=#Tp 1'b1;
		end else if(sample_point & sampled_bit & (bus_free_cnt==4'd10) & (~node_bus_off)) begin
			bus_free_cnt_en <=#Tp 1'b0;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			bus_free <= 1'b0;
		end else if(sample_point & sampled_bit & (bus_free_cnt==4'd10) && waiting_for_bus_free) begin
			bus_free <=#Tp 1'b1;
		end else begin
			bus_free <=#Tp 1'b0;
		end
	end


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			waiting_for_bus_free <= 1'b1;
		end else if(bus_free & (~node_bus_off)) begin
			waiting_for_bus_free <=#Tp 1'b0;
		end else if(node_bus_off_q & (~reset_mode)) begin
			waiting_for_bus_free <=#Tp 1'b1;
		end
	end


	assign bus_off_on = ~node_bus_off;

	assign set_reset_mode = node_bus_off & (~node_bus_off_q);
	assign error_status = extended_mode? ((rx_err_cnt >= error_warning_limit) | (tx_err_cnt >= error_warning_limit))    :
		((rx_err_cnt >= 9'd96) | (tx_err_cnt >= 9'd96))                                ;

	assign transmit_status = transmitting  || (extended_mode && waiting_for_bus_free);
	assign receive_status  = extended_mode ? (waiting_for_bus_free || (!rx_idle) && (!transmitting)) :
		((!waiting_for_bus_free) && (!rx_idle) && (!transmitting));

	/* Error code capture register */
	always @(posedge clk or posedge rst) begin
		if(rst) begin
			error_capture_code <= 8'h0;
		end else if(read_error_code_capture_reg) begin
			error_capture_code <=#Tp 8'h0;
		end else if(set_bus_error_irq) begin
			error_capture_code <=#Tp {error_capture_code_type[7:6], error_capture_code_direction, error_capture_code_segment[4:0]};
		end
	end



	assign error_capture_code_segment[0] = rx_idle | rx_ide | (rx_id2 & (bit_cnt<6'd13)) | rx_r1 | rx_r0 | rx_dlc | rx_ack | rx_ack_lim | error_frame & node_error_active;
	assign error_capture_code_segment[1] = rx_idle | rx_id1 | rx_id2 | rx_dlc | rx_data | rx_ack_lim | rx_eof | rx_inter | error_frame & node_error_passive;
	assign error_capture_code_segment[2] = (rx_id1 & (bit_cnt>6'd7)) | rx_rtr1 | rx_ide | rx_id2 | rx_rtr2 | rx_r1 | error_frame & node_error_passive | overload_frame;
	assign error_capture_code_segment[3] = (rx_id2 & (bit_cnt>6'd4)) | rx_rtr2 | rx_r1 | rx_r0 | rx_dlc | rx_data | rx_crc | rx_crc_lim | rx_ack | rx_ack_lim | rx_eof | overload_frame;
	assign error_capture_code_segment[4] = rx_crc_lim | rx_ack | rx_ack_lim | rx_eof | rx_inter | error_frame | overload_frame;
	assign error_capture_code_direction  = ~transmitting;


	always @(bit_err or form_err or stuff_err) begin
		if(bit_err) begin
			error_capture_code_type[7:6] = 2'b00;
		end else if(form_err) begin
			error_capture_code_type[7:6] = 2'b01;
		end else if(stuff_err) begin
			error_capture_code_type[7:6] = 2'b10;
		end else begin
			error_capture_code_type[7:6] = 2'b11;
		end
	end


	assign set_bus_error_irq = go_error_frame & (~error_capture_code_blocked);


	always @(posedge clk or posedge rst) begin
		if(rst) begin
			error_capture_code_blocked <= 1'b0;
		end else if(read_error_code_capture_reg) begin
			error_capture_code_blocked <=#Tp 1'b0;
		end else if(set_bus_error_irq) begin
			error_capture_code_blocked <=#Tp 1'b1;
		end
	end


endmodule

