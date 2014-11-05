/*
 * Copyright (c) 2011, Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
 * All rights reserved.
 *
 * Redistribution and use in source and non-source forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in non-source form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS WORK IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * WORK, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

module mem_wb_if #(
	parameter TECHNOLOGY = "GENERIC",
	parameter AW = 32,
		   parameter WB_DW = 32,
	parameter DQ_WIDTH = 16,
	parameter BUF_WIDTH = 3
)
(
	// Wishbone
	input 		      wb_clk,
	input 		      wb_rst,
	input [AW-1:0] 	      wb_adr_i,
	input [WB_DW-1:0]     wb_dat_i,
	input [WB_DW/8-1:0]   wb_sel_i,
	input 		      wb_we_i,
	input 		      wb_cyc_i,
	input 		      wb_stb_i,
	input [2:0] 	      wb_cti_i,
	input [1:0] 	      wb_bte_i,
	output [WB_DW:0]      wb_dat_o,
	output 		      wb_ack_o,

	// Internal interface
	input 		      sdram_rst,
	input 		      sdram_clk,

	output [AW-3:0]       ca_adr_o, //FIXME width = AW-clog2(WB_DW/8)
	output 		      ca_we_o,
	output 		      ca_valid_o,
	input 		      ca_ready_i,

	output [WB_DW-1:0]    dm_dat_o,
	output [WB_DW/8-1:0]  dm_sel_o,
	output 		      dm_valid_o,
	input 		      dm_ready_i,

	input [BUF_WIDTH-1:0] r_adr_i,
	input [WB_DW-1:0]     r_dat_i,
	input 		      r_vld_i,

	// Buffer write
	input [WB_DW-1:0]     bufw_adr_i,
	input [WB_DW-1:0]     bufw_dat_i,
	input [WB_DW/8-1:0]   bufw_sel_i,
	input 		      bufw_we_i
);

   reg [31:0] 		     wb_adr;
	reg  [31:0]			wb_dat;
	reg  [3:0]			wb_sel;
	reg				wb_read_ack;
	reg				wb_write_ack;
	wire [31:0]			next_wb_adr;

	reg				wb_write_bufram;
	reg				sdram_write_bufram;
	wire [3:0]			wb_bufram_we;
	wire [BUF_WIDTH-1:0]		wb_bufram_addr;
	wire [31:0]			wb_bufram_di;
	wire [3:0]			sdram_bufram_we;

	reg  [31:BUF_WIDTH+2]		buf_adr;
	reg  [(1<<BUF_WIDTH)-1:0]	buf_clean;
	reg  [(1<<BUF_WIDTH)-1:0]	buf_clean_r;
	reg  [(1<<BUF_WIDTH)-1:0]	buf_clean_wb;
	wire				bufhit;
	wire				next_bufhit;
	wire				bufw_hit;

	reg				read_req_wb;

	reg  [31:0]			adr_o_r;

	reg				first_req;

	reg  [2:0]			wb_state;

	wire				fifo_ca_ready;

	wire				fifo_dm_ready;
   
	localparam [2:0]
		IDLE	= 3'd0,
		READ	= 3'd1,
		WRITE	= 3'd2,
		REFILL	= 3'd3;


	localparam [2:0]
		CLASSIC      = 3'b000,
		CONST_BURST  = 3'b001,
		INC_BURST    = 3'b010,
		END_BURST    = 3'b111;

	localparam [1:0]
		LINEAR_BURST = 2'b00,
		WRAP4_BURST  = 2'b01,
		WRAP8_BURST  = 2'b10,
		WRAP16_BURST = 2'b11;

	localparam READ_BURSTS_PER_BUF = 2;

	assign wb_ack_o      = wb_read_ack | wb_write_ack;

	assign next_wb_adr   = (wb_bte_i == LINEAR_BURST) ?
			       (wb_adr_i[31:0] + 32'd4) :
			       (wb_bte_i == WRAP4_BURST ) ?
			       {wb_adr_i[31:4], wb_adr_i[3:0] + 4'd4} :
			       (wb_bte_i == WRAP8_BURST ) ?
			       {wb_adr_i[31:5], wb_adr_i[4:0] + 5'd4} :
			     /*(wb_bte_i == WRAP16_BURST) ?*/
			       {wb_adr_i[31:6], wb_adr_i[5:0] + 6'd4};

	assign bufhit	     = (buf_adr == wb_adr_i[31:BUF_WIDTH+2]) &
			       buf_clean_wb[wb_adr_i[BUF_WIDTH+1:2]];
	assign next_bufhit   = (buf_adr == next_wb_adr[31:BUF_WIDTH+2]) &
			       buf_clean_wb[next_wb_adr[BUF_WIDTH+1:2]];
	assign bufw_hit      = (bufw_adr_i[31:BUF_WIDTH+2] ==
				buf_adr[31:BUF_WIDTH+2]);

	assign wb_bufram_we  = bufw_we_i & bufw_hit ? bufw_sel_i :
			       wb_write_bufram ? wb_sel : 4'b0;

	assign wb_bufram_addr = bufw_we_i & bufw_hit ?
			        bufw_adr_i[BUF_WIDTH+1:2] :
			        wb_write_bufram ?
			        wb_adr[BUF_WIDTH+1:2] :
			        (wb_cti_i == INC_BURST) & wb_ack_o ?
			        next_wb_adr[BUF_WIDTH+1:2] :
			        wb_adr_i[BUF_WIDTH+1:2];
	assign wb_bufram_di   = bufw_we_i & bufw_hit ? bufw_dat_i : wb_dat;
	assign sdram_bufram_we = {4{r_vld_i}};

   wire [31:0] 				sdram_bufram_di = {r_dat_i[15:0],r_dat_i[31:16]};

bufram #(
	.TECHNOLOGY(TECHNOLOGY),
	.ADDR_WIDTH(BUF_WIDTH)
) bufram (
	.clk_a		(wb_clk),
	.addr_a		(wb_bufram_addr),
	.we_a		(wb_bufram_we),
	.di_a		(wb_bufram_di),
	.do_a		(wb_dat_o),

	.clk_b		(sdram_clk),
	.addr_b		(r_adr_i),
	.we_b		(sdram_bufram_we),
	.di_b		(sdram_bufram_di),
	.do_b		()
);

   wire [29:0] 				wb_adr_int;
   reg [1:0] burst_cnt;

   assign wb_adr_int = {wb_adr_i[31:5],burst_cnt[0]+wb_adr_i[4],wb_adr_i[3:2]};

   stream_dual_clock_fifo
     #(.AW(3),
       .DW(31))
   wrfifo_cmd_adr
     (.wr_rst	(wb_rst),
      .wr_clk	(wb_clk),
      .stream_s_data_i  ({wb_we_i, wb_adr_int}),
      .stream_s_valid_i (wb_write_ack | read_req_wb),
      .stream_s_ready_o (fifo_ca_ready),

      .rd_rst	(sdram_rst),
      .rd_clk	(sdram_clk),
      .stream_m_data_o  ({ca_we_o,ca_adr_o}),
      .stream_m_valid_o (ca_valid_o),
      .stream_m_ready_i (ca_ready_i));

   stream_dual_clock_fifo
     #(.AW(3),
       .DW(36))
   wrfifo_data_mask
     (.wr_rst	(wb_rst),
      .wr_clk	(wb_clk),
      .stream_s_data_i  ({wb_dat_i,  wb_sel_i}),
      .stream_s_valid_i (wb_write_ack),
      .stream_s_ready_o (fifo_dm_ready),

      .rd_rst	(sdram_rst),
      .rd_clk	(sdram_clk),
      .stream_m_data_o  ({dm_dat_o, dm_sel_o}),
      .stream_m_valid_o (dm_valid_o),
      .stream_m_ready_i (dm_ready_i));
   

	//
	// WB clock domain
	//
   reg buf_flush;
   
	always @(posedge wb_clk)
		if (wb_rst) begin
			wb_read_ack <= 1'b0;
			wb_write_ack <= 1'b0;
			wb_write_bufram <= 1'b0;
			read_req_wb <= 1'b0;
			first_req <= 1'b1;
			wb_adr <= 0;
			wb_dat <= 0;
			wb_sel <= 0;
			wb_state <= IDLE;
			buf_flush <= 1'b0;
			burst_cnt <= 0;
		end else begin
			wb_read_ack <= 1'b0;
			wb_write_ack <= 1'b0;
			wb_write_bufram <= 1'b0;
			buf_flush <= 1'b0;
			case (wb_state)
			IDLE: begin
				burst_cnt <= 0;
				wb_sel <= wb_sel_i;
				wb_dat <= wb_dat_i;
				wb_adr <= wb_adr_i;
				if (wb_cyc_i & wb_stb_i & !wb_we_i) begin
					if (bufhit) begin
						wb_read_ack <= 1'b1;
						wb_state <= READ;
					/*
					 * wait for the ongoing refill to finish
					 * until issuing a new request
					 */
					end else if (buf_clean_wb[wb_adr_i[BUF_WIDTH+1:2]] |
						     first_req) begin
						first_req <= 1'b0;
						read_req_wb <= 1'b1;
						wb_state <= REFILL;
					end
				end else if (wb_cyc_i & wb_stb_i & wb_we_i &
					     (bufhit & (&buf_clean_wb) | first_req |
					      buf_adr != wb_adr_i[31:BUF_WIDTH+2])) begin
					if (fifo_ca_ready & fifo_dm_ready)
						wb_write_ack <= 1'b1;

					if (bufhit)
						wb_write_bufram <= 1'b1;

					wb_state <= WRITE;
				end
			end

			READ: begin
				if (wb_cyc_i & wb_stb_i & !wb_we_i &
				   (wb_cti_i == INC_BURST) & next_bufhit) begin
					wb_read_ack <= 1'b1;
				end else begin
					wb_state <= IDLE;
				end
			end

			REFILL: begin
				buf_adr <= wb_adr[31:BUF_WIDTH+2];
				if (fifo_ca_ready) begin
					if(burst_cnt == READ_BURSTS_PER_BUF-1)
						read_req_wb <= 1'b0;
					else
						burst_cnt <= burst_cnt + 1;
					if (burst_cnt == 0)
						buf_flush <= 1'b1;
				end
				if (!(|buf_clean_wb) & !read_req_wb)
					wb_state <= IDLE;

			end

			WRITE: begin
				if (wb_cyc_i & wb_stb_i & wb_we_i) begin

					if (fifo_ca_ready &
					    fifo_dm_ready)
						wb_write_ack <= 1'b1;

					if (bufhit) begin
						wb_sel <= wb_sel_i;
						wb_dat <= wb_dat_i;
						wb_adr <= wb_adr_i;
						wb_write_bufram <= 1'b1;
					end
				end

				// TODO: burst writes
				if (wb_ack_o) begin
					wb_state <= IDLE;
					wb_write_ack <= 0;
					wb_write_bufram <= 0;
				end
			end
			endcase
		end

	always @(posedge wb_clk) begin
		buf_clean_r <= buf_clean;
		buf_clean_wb <= buf_clean_r;
	end

	//
	// SDRAM clock domain
	//
	always @(posedge sdram_clk)
		if (sdram_rst)
			buf_clean <= 0;
		else
			if (r_vld_i)
				buf_clean[r_adr_i] <= 1'b1;
			else if (buf_flush)
				buf_clean <= 0;

endmodule
