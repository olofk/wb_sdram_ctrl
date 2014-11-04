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

module wb_port #(
	parameter TECHNOLOGY = "GENERIC",
	parameter DQ_WIDTH = 16,
	parameter BUF_WIDTH = 3
)
(
	// Wishbone
	input			wb_clk,
	input			wb_rst,
	input		[31:0]	wb_adr_i,
	input			wb_stb_i,
	input			wb_cyc_i,
	input		[2:0]	wb_cti_i,
	input		[1:0]	wb_bte_i,
	input			wb_we_i,
	input		[3:0]	wb_sel_i,
	input		[31:0]	wb_dat_i,
	output		[31:0]	wb_dat_o,
	output			wb_ack_o,

	// Internal interface
	input			sdram_rst,
	input			sdram_clk,
	input		[31:0]	adr_i,
	output		[31:0]	adr_o,
	input		[15:0]	dat_i,
	output		[15:0]	dat_o,
	output		[1:0]	sel_o,
	output 			acc_o,
	output 			dv_o,
	input			ack_i,
	input			vld_i,
	output 			we_o,

	// Buffer write
	input [31:0]		bufw_adr_i,
	input [31:0]		bufw_dat_i,
	input [3:0]		bufw_sel_i,
	input			bufw_we_i
);

	reg  [31:0]			wb_adr;
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
	wire [BUF_WIDTH-1:0] 		sdram_bufram_addr;
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

	wire				wrfifo_cmd_adr_s_ready;
	wire [30:0]			wrfifo_cmd_adr_m_data;
	wire				wrfifo_cmd_adr_m_valid;
	wire				wrfifo_cmd_adr_m_ready;

	wire [35:0]			wrfifo_data_mask_s_data;
	wire				wrfifo_data_mask_s_valid;
	wire				wrfifo_data_mask_s_ready;
	wire [35:0]			wrfifo_data_mask_m_data;
	wire				wrfifo_data_mask_m_valid;
	wire				wrfifo_data_mask_m_ready;
   
	wire [3:0]			sdram_sel;
	wire [31:0]			sdram_dat;

	wire [31:0]			dat_i_32;
	wire 				vld_i_32;

   
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
	assign sdram_bufram_we = {4{vld_i_32}};

	assign sdram_sel      = wrfifo_data_mask_m_data[3:0];
	assign sdram_dat      = wrfifo_data_mask_m_data[35:4];

   wire [31:0] 				sdram_bufram_di = {dat_i_32[15:0],dat_i_32[31:16]};

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
	.addr_b		(sdram_bufram_addr),
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
      .stream_s_ready_o (wrfifo_cmd_adr_s_ready),

      .rd_rst	(sdram_rst),
      .rd_clk	(sdram_clk),
      .stream_m_data_o  (wrfifo_cmd_adr_m_data),
      .stream_m_valid_o (wrfifo_cmd_adr_m_valid),
      .stream_m_ready_i (wrfifo_cmd_adr_m_ready));

stream_dual_clock_fifo #(
	.AW(3),
	.DW(36)
) wrfifo_data_mask (
	.wr_rst	(wb_rst),
	.wr_clk	(wb_clk),
	.stream_s_data_i  ({wb_dat_i,  wb_sel_i}),
	.stream_s_valid_i (wb_write_ack),
	.stream_s_ready_o (wrfifo_data_mask_s_ready),

	.rd_rst	(sdram_rst),
	.rd_clk	(sdram_clk),
	.stream_m_data_o  (wrfifo_data_mask_m_data),
	.stream_m_valid_o (wrfifo_data_mask_m_valid),
	.stream_m_ready_i (wrfifo_data_mask_m_ready));
   

   mem_wr_downsizer
     #(.WB_DW  (32),
       .MEM_DW (16),
       .AW     (30))
   wr_downsizer
     (.clk	      (sdram_clk),
      .rst	      (sdram_rst),

      .s_we_i         (wrfifo_cmd_adr_m_data[30]),
      .s_adr_i        (wrfifo_cmd_adr_m_data[29:0]),
      .s_cmd_valid_i  (wrfifo_cmd_adr_m_valid),
      .s_cmd_ready_o  (wrfifo_cmd_adr_m_ready),

      .s_data_i       (sdram_dat),
      .s_sel_i        (sdram_sel),
      .s_data_valid_i (wrfifo_data_mask_m_valid),
      .s_data_ready_o (wrfifo_data_mask_m_ready),

      .m_we_o         (we_o),
      .m_adr_o        (adr_o),
      .m_cmd_valid_o  (acc_o),
      .m_cmd_ready_i  (ack_i),

      .m_data_o       (dat_o),
      .m_sel_o        (sel_o),
      .m_data_valid_o (dv_o),
      .m_data_ready_i (ack_i));

   mem_rd_upsizer
     #(.WB_DW (32),
       .MEM_DW (16),
       .AW (BUF_WIDTH+2))
   upsize_rddata
     (.clk (sdram_clk),
      .rst (sdram_rst),
      .s_adr_i   (adr_i[BUF_WIDTH+1:0]),
      .s_data_i  (dat_i),
      .s_valid_i (vld_i),
      .s_ready_o (), //FIXME: Error if 0
      .m_adr_o   (sdram_bufram_addr),
      .m_data_o  (dat_i_32),
      .m_valid_o (vld_i_32),
      .m_ready_i (1'b1)); //Must always be ready for now

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
					if (wrfifo_cmd_adr_s_ready & wrfifo_data_mask_s_ready)
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
				if (wrfifo_cmd_adr_s_ready) begin
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

					if (wrfifo_cmd_adr_s_ready &
					    wrfifo_data_mask_s_ready)
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
			if (vld_i_32)
				buf_clean[sdram_bufram_addr] <= 1'b1;
			else if (buf_flush)
				buf_clean <= 0;

endmodule
