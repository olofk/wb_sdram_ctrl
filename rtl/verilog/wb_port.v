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
	output		[31:0]	ca_adr_o,
	output 			ca_we_o,
	output 			ca_valid_o,
	input			ca_ready_i,

	output		[15:0]	dm_dat_o,
	output		[1:0]	dm_sel_o,
	output 			dm_valid_o,

	input		[31:0]	r_adr_i,
	input		[15:0]	r_dat_i,
	input			r_valid_i,

	// Buffer write
	input [31:0]		bufw_adr_i,
	input [31:0]		bufw_dat_i,
	input [3:0]		bufw_sel_i,
	input			bufw_we_i
);


   wire [29:0] 			mem_wb_if_ca_adr;
   wire 			mem_wb_if_ca_we;
   wire				mem_wb_if_ca_valid;
   wire				mem_wb_if_ca_ready;
   
   wire [31:0] 			mem_wb_if_dm_dat;
   wire [3:0] 			mem_wb_if_dm_sel;
   wire 			mem_wb_if_dm_valid;
   wire 			mem_wb_if_dm_ready;

   wire 			wr_downsizer_ca_we;
   wire [29:0] 			wr_downsizer_ca_adr;
   wire 			wr_downsizer_ca_valid;
   wire 			wr_downsizer_ca_ready;
   
   wire [15:0] 			wr_downsizer_dm_dat;
   wire [1:0] 			wr_downsizer_dm_sel;
   wire 			wr_downsizer_dm_valid;
   wire 			wr_downsizer_dm_ready;
   
   wire [BUF_WIDTH-1:0] 	rd_upsizer_r_adr;
   wire [31:0] 			rd_upsizer_r_dat;
   wire 			rd_upsizer_r_valid;

   mem_wb_if
     #(.TECHNOLOGY	(TECHNOLOGY),
       .BUF_WIDTH	(BUF_WIDTH))
   wb_if
     (// Wishbone
      .wb_clk	(wb_clk),
      .wb_rst	(wb_rst),
      .wb_adr_i	(wb_adr_i),
      .wb_stb_i	(wb_stb_i),
      .wb_cyc_i	(wb_cyc_i),
      .wb_cti_i	(wb_cti_i),
      .wb_bte_i	(wb_bte_i),
      .wb_we_i	(wb_we_i ),
      .wb_sel_i	(wb_sel_i),
      .wb_dat_i	(wb_dat_i),
      .wb_dat_o	(wb_dat_o),
      .wb_ack_o	(wb_ack_o),
      // Internal interface
      .sdram_clk	(sdram_clk),
      .sdram_rst	(sdram_rst),

      .ca_adr_o		(mem_wb_if_ca_adr),
      .ca_we_o		(mem_wb_if_ca_we ),
      .ca_valid_o	(mem_wb_if_ca_valid),
      .ca_ready_i	(mem_wb_if_ca_ready),

      .dm_dat_o		(mem_wb_if_dm_dat),
      .dm_sel_o		(mem_wb_if_dm_sel),
      .dm_valid_o	(mem_wb_if_dm_valid),
      .dm_ready_i	(mem_wb_if_dm_ready),

      .r_adr_i		(rd_upsizer_r_adr),
      .r_dat_i		(rd_upsizer_r_dat),
      .r_vld_i		(rd_upsizer_r_valid),
      // Buffer write
      .bufw_adr_i	(bufw_adr_i),
      .bufw_dat_i	(bufw_dat_i),
      .bufw_sel_i	(bufw_sel_i),
      .bufw_we_i	(bufw_we_i));


   mem_wr_downsizer
     #(.WB_DW  (32),
       .MEM_DW (16),
       .AW     (30))
   wr_downsizer
     (.clk	      (sdram_clk),
      .rst	      (sdram_rst),

      .s_we_i         (mem_wb_if_ca_we),
      .s_adr_i        (mem_wb_if_ca_adr),
      .s_cmd_valid_i  (mem_wb_if_ca_valid),
      .s_cmd_ready_o  (mem_wb_if_ca_ready),

      .s_data_i       (mem_wb_if_dm_dat),
      .s_sel_i        (mem_wb_if_dm_sel),
      .s_data_valid_i (mem_wb_if_dm_valid),
      .s_data_ready_o (mem_wb_if_dm_ready),

      .m_we_o         (wr_downsizer_ca_we),
      .m_adr_o        (wr_downsizer_ca_adr),
      .m_cmd_valid_o  (wr_downsizer_ca_valid),
      .m_cmd_ready_i  (wr_downsizer_ca_ready),

      .m_data_o       (wr_downsizer_dm_dat),
      .m_sel_o        (wr_downsizer_dm_sel),
      .m_data_valid_o (wr_downsizer_dm_valid),
      .m_data_ready_i (wr_downsizer_dm_ready));

   assign ca_we_o    = wr_downsizer_ca_we;
   assign ca_adr_o   = wr_downsizer_ca_adr;
   assign ca_valid_o = wr_downsizer_ca_valid;
   assign wr_downsizer_ca_ready = ca_ready_i;

   assign dm_dat_o    = wr_downsizer_dm_dat;
   assign dm_sel_o    = wr_downsizer_dm_sel;
   assign dm_valid_o  = wr_downsizer_dm_valid;
   assign wr_downsizer_dm_ready = ca_ready_i;
   
   mem_rd_upsizer
     #(.WB_DW (32),
       .MEM_DW (16),
       .AW (BUF_WIDTH+2))
   upsize_rddata
     (.clk (sdram_clk),
      .rst (sdram_rst),
      .s_adr_i   (r_adr_i[BUF_WIDTH+1:0]),
      .s_data_i  (r_dat_i),
      .s_valid_i (r_valid_i),
      .s_ready_o (), //FIXME: Error if 0
      .m_adr_o   (rd_upsizer_r_adr),
      .m_data_o  (rd_upsizer_r_dat),
      .m_valid_o (rd_upsizer_r_valid),
      .m_ready_i (1'b1)); //Must always be ready for now

endmodule
