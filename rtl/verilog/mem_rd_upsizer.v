module mem_rd_upsizer
  #(parameter WB_DW  = 32,
    parameter MEM_DW = 16,
    parameter AW  = 0)
   (input 	       clk,
    input 		rst,
    //Data & mask input stream
    input [AW-1:0] 	s_adr_i,
    input [MEM_DW-1:0] 	s_data_i,
    input 		s_valid_i,
    output 		s_ready_o,
    //Data & mask input stream
    output reg [AW-1:0] m_adr_o,
    output [WB_DW-1:0] 	m_data_o,
    output 		m_valid_o,
    input 		m_ready_i);

   stream_upsizer
     #(.DW_IN (MEM_DW),
       .SCALE  (WB_DW/MEM_DW))
   upsize_rddata
     (.clk       (clk),
      .rst       (rst),
      .s_data_i  (s_data_i),
      .s_valid_i (s_valid_i),
      .s_ready_o (), //FIXME: Error if 0
      .m_data_o  (m_data_o),
      .m_valid_o (m_valid_o),
      .m_ready_i (1'b1)); //Must always be ready for now

   always @(posedge clk) begin
      m_adr_o <= s_adr_i[AW-1:2];
   end
endmodule
