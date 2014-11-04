module mem_wr_downsizer
  #(parameter WB_DW  = 32,
    parameter MEM_DW = 16,
    parameter AW     = 32)
   (    input 		  clk,
    input 		  rst,
    //Command & address input stream
    input 		  s_we_i,
    input [AW-1:0] 	  s_adr_i,
    input 		  s_cmd_valid_i, 
    output 		  s_cmd_ready_o,
    //Data & mask input stream
    input [WB_DW-1:0] 	  s_data_i,
    input [WB_DW/8-1:0]   s_sel_i,
    input 		  s_data_valid_i,
    output 		  s_data_ready_o,
    //Command & address output stream
    output 		  m_we_o,
    output [AW+1:0] 	  m_adr_o,
    output 		  m_cmd_valid_o,
    input 		  m_cmd_ready_i,
    //Data & mask output stream
    output [MEM_DW-1:0]   m_data_o,
    output [MEM_DW/8-1:0] m_sel_o,
    output 		  m_data_valid_o,
    input 		  m_data_ready_i);

   reg 			  cmd_valid;
   reg 			  cmd_ready;
   
   stream_downsizer
     #(.DW_OUT (MEM_DW+MEM_DW/8),
       .SCALE  (WB_DW/MEM_DW))
   downsize_wrdata
     (.clk (clk),
      .rst (rst),
      .s_data_i  ({s_data_i[15:0],s_sel_i[1:0],s_data_i[31:16],s_sel_i[3:2]}),
      .s_valid_i (s_data_valid_i),
      .s_ready_o (s_data_ready_o),
      .m_data_o  ({m_data_o,m_sel_o}),
      .m_valid_o (m_data_valid_o),
      .m_ready_i (m_we_o & m_data_ready_i)); //Must always be ready for now

   reg [1:0] 		  ack_count;

   assign m_we_o = s_we_i;
   assign m_adr_o = s_we_i ? {s_adr_i,ack_count[0],1'b0} :
		    {s_adr_i,2'b00};

   assign m_cmd_valid_o = s_cmd_valid_i & cmd_valid;

   assign s_cmd_ready_o = s_we_i ? cmd_ready : m_cmd_ready_i;
   
   always @(posedge clk) begin
      if (rst) begin
	 ack_count <= 0;
	 cmd_valid <= 1'b0;
	 cmd_ready <= 1'b0;
      end else begin
	 cmd_ready <= 1'b0;
	 //ack_count <= 0;

	 if (s_cmd_valid_i) begin
	    if (s_we_i) begin
	       cmd_valid <= /*m_data_valid_o*/ 1'b1;
	       if (m_cmd_ready_i) begin
		  ack_count <= ack_count + 1;
		  if (ack_count[0] == 1'b0)
		    cmd_ready <= 1'b1;
	       end
	    end
	 end
      end
   end
endmodule
