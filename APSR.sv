module APSR
(	
	input logic  clk,
	input logic  rst,
	input logic  setFlags,
	input logic  [3:0]flag,
	output logic [3:0]flag_q
);
	logic[3:0] r_data;
	
	always_ff @(posedge clk)
		begin
			if(rst)
				r_data <= 4'b0000;
			else if(setFlags)
				r_data <= flag;
		end
	
	 assign flag_q = r_data;
endmodule