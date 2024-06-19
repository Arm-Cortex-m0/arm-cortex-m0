module register_file
	#(
		parameter DATA_N = 32,
				  SIZE = 16
	)
	(
		input logic clk,
		input logic[$clog2(SIZE)-1:0] r0_addr, // 來源1
		input logic[$clog2(SIZE)-1:0] r1_addr, // 來源2
		input logic[$clog2(SIZE)-1:0] w_addr,  // 目的
		input logic[DATA_N-1:0] w_data, // 寫入的資料
		input logic wr_en, // 是否能寫入
		output logic [DATA_N-1:0] r0_data, r1_data, r2_data
	);
	
	// 16 個 32bits register_file
	logic [DATA_N-1:0] regs[SIZE-1:0];
	
	always_ff @(posedge clk) begin
		if (wr_en)
			regs[w_addr] <= w_data;
	end
	
	assign r0_data = regs[r0_addr];
	assign r1_data = regs[r1_addr];
	assign r2_data = regs[w_addr];
	
endmodule
