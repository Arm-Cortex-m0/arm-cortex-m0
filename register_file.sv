module register_file
	#(
		parameter DATA_N = 32,
				  SIZE = 16
	)
	(
		input logic clk,
		input logic rst,
		input logic[$clog2(SIZE)-1:0] r0_addr, // 來源1
		input logic[$clog2(SIZE)-1:0] r1_addr, // 來源2
		input logic[$clog2(SIZE)-1:0] w_addr,  // 目的
		input logic[$clog2(SIZE)-1:0] read_addr,  // 目的
		input logic[DATA_N-1:0] w_data, // 寫入的資料
		input logic wr_en, // 是否能寫入
		output logic [DATA_N-1:0] r0_data, r1_data, r2_data, 
		output logic [DATA_N-1:0] reg0_data, reg1_data, reg2_data, reg3_data, 
		                          reg4_data, reg5_data, reg6_data, reg7_data,
								  reg8_data, reg9_data, reg10_data, reg11_data,
								  reg12_data, reg_sp, reg_lr, reg_pc
	);
	
	// 16 個 32bits register_file
	logic [DATA_N-1:0] regs[SIZE-1:0];
	
	always_ff @(posedge clk) begin
		if (rst) begin
			regs[13] <= 0;
			// test case
			
			/*regs[1]  <= 4;
			regs[2]  <= 5;
			regs[3]  <= 6;*/
			
		end
		else if (wr_en) begin
			regs[w_addr] <= w_data;
		end
	end
	
	assign r0_data = regs[r0_addr];
	assign r1_data = regs[r1_addr];
	assign r2_data = regs[read_addr];
	assign reg0_data = regs[0];
	assign reg1_data = regs[1];
	assign reg2_data = regs[2];
	assign reg3_data = regs[3];
	assign reg4_data = regs[4];
	assign reg5_data = regs[5];
	assign reg6_data = regs[6];
	assign reg7_data = regs[7];
	assign reg8_data = regs[8];
	assign reg9_data = regs[9];
	assign reg10_data = regs[10];
	assign reg11_data = regs[11];
	assign reg12_data = regs[12];
	assign reg_sp = regs[13];
	assign reg_lr = regs[14];
	assign reg_pc = regs[15];
	
endmodule
