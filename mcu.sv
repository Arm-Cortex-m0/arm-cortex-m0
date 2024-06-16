module mcu(
	input logic clk,
	input logic rst
);
	
	parameter DATA_N = 32,
	          SIZE = 16;
	
	// T1~T5: five stage pipeline、T6~T10: interrupt
	typedef enum {T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10} state_t;
	state_t ps, ns;
	
	// FSM
	always_ff @(posedge clk)begin
		if(rst)
			ps <= T0;
		else
			ps <= ns;
	end
	
	/*--------------------
	Instruction Fetch
	--------------------*/
	
	// Mux: 目前指令長度 ( byte )
	logic sel_inst_len;
	logic [2:0] inst_len;
	always_comb begin
		case(sel_inst_len)
			0: inst_len = 4; // 32 bits 
			1: inst_len = 2; // 16 bits 
		endcase
	end
	
	// Mux: 下一個要執行指令的地址位移量
	logic sel_pc_offset;
	logic [2:0] pc_offset;
	always_comb begin
		case(sel_pc_offset)
			0: pc_offset = inst_len;
			1: pc_offset = 0; // from branch offset !!!
		endcase
	end
	
	// Mux: 下一個要執行指令的地址
	logic sel_pc;
	logic [15:0] pc, pc_next;
	always_comb begin
		case (sel_pc)
			0: pc_next = pc + pc_offset;
			1: pc_next = 0; // from ALU result !!!
		endcase
	end
	
	// Program Counter
	logic load_pc;
	always_ff @(posedge clk) begin
		if (rst)          
			pc <= 0;
		else if (load_pc) 
			pc <= pc_next;
	end
	
	// MAR(memmory address register) 
	logic load_mar;
	logic [15:0] mar_q;
	always_ff @(posedge clk) begin
		if(rst) 
			mar_q <= 0;
		else if (load_mar) 
			mar_q <= pc;
	end
	
	// Program Rom
	logic sel_mem_1;
	logic [1:0] sel_mem_0;
	logic [15:0] IR_0, IR_1;
	Program_Rom Program_Rom_1(
		.Rom_addr_in(mar_q[15:2]),
		.pc_1(mar_q[1]),
		.sel_mem_1(sel_mem_1),
		.sel_mem_0(sel_mem_0),
		.IR_0(IR_0),
		.IR_1(IR_1)
	);
	
	// Instruction Register ( IF to ID )
	logic load_IF_ID_reg;
	logic [31:0] IF_ID_ir_q;
	always_ff @(posedge clk) begin
		if(rst) 
			IF_ID_ir_q <= 0;
		else if(load_IF_ID_reg) begin
			IF_ID_ir_q[31:16] <= IR_0;
			IF_ID_ir_q[15:0]  <= IR_1;
		end
	end
	
	/*--------------------
	Instruction Decode
	--------------------*/
	logic isThumb;
	assign isThumb = IF_ID_ir_q[31:27] != 5'b11101 &&
		IF_ID_ir_q[31:27] != 5'b11110 &&
		IF_ID_ir_q[31:27] != 5'b11111;
	
	logic [3:0] Rm, Rn, Rd;
	logic [31:0] imm;
	inst_decode inst_decode_1(
		.isThumb(isThumb),
		.ir_q0(IF_ID_ir_q[31:16]),
		.ir_q1(IF_ID_ir_q[15:0]),
		.cmd(),
		.Rm(Rm),
		.Rn(Rn),
		.Rd(Rd),
		.imm(imm)
	);
	
	// Register File
	logic [3:0] r0_addr, r1_addr, w_addr;
	logic [31:0] w_data, r0_data, r1_data;
	logic wr_en;
	register_file #(
		.DATA_N(32),
		.SIZE(16)
	) register_file_u ( 
		.clk(clk),
		.r0_addr(Rm),  // 來源1
		.r1_addr(Rn),  // 來源2
		.w_addr(Rd),   // 目的
		.w_data(w_data),   // 寫入的資料
		.wr_en(wr_en),     // 是否能寫入
		.r0_data(r0_data),
		.r1_data(r1_data)
	);
	
	// Fordwarding Unit_2
	
	
	/*--------------------
	Excute
	--------------------*/
	
	logic MOV_imm, ADDS_reg;
	assign MOV_imm = ir_q[31:27] == 5'b00100;
	assign ADDS_reg = ir_q[31:25] == 7'b0001100;
	
	// Fordwarding Unit_1
	
	
	
	// ALU
	logic [3:0] opcode;
	logic [3:0] flag, flag_q;
	logic [31:0]result;
	ALU ALU_1(    
		.opcode(opcode),
		.op1(r0_data),
		.op2(r1_data),
		.flag(flag),
		.flag_q(flag_q),
		.result(result)
	);

	/*--------------------
	Memory Access
	--------------------*/
	
	logic write_enable; // decode 之後可得知
	logic [1:0] size;   // decode 之後可得知
	logic [31:0] mem_read;
	single_port_ram single_port_ram_1(
		.clk(clk),           
		.rst(rst),           
		.size(size),       // 資料大小選擇（00: 8-bit, 01: 16-bit, 10: 32-bit）
		.write_enable(write_enable),  
		.address(result),  // 記憶體位址
		.write_data(),     // 根據不同指令選擇不同目的
		.read_data(mem_read)      
	);
	
	
	/*--------------------
	Write Back
	--------------------*/
	
	logic sel_write_back_data;
	logic [31:0] write_back_data;
	// Mux: 選擇寫回暫存器的資料
	always_comb begin
		case(sel_write_back_data)
			0: write_back_data = result; // ALU 的計算結果
			1: write_back_data = mem_read;
		endcase
	end
	
					 
	// Controller
	always_comb begin
		// IF
		load_mar = 0;
		
		load_IF_ID_reg = 0;
		// ID
		load_pc = 0;
		
		sel_inst_len = 1; // 假設目前為止指令皆為16bits
		sel_pc_offset = 0;
		sel_pc = 0;
		
		
		sel_mem_0 = 0;  // 程式記憶體
		sel_mem_1 = 1;  // 程式記憶體
		
		// EX
		
		ns = ps;
		case(ps)
			T0:
				ns = T1;
			T1: // IF
				begin
					load_mar = 1;
					// load_pc = 1;
					load_IF_ID_reg = 1; 
					ns = T2;
				end
			T2: // ID
				begin
					// IF
					// load_mar = 1;
					// load_IF_ID_reg = 1; 
					// ID
					load_pc = 1;
					if (isThumb) begin
						if (mar_q[1]) begin
							sel_mem_0 = 2;
							sel_mem_1 = 0;
						end
						else begin
							sel_mem_0 = 0;
							sel_mem_1 = 1;
						end
					end
					ns = T0;
				end
			T3: // EX
				begin
					
					// EX
					if(MOV_imm) begin
						opcode = 8;
					end
					if(ADDS_reg) begin
						opcode = 0;
					end
					ns = T0;
				end
			T4: // MEM
				begin
				
				end
			T5: // WB
				begin
				
				end
		endcase
	end

endmodule