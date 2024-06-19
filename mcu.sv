module mcu
	(
		input logic clk,
		input logic rst
	);
	
	`include "cmds.sv"
	
	/*-------------------------
	    Instruction Fetch
	-------------------------*/
	
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
	logic [31:0] alu_result;
	
	always_comb begin
		case (sel_pc)
			0: pc_next = pc + pc_offset;
			1: pc_next = alu_result; 
		endcase
	end
	
	// Program Counter
	logic load_pc;
	
	always_ff @(posedge clk) begin
		if (rst) begin          
			pc <= 0;
		end
		else if (load_pc) begin
			pc <= pc_next;
		end
	end
	
	// MAR(memmory address register) 
	logic load_mar;
	logic [15:0] mar_q;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			mar_q <= 0;
		end
		else if (load_mar) begin
			mar_q <= pc;
		end
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
	logic load_ir;
	logic [31:0] ir_q;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			ir_q <= 0;
		end
		else if(load_ir) begin
			ir_q[31:16] <= IR_0;
			ir_q[15:0]  <= IR_1;
		end
	end
	
	/*--------------------------
	    Instruction Decode
	--------------------------*/
	
	logic isThumb;
	assign isThumb = ir_q[31:27] != 5'b11101 &&
		             ir_q[31:27] != 5'b11110 &&
		             ir_q[31:27] != 5'b11111;
	
	logic [3:0] Rm, Rn, Rd, Rt;
	logic [31:0] imm;
	logic [6:0] cmd;
	
	inst_decode inst_decode_1(
		.isThumb(isThumb),
		.ir_q0(ir_q[31:16]),
		.ir_q1(ir_q[15:0]),
		.cmd(cmd), // 指令總類
		.Rm(Rm),
		.Rn(Rn),
		.Rd(Rd),
		.Rt(Rt),
		.imm(imm)
	);
	
	// Mux: 選擇第一個運算元
	logic [1:0] sel_op1;
	logic [3:0] r0_addr;
	
	always_comb begin
		unique case(sel_op1)
			0: r0_addr = Rm;
			1: r0_addr = Rn;
			2: r0_addr = 4'b1101; // 堆疊
		endcase
	end
	
	// Mux: 選擇第二個運算元
	logic sel_op2;
	logic [3:0] r1_addr;
	
	always_comb begin
		unique case(sel_op2)
			0: r1_addr = Rn;
		endcase
	end
	
	// Mux: 選擇目的地運算元
	logic sel_dest_reg;
	logic [3:0] w_addr;
	
	always_comb begin
		unique case(sel_dest_reg)
			0: w_addr = Rd;
			1: w_addr = Rt;
		endcase
	end
	
	// Register File
	logic [31:0] r0_data, r1_data, r2_data;
	logic [31:0] write_back_data;
	logic regsfile_wr_en;
	
	register_file #(
		.DATA_N(32),
		.SIZE(16)
	) register_file_u ( 
		.clk(clk),
		.r0_addr(r0_addr), // 來源1
		.r1_addr(r1_addr), // 來源2
		.w_addr(w_addr),   // 目的
		.w_data(write_back_data), // 寫入的資料
		.wr_en(regsfile_wr_en),   // 是否能寫入
		.r0_data(r0_data),
		.r1_data(r1_data),
		.r2_data(r2_data)  // 目的暫存器
	);
	
	// Fordwarding Unit_2
	
	
	/*--------------
	    Excute
	--------------*/
	
	// Fordwarding Unit_1
	
	// Mux: 選擇要傳入ALU的資料是來自 RegFile 還是 imm
	logic sel_reg_imm;
	logic [31:0] op2; 
	
	always_comb begin
		case(sel_reg_imm)
			0: op2 = r1_data;
			1: op2 = imm;
		endcase
	end
	
	// ALU
	logic [3:0] opcode;
	logic [3:0] apsr_flag_q, alu_flag_q;
	assign apsr_flag_q = 4'b0000;
	
	ALU ALU_1(    
		.opcode(opcode),
		.op1(r0_data),
		.op2(op2),
		.flag(apsr_flag_q),
		.flag_q(alu_flag_q),
		.result(alu_result)
	);
	
	// APSR
	logic setFlags;
	
	APSR APSR_1(	
		.clk(clk),
		.rst(rst),
		.setFlags(setFlags),
		.flag(alu_flag_q),
		.flag_q(apsr_flag_q)
	);

	/*---------------------
	    Memory Access
	---------------------*/
	
	logic ram_wr_en;    // decode 之後可得知
	logic [1:0] size;   // decode 之後可得知
	logic [31:0] mem_read;
	
	single_port_ram single_port_ram_1(
		.clk(clk),           
		.rst(rst),           
		.size(size),             // 資料大小選擇（00: 8-bit, 01: 16-bit, 10: 32-bit）
		.write_enable(ram_wr_en),  
		.address(alu_result),    // 記憶體位址 
		.write_data(r2_data), 
		.read_data(mem_read)      
	);
	
	// test:
		// LDR_reg: Rt <- Mem[Rn + Rm]
		// STR_reg: Mem[ Rn + Rm ] <- Rt(data)
	
	/*------------------
	    Write Back
	------------------*/
	
	// Mux: 選擇寫回暫存器的資料
	logic sel_write_back_data;
	
	always_comb begin
		case(sel_write_back_data)
			0: write_back_data = alu_result; // ALU 的計算結果
			1: write_back_data = mem_read;
		endcase
	end
	
	/*-------------------------------------
	    T1 ~ T5:  five stage pipeline
	    T6 ~ T10: interrupt
 	-------------------------------------*/
	typedef enum {T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10} state_t;
	state_t ps, ns;
	
	// FSM
	always_ff @(posedge clk)begin
		if(rst) begin
			ps <= T0;
		end
		else begin
			ps <= ns;
		end
	end
	
	// Controller
	always_comb begin
		opcode = 0;
		// IF
		load_mar = 0;
		load_pc = 0;
		
		
		sel_inst_len = 1; // 假設目前為止指令皆為16bits
		sel_pc_offset = 0;
		sel_pc = 0;
		
		// ID
		load_ir = 0;
		sel_mem_0 = 0;  // 程式記憶體
		sel_mem_1 = 1;  // 程式記憶體
		
		sel_op1 = 0;
		sel_op2 = 0;
		sel_dest_reg = 0;
		
		// EX
		sel_reg_imm = 0;
		
		// MEM
		ram_wr_en = 0;
		size = 0;
		setFlags = 0;
		
		// WB
		sel_write_back_data = 0;
		regsfile_wr_en = 0; // register file
		
		ns = ps;
		case(ps)
			T0:
				begin
					ns = T1;
				end
			T1: // IF
				begin
					// 由上一個指令決定 (目前皆假設為16bits指令 且 無跳躍)
					sel_inst_len = 1;
					sel_pc_offset = 0;
					sel_pc = 0;
					// 
					load_pc = 1;
					load_mar = 1; 
					ns = T2;
				end
			T2: // ID
				begin
					// ID
					load_ir = 1; 
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
					case(cmd)
						MOV_imm8: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;
						end
						ADD_reg: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;
							
						end
						STR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
						end
						LDR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
						end
					endcase
					ns = T3;
				end
			T3: // EX
				begin
					// EX
					case(cmd)
						MOV_imm8: begin
							opcode = 8;
							sel_reg_imm = 1;
						end
						ADD_reg: begin
							opcode = 0;
							sel_reg_imm = 0;
						end
						STR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
							
							opcode = 0;	
							sel_reg_imm = 1;
						end		
						LDR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
						
							opcode = 0;
							sel_reg_imm = 1;
						end
					endcase
					ns = T4;
				end
			T4: // MEM
				begin
					case(cmd)
						MOV_imm8: begin
							opcode = 8;
							sel_reg_imm = 1;
							setFlags = 1; 
						end
						ADD_reg: begin
						    setFlags = 1; 
						end
						STR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
							sel_reg_imm = 1;
							
							ram_wr_en = 1;
							size = 2'b10;
							setFlags = 0;
						end		
						LDR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
							sel_reg_imm = 1;
							
							size = 2'b10;
							setFlags = 0;							
						end
					endcase
					ns = T5;
				end
			T5: // WB
				begin
					case(cmd)
						MOV_imm8: begin
							opcode = 8;
							sel_reg_imm = 1; 
							regsfile_wr_en = 1;
						end
						ADD_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						STR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
							sel_reg_imm = 1;
						
							sel_write_back_data = 0;
							regsfile_wr_en = 0;												
						end
						LDR_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
							sel_reg_imm = 1;
						
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
						end
					endcase					

					ns = T1;
				end
		endcase
	end

endmodule