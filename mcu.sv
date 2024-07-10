module mcu
	(
		input logic clk,
		input logic rst
	);
	
	`include "cmds.sv"
	
	/*-------------------------
	    Instruction Fetch
	-------------------------*/
	
	// Mux: 下一個要執行指令的地址位移量
	logic sel_pc_offset;
	logic [2:0] pc_offset_;
	
	always_comb begin
		case(sel_pc_offset)
			0: pc_offset_ = 2; // 16 bits 
			1: pc_offset_ = 4; // 32 bits 
			2: pc_offset_ = 0; // from branch offset !!!
		endcase
	end
	
	// Mux: 下一個要執行指令的地址
	logic sel_pc;
	logic [15:0] pc, pc_next;
	logic [31:0] alu_result;
	
	always_comb begin
		case (sel_pc)
			0: pc_next = pc + pc_offset_;
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
	
	// MAR ( memmory address register )  
	/*logic load_mar;
	logic [15:0] mar;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			mar <= 0;
		end
		else if (load_mar) begin
			mar <= pc;
		end
	end*/
	
	// Program Rom
	logic sel_mem_1;
	logic [1:0] sel_mem_0;
	logic [15:0] IR_0, IR_1;
	
	Program_Rom Program_Rom_1(
		.Rom_addr_in(pc[15:2]),
		.pc_1(pc[1]),
		.sel_mem_1(sel_mem_1),
		.sel_mem_0(sel_mem_0),
		.IR_0(IR_0),
		.IR_1(IR_1)
	);
	
	// Instruction Register ( IF to ID Register )
	logic load_IF_ID_reg;
	logic [31:0] tmp;
	logic [31:0] IF_ID_reg;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			tmp <= 0;
		end
		else if(load_IF_ID_reg) begin
			tmp[31:16] <= IR_0;
			tmp[15:0]  <= IR_1;
		end
	end
	
	always_ff @(negedge clk) begin
		IF_ID_reg <= tmp;
	end
	
	/*--------------------------
	    Instruction Decode
	--------------------------*/
	
	logic thumb2_flag; // 1: 指令為 32bits 要做合併 ()
	logic isThumb; // 0: 32 bits 1: 16 bits
	assign isThumb = IF_ID_reg[31:27] != 5'b11101 &&
		             IF_ID_reg[31:27] != 5'b11110 &&
		             IF_ID_reg[31:27] != 5'b11111;
	
	logic [3:0] Rm, Rn, Rd, Rt;
	logic [31:0] imm;
	logic [6:0] cmd;
	
	inst_decode inst_decode_1(
		.isThumb(isThumb),
		.ir_q0(IF_ID_reg[31:16]),
		.ir_q1(IF_ID_reg[15:0]),
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
			3: r0_addr = 4'b1111; // PC
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
	logic [3:0] MEM_WB_w_addr;
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
		.w_addr(regsfile_wr_en ? MEM_WB_w_addr : w_addr),   // 目的
		.w_data(write_back_data), // 寫入的資料
		.wr_en(regsfile_wr_en),   // 是否能寫入
		.r0_data(r0_data),
		.r1_data(r1_data),
		.r2_data(r2_data)  // 目的暫存器
	);
	
	// Fordwarding Unit_2
	
	// ID to EX Register ( 暫時放在這裡 之後會單獨寫檔案 )
	logic load_ID_EX_reg;
	logic [3:0] ID_EX_w_addr;
	logic [31:0] ID_EX_r0_data, ID_EX_r1_data, ID_EX_r2_data, ID_EX_imm;
	
	always_ff @(posedge clk) begin 
		if (load_ID_EX_reg) begin
			ID_EX_r0_data <= r0_data;
			ID_EX_r1_data <= r1_data;
			ID_EX_r2_data <= r2_data;
			ID_EX_imm <= imm;
			ID_EX_w_addr <= w_addr;
		end
	end
	
	/*--------------
	    Excute
	--------------*/
	
	// Fordwarding Unit_1
	
	// Mux: 選擇要傳入 ALU 的資料是來自 RegFile 還是 imm
	logic sel_reg_imm;
	logic [31:0] op2; 
	
	always_comb begin
		case(sel_reg_imm) 
			0: op2 = ID_EX_r1_data;
			1: op2 = ID_EX_imm;
		endcase
	end
	
	// ALU
	logic [4:0] opcode;
	logic [3:0] apsr_flag_q, alu_flag_q;
	assign apsr_flag_q = 4'b0000;
	
	ALU ALU_1(    
		.opcode(opcode),
		.op1(ID_EX_r0_data),
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
	
	// EX to MEM Register alu_result ID_EX_r2_data
	logic load_EX_MEM_reg;
	logic [3:0] EX_MEM_w_addr;
	logic [31:0] EX_MEM_alu_result, EX_MEM_r2_data;
	
	always_ff @(posedge clk) begin
		if (load_EX_MEM_reg) begin
			EX_MEM_alu_result <= alu_result;
			EX_MEM_r2_data <= ID_EX_r2_data;
			EX_MEM_w_addr <= ID_EX_w_addr;
		end
	end
	
	
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
		.address(EX_MEM_alu_result),   // 記憶體位址 
		.write_data(EX_MEM_r2_data), 
		.read_data(mem_read)      
	);
	
	// MEM to WB Register
	logic load_MEM_WB_reg;
	logic [31:0] MEM_WB_alu_result, MEM_WB_mem_read;
	
	always_ff @(posedge clk) begin
		if (load_MEM_WB_reg) begin
			MEM_WB_mem_read <= mem_read;
			MEM_WB_alu_result <= EX_MEM_alu_result;
			MEM_WB_w_addr <= EX_MEM_w_addr;
		end
	end
	
	/*------------------
	    Write Back
	------------------*/
	
	// Mux: 選擇寫回暫存器的資料
	logic [1:0] sel_write_back_data;
	
	always_comb begin
		case(sel_write_back_data)
			2'b00: write_back_data = MEM_WB_alu_result; // ALU 的計算結果
			2'b01: write_back_data = MEM_WB_mem_read;
			2'b10: write_back_data = {{24{MEM_WB_mem_read[7]}}, MEM_WB_mem_read}; // 8 bits sign extend
			2'b11: write_back_data = {{16{MEM_WB_mem_read[15]}}, MEM_WB_mem_read}; // 16 bits sign extend
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
		//load_mar = 0;
		load_pc = 0;
		// 假設目前為止指令皆為 16 bits
		sel_pc_offset = 0;
		sel_pc = 0;
		
		load_IF_ID_reg = 0;
		
		// ID	
		sel_mem_0 = 0;  // 程式記憶體
		sel_mem_1 = 1;  // 程式記憶體
		
		sel_op1 = 0;
		sel_op2 = 0;
		sel_dest_reg = 0;
		
		load_ID_EX_reg = 0;
		
		// EX
		sel_reg_imm = 0;
		
		load_EX_MEM_reg = 0;
		
		// MEM
		ram_wr_en = 0;
		size = 0;
		setFlags = 0;
		
		load_MEM_WB_reg = 0;
		
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
					// 目前皆假設為16bits指令 且 無跳躍
					sel_pc_offset = 0;
					sel_pc = 0;
					
					load_pc = 1;
					// load_mar = 1; 
					
					load_IF_ID_reg = 1;
					
					if (isThumb) begin
						if (pc[1]) begin
							sel_mem_0 = 2;
							sel_mem_1 = 0;
						end
						else begin
							sel_mem_0 = 0;
							sel_mem_1 = 1;
						end
					end
					
					ns = T2;
				end
			T2: // ID
				begin
					// ID
					load_ID_EX_reg = 1;
					case(cmd)
						LSL_imm5: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;
						end
						LSR_imm5: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;						
						end
						ASR_imm5: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;						
						end
						ADD_reg: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;
						end
						SUB_reg: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;						
						end
						ADD_imm3: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 0;								
						end
						SUB_imm3: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 0;							
						end
						MOV_imm8: begin
							sel_op1 = 0;
							sel_op2 = 0;
							sel_dest_reg = 0;
						end
						CMP_imm8: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0; // 未使用 
							sel_dest_reg = 0; // 未使用
						end
						ADD_imm8: begin
							sel_op1 = 1; // Rdn
							sel_op2 = 0;
							sel_dest_reg = 0;							
						end
						SUB_imm8: begin
							sel_op1 = 1; // Rdn
							sel_op2 = 0;
							sel_dest_reg = 0;						
						end
						
						RSB_imm: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0; // 未使用
							sel_dest_reg = 0;							
						end
						
						MVN_reg: begin
							sel_op1 = 0;
							sel_op2 = 0; // 未使用 
							sel_dest_reg = 0;						
						end
						
						STR_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt						
						end						
						STRH_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt						
						end						
						STRB_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt						
						end
						LDRSB_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt							
						end						
						LDR_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt							
						end
						LDRH_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt									
						end						
						LDRB_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt									
						end						
						LDRSH_reg: begin
							sel_op1 = 0; 
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt									
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
						STRB_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt
						end
						LDRB_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt						
						end
						STRH_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt						
						end						
						LDRH_imm5: begin
							sel_op1 = 1; // Rn
							sel_op2 = 0;
							sel_dest_reg = 1; // Rt						
						end
						STR_imm8_SP: begin
							sel_op1 = 2; // SP
							sel_op2 = 0; // 未使用 
							sel_dest_reg = 1; // Rt	
						end
						LDR_imm8_SP: begin
							sel_op1 = 2; // SP
							sel_op2 = 0; // 未使用 
							sel_dest_reg = 1; // Rt						
						end
						ADR: begin
							sel_op1 = 3; // PC
							sel_op2 = 0; // 未使用 
							sel_dest_reg = 0; // Rd							
						end
						ADD_SP_imm8: begin
							sel_op1 = 2; // SP
							sel_op2 = 0; // 未使用 
							sel_dest_reg = 0; // Rd
						end
						
					endcase
					
					ns = T3;
				end
			T3: // EX
				begin
					// EX
					load_EX_MEM_reg	= 1;
					case(cmd)
						LSL_imm5: begin
							opcode = LSL_OP;
							sel_reg_imm = 1;
						end
						LSR_imm5: begin
							opcode = LSR_OP;
							sel_reg_imm = 1;					
						end
						ASR_imm5: begin
							opcode = ASR_OP;
							sel_reg_imm = 1;							
						end
						ADD_reg: begin
							opcode = ADD_OP;
							sel_reg_imm = 0;
						end
						SUB_reg: begin
							opcode = SUB_OP2;
							sel_reg_imm = 0;					
						end
						ADD_imm3: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;						
						end
						SUB_imm3: begin
							opcode = SUB_OP1;
							sel_reg_imm = 1;							
						end
						MOV_imm8: begin
							opcode = MOV_OP;
							sel_reg_imm = 1;
						end
						CMP_imm8: begin
							opcode = SUB_OP1;
							sel_reg_imm = 1;
						end
						ADD_imm8: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;							
						end
						SUB_imm8: begin
							opcode = SUB_OP1;
							sel_reg_imm = 1;						
						end
						AND_reg: begin
							opcode = AND_OP;
							sel_reg_imm = 0;						
						end
						EOR_reg: begin
							opcode = XOR_OP;
							sel_reg_imm = 0;
						end
						LSL_reg: begin
							opcode = LSL_OP;
							sel_reg_imm = 0;
						end
						LSR_reg: begin
							opcode = LSR_OP;
							sel_reg_imm = 0;
						end
						ASR_reg : begin
							opcode = ASR_OP;
							sel_reg_imm = 0;						
						end
						ADC_reg: begin
							opcode = ADD_FLAG_OP;
							sel_reg_imm = 0;						
						end
						SBC_reg: begin
							opcode = SUB_FLAG_OP1;
							sel_reg_imm = 0;
						end
						ROR_reg: begin
							opcode = ROR_OP;
							sel_reg_imm = 0;
						end
						TST_reg: begin
							opcode = AND_OP;
							sel_reg_imm = 0;							
						end
						RSB_imm: begin
							opcode = SUB_OP2; // imm(0) - Rn
							sel_reg_imm = 1;						
						end
						CMP_reg: begin
							opcode = SUB_OP1;
							sel_reg_imm = 0;
						end
						CMN_reg: begin
							opcode = ADD_OP;
							sel_reg_imm = 0;						
						end
						ORR_reg: begin
							opcode = OR_OP;
							sel_reg_imm = 0;						
						end
						MUL_reg: begin
							opcode = MUL_OP;
							sel_reg_imm = 0;							
						end
						BIC_reg: begin
							opcode = BIC_OP;
							sel_reg_imm = 0;
						end
						MVN_reg: begin
							opcode = BNOT_OP;
							sel_reg_imm = 0;						
						end
						
						STR_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end						
						STRH_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end						
						STRB_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end
						LDRSB_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end						
						LDR_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end						
						LDRH_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end						
						LDRB_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end						
						LDRSH_reg: begin
							opcode = ADD_OP;	
							sel_reg_imm = 0;						
						end
						STR_imm5: begin
							opcode = ADD_OP;	
							sel_reg_imm = 1;
						end		
						LDR_imm5: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;
						end						
						STRB_imm5: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;
						end
						LDRB_imm5: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;						
						end
						STRH_imm5: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;						
						end
						LDRH_imm5: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;						
						end
						STR_imm8_SP: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;							
						end
						LDR_imm8_SP: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;						
						end
						ADR: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;						
						end
						ADD_SP_imm8: begin
							opcode = ADD_OP;
							sel_reg_imm = 1;					
						end
						
					endcase
					
					ns = T4;
				end
			T4: // MEM
				begin
					load_MEM_WB_reg = 1;
					case(cmd)
						LSL_imm5: begin
							setFlags = 1;
						end
						LSR_imm5: begin
							setFlags = 1;
						end
						ASR_imm5: begin
							setFlags = 1;
						end
						MOV_imm8: begin
							setFlags = 1; 
						end
						CMP_imm8: begin
							setFlags = 1; 
						end
						ADD_reg: begin
						    setFlags = 1; 
						end
						SUB_reg: begin
							setFlags = 1; 
						end
						ADD_imm3: begin
							setFlags = 1;
						end
						SUB_imm3: begin
							setFlags = 1;
						end
						ADD_imm8: begin
							setFlags = 1;
						end
						SUB_imm8: begin
							setFlags = 1;
						end
						AND_reg: begin
							setFlags = 1;
						end
						EOR_reg: begin
							setFlags = 1;
						end
						LSL_reg: begin
							setFlags = 1;
						end
						LSR_reg: begin
							setFlags = 1;
						end
						ASR_reg: begin
							setFlags = 1;
						end
						ADC_reg: begin
							setFlags = 1;
						end
						SBC_reg: begin
							setFlags = 1;
						end
						ROR_reg: begin
							setFlags = 1;
						end
						TST_reg: begin
							setFlags = 0;
						end
						RSB_imm: begin
							setFlags = 1;
						end
						CMP_reg: begin
							setFlags = 0;
						end
						CMN_reg: begin
							setFlags = 0;
						end
						ORR_reg: begin
							setFlags = 1;
						end
						MUL_reg: begin
							setFlags = 1;
						end
						BIC_reg: begin
							setFlags = 1;
						end
						
						MVN_reg: begin
							setFlags = 1;
						end
						
						STR_reg: begin
							ram_wr_en = 1;
							size = 2'b10; // 32 bits
							setFlags = 0;
						end
						STRH_reg: begin
							ram_wr_en = 1;
							size = 2'b01; // 16 bits
							setFlags = 0;
						end						
						STRB_reg: begin
							ram_wr_en = 1;
							size = 2'b00; // 8 bits
							setFlags = 0;
						end
						LDRSB_reg: begin
							size = 2'b00; // 8 bits
							setFlags = 0;						
						end
						LDR_reg: begin
							size = 2'b10; // 32 bits
							setFlags = 0;						
						end						
						LDRH_reg: begin
							size = 2'b01; // 16 bits
							setFlags = 0;						
						end						
						LDRB_reg: begin
							size = 2'b00; // 8 bits
							setFlags = 0;						
						end						
						LDRSH_reg: begin
							size = 2'b01; // 16 bits
							setFlags = 0;						
						end
						STR_imm5: begin
							ram_wr_en = 1;
							size = 2'b10; // 32 bits
							setFlags = 0;
						end		
						LDR_imm5: begin
							size = 2'b10; // 32 bits
							setFlags = 0;							
						end						
						STRB_imm5: begin
							ram_wr_en = 1;
							size = 2'b00; // 8 bits
							setFlags = 0;							
						end
						LDRB_imm5: begin
							size = 2'b00; // 8 bits
							setFlags = 0;						
						end
						STRH_imm5: begin
							ram_wr_en = 1;
							size = 2'b01; // 16 bits
							setFlags = 0;							
						end
						LDRH_imm5: begin
							size = 2'b01; // 16 bits
							setFlags = 0;						
						end						
						STR_imm8_SP: begin
							size = 2'b10; // 32 bits
							setFlags = 0;						
						end
						LDR_imm8_SP: begin
							size = 2'b10; // 32 bits
							setFlags = 0;					
						end
						ADR: begin
							setFlags = 0;	
						end
						ADD_SP_imm8: begin
							setFlags = 0;
						end
						
					endcase
					ns = T5;
				end
			T5: // WB
				begin
					case(cmd)
						LSL_imm5: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						LSR_imm5: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;					
						end
						ASR_imm5: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						MOV_imm8: begin		
							regsfile_wr_en = 1;
						end
						CMP_imm8: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;	 // 不寫入暫存器
						end
						ADD_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						SUB_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;						
						end
						ADD_imm3: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						SUB_imm3: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						ADD_imm8: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;									
						end
						SUB_imm8: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;								
						end
						AND_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						EOR_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						LSL_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;						
						end
						LSR_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;								
						end
						ASR_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;								
						end
						ADC_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						SBC_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						ROR_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						TST_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0; // 捨棄結果								
						end
						RSB_imm: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1; 					
						end
						CMP_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0; // 捨棄結果							
						end
						CMN_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0; // 捨棄結果							
						end
						ORR_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1; 				
						end
						MUL_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;						
						end
						BIC_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						
						MVN_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;					
						end
						
						STR_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;							
						end						
						STRH_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;							
						end						
						STRB_reg: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;							
						end
						LDRSB_reg: begin
							sel_write_back_data = 2; // 8 bits sign extend
							regsfile_wr_en = 1;						
						end						
						LDR_reg: begin
							sel_write_back_data = 1;
							regsfile_wr_en = 1;						
						end
						LDRH_reg: begin // 自動補0
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end						
						LDRB_reg: begin // 自動補0
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end						
						LDRSH_reg: begin 
							sel_write_back_data = 3; // 16 bits sign extend
							regsfile_wr_en = 1;							
						end
						STR_imm5: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;												
						end
						LDR_imm5: begin
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
						end						
						STRB_imm5: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;
						end
						LDRB_imm5: begin // 自動補0
							sel_write_back_data = 1;
							regsfile_wr_en = 1;						
						end
						STRH_imm5: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;						
						end
						LDRH_imm5: begin // 自動補0
							sel_write_back_data = 1;
							regsfile_wr_en = 1;						
						end
						STR_imm8_SP: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 0;							
						end
						LDR_imm8_SP: begin
							sel_write_back_data = 1;
							regsfile_wr_en = 1;								
						end
						ADR: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end						
						ADD_SP_imm8: begin
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						
					endcase					

					ns = T1;
				end
		endcase
	end

endmodule