module cpu
	(
		input clk,
		input rst
	);
	
	/*--------------------
	Instruction Fetch
	--------------------*/
	
	// Mux: Select instruction length (the number of byte)
	logic sel_inst_len;
	logic [2:0] inst_len;
	always_comb begin
		case(sel_inst_len)
			0: inst_len = 4; // 32 bits 
			1: inst_len = 2; // 16 bits 
		endcase
	end
	
	// Mux: Select next instruction address
	logic sel_pc_offset;
	logic [2:0] pc_offset;
	always_comb begin
		case(sel_pc_offset)
			0: pc_offset = inst_len;
			1: pc_offset = 0; // from branch offset !!!
		endcase
	end
	
	// Mux: Select next pc address
	logic sel_pc;
	assign sel_pc = 0;
	always_comb begin
		case (sel_pc)
			0: pc_next = pc + pc_offset;
			1: pc_next = 0; // from ALU result !!!
		endcase
	end
	
	// Program Counter
	logic load_pc;
	logic [15:0] pc, pc_next;
	always_ff @(posedge clk) begin
		if (rst)          
			pc <= 0;
		else if (load_pc) 
			pc <= pc_next;
	end
	
	// MAR(memmory address register) T1 
	// 將PC值給ROM 去找他對應要執行的指令
	logic load_mar;
	logic [15:0] mar_q;
	always_ff @(posedge clk)
	begin
		if(rst) mar_q <= #1 16'b0;
		else if (load_mar) mar_q <= #1 pc;
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
	
	// Instruction Register
	logic load_ir;
	logic [31:0] ir_q;
	always_ff @(posedge clk)begin
		if(rst) ir_q <= 0;
		else if(load_ir)begin
			ir_q[31:16] <= IR_0;
			ir_q[15:0]  <= IR_1;
		end
	end
	
	
	
	// Instruction Decode
	assign isThumb = ir_q[31:27] != 5'b11101 &&
					 ir_q[31:27] != 5'b11110 &&
					 ir_q[31:27] != 5'b11111;
	
	
	//assign MOV = isThumb && ir_q[31:26] == 6'b010001 && ir_q[25:24] == 2'b10;
	//assign MOV_imm = isThumb && ir_q[31:27] == 5'b00100;
	//assign ADDS_reg = isThumb && ir_q[31:25] == 7'b0001100;
	assign MOV_imm = ir_q[31:27] == 5'b00100;
	assign ADDS_reg = ir_q[31:25] == 7'b0001100;
	
	logic [3:0] Rm;
	logic [3:0] Rn;
	logic [3:0] Rd;
	logic [31:0] imm;
	inst_decoder inst_decoder_1(
		.ir_q0(ir_q[31:16]),
		.ir_q1(ir_q[15:0]),
		.cmd(),
		.Rm(Rm),
		.Rn(Rn),
		.Rd(Rd),
		.imm(imm)
	);

	/*--------------------
	Instruction Decode
	--------------------*/
	
	// Register File
	logic [3:0] r0_addr, r1_addr, w_addr;
	logic [31:0] w_data, r0_data, r1_data;
	logic wr_en;
	register_file register_file_1(
		.clk(clk),
		.r0_addr(Rm), // 來源1
		.r1_addr(Rn), // 來源2
		.w_addr(Rd),  // 目的
		.w_data(w_data), // 寫入的資料
		.wr_en(wr_en), // 是否能寫入
		.r0_data(r0_data),
		.r1_data(r1_data)
	);
	
	/*--------------------
	Excute
	--------------------*/
	
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

	
	/*--------------------
	Write Back
	--------------------*/
	

	
	
	
	
	// Finite State Machine Controller
	typedef enum {START, IF, ID, EX, MEM, WB} state_t;
	state_t ps, ns;
	
	always_ff @(posedge clk)
		if (rst) ps <= START;
		else     ps <= ns;
		
	always_comb begin
		
		load_mar = 0;
	
	    sel_inst_len = 1; // 因為目前實作 thumb-1 指令，所以先預設 inst_length = 2
		load_ir = 0;
		load_pc = 0;
		wr_en = 0;
		ns = ps;
		sel_mem_0 = 0;  // 程式記憶體
		sel_mem_1 = 1;  // 程式記憶體
		case (ps)
			
			START: begin
				ns = IF;
				sel_mem_0 = 0;
				sel_mem_1 = 1;
			end
			
			IF: begin
				load_mar = 1;
				load_pc = 1; 
				ns = ID;
			end
			
			// read data from register file and decode
			ID: begin
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
				ns = EX;
			end
			
			EX: begin
				
				if(MOV_imm)begin
					opcode = 8;
					// 因為 Rm, Rn, Rd, imm 是從哪裡開始、長度是多少，都是不固定的
					// 所以才先在 FSM 裡面寫 ir_q
					// 之後的工作是要改成控制線的版本
					//w_addr = ir_q[26:24];
					//w_data = ir_q[23:16];
					//wr_en = 1;
				end
				if(ADDS_reg)begin
					//r0_addr = ir_q[8:6];
					//r1_addr = ir_q[5:3];
					opcode = 0;
					//w_data = result;
					//w_addr = ir_q[2:0];
					//wr_en = 1;
				end
				ns = MEM;
			end
			
			MEM:
				begin
					ns = WB;
				end
			
			WB:
				begin
					if(MOV_imm)begin
						w_data = ir_q[23:16];
						wr_en = 1;
					end
					if(ADDS_reg)begin
						w_data = result;
						wr_en = 1;
					end
					ns = IF;
				end
		endcase
	end

endmodule

// TODO: imm length p.79