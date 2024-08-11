///////////////////////////////////////////////////////////////////////////////
// Copyright(C) NTOU CSE Arm Cortex-M0 Team. Open source License: MIT.
// ALL RIGHT RESERVED
// File name   : mcu.sv
// Author      : 劉時軒
// Date        : 2024-08-10
// Version     : 1.0
// Description :
//    This is a Arm Cortex-M0 Five Stage Pipeline MCU TOP Level
//    
// Parameter   :
//    All parameters are in cmds.sv
//
// IO Port     :
//    ...
//    ...
// Modification History:
//   Date     |   Author   |   Version   |   Change Description
//==============================================================================
//   24-08-10 |    劉時軒  |     1.0     |   Five Stage Pipeline
//   24-08-11 |    劉時軒  |     1.1     |   Fix BX BL instruction
// ...
////////////////////////////////////////////////////////////////////////////////

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
	logic [1:0] sel_pc_offset, 
	            ID_EX_sel_pc_offset; // 分支指令使用
	logic [31:0] pc_offset_, 
                 branch_offset_;
		
	always_comb begin
		unique casez ({ID_EX_sel_pc_offset, sel_pc_offset})
			4'b0000: pc_offset_ = 2; // 16 bits 
			4'b0001: pc_offset_ = 4; // 32 bits 
			4'b10??: pc_offset_ = branch_offset_; // 跳躍
		endcase
	end	
	
	// Mux: 下一個要執行指令的地址
	logic sel_pc, ID_EX_sel_pc;
	logic [31:0] pc, pc_next_,
                 alu_result;
	logic [31:0] ID_EX_r0_data;
	
	always_comb begin
		case (sel_pc | ID_EX_sel_pc) 
			0: pc_next_ = pc + pc_offset_;
			1: pc_next_ = ID_EX_r0_data; // TODO: 確認 sel_pc = 1 的指令
		endcase
	end
	
	// Program Counter
	logic load_pc, ID_EX_load_pc;
		
	always_ff @(posedge clk) begin
		if (rst) begin          
			pc <= 0;
		end // TODO: ID_EX_load_pc 會造成 PUSH POP LDM STM load_pc
		else if (load_pc | ID_EX_load_pc) begin
			pc <= pc_next_; 
		end
	end
	
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
	logic flush, ID_EX_flush; // 當跳躍發生
	logic [15:0] IF_ID_reg_IR_0,
	             IF_ID_reg_IR_1;
	logic [31:0] IF_ID_pc;
    
	always_ff @(posedge clk) begin
		if (ID_EX_flush) begin
			IF_ID_reg_IR_0 <= 0;
			IF_ID_reg_IR_1 <= 0;		
		end
		else if (load_IF_ID_reg) begin
			IF_ID_reg_IR_0 <= IR_0;
			IF_ID_reg_IR_1 <= IR_1;
			IF_ID_pc <= pc;
		end
	end
	
	/*--------------------------
	    Instruction Decode
	--------------------------*/
	
	// 指令解碼
	logic thumb1_flag; // 0: 32 bits 1: 16 bits
	assign thumb1_flag = IF_ID_reg_IR_0[15:11] != 5'b11101 &&
		                IF_ID_reg_IR_0[15:11] != 5'b11110 &&
		                IF_ID_reg_IR_0[15:11] != 5'b11111;
	
	logic [3:0]  Rm, Rn, Rd, Rt; // register
	logic [3:0]  cond; // branch condition
	logic [31:0] imm;  // immediate
	logic [6:0]  cmd;  // instruction
	logic [7:0]  SYSm; // system control
	logic [15:0] registers; // multi register
	
	inst_decode inst_decode_1(
		.isThumb(thumb1_flag),
		.ir_q0(IF_ID_reg_IR_0),
		.ir_q1(IF_ID_reg_IR_1),
		.cmd(cmd),
		.Rm(Rm),
		.Rn(Rn),
		.Rd(Rd),
		.Rt(Rt),
		.imm(imm),
		.cond(cond),
		.registers(registers),
		.SYSm(SYSm)
	);
	
	// Mux: 選擇第一個運算元	
	logic [1:0] sel_r0_addr;
	logic [3:0] r0_addr;
	logic [3:0] r1_addr;
	
	// TODO: 將 Stack、PC 在 inst_decode 先處理
	always_comb begin
		case(sel_r0_addr)
			2'b00: r0_addr = Rm;
			2'b01: r0_addr = Rn;
			2'b10: r0_addr = 4'b1101; // 堆疊
			2'b11: r0_addr = 4'b1111; // PC
		endcase
	end
	
	//  選擇第二個運算元
    assign r1_addr = Rn;
	
	// Mux: 選擇目的地運算元	
	logic sel_w_addr;
	logic [3:0] w_addr;
	
	always_comb begin
		case(sel_w_addr)
			0: w_addr = Rd;
			1: w_addr = Rt;
		endcase
	end
	
	// Mux: 選擇寫回 暫存器集 的 暫存器地址
	// control line
	logic pop; // POP
	logic stm_wr_en; // STM
	logic ldm_wr_en; // LDM
	logic MEM_WB_sp_wr_en; // STACK
	logic MEM_WB_regsfile_wr_en; // LAOD
	
	// write back addr
	logic [3:0] write_back_addr;
	logic [3:0] pop_reg_addr;
	logic [3:0] ldm_reg_addr;	
	logic [3:0] MEM_WB_w_addr;
	logic [3:0] base_reg_addr;
	
	always_comb begin
		unique casez({pop, MEM_WB_regsfile_wr_en, MEM_WB_sp_wr_en, stm_wr_en, ldm_wr_en})
			5'b00000: write_back_addr = w_addr; // STORE
			5'b0?001: write_back_addr = ldm_reg_addr; // LDM
			5'b0?010: write_back_addr = base_reg_addr; // STM LDM
			5'b0?100: write_back_addr = 4'b1101; // STACK
			5'b01000: write_back_addr = MEM_WB_w_addr; // LOAD
			5'b1?000: write_back_addr = pop_reg_addr; // POP
		endcase
	end
	
	// Register File
	logic [31:0] r0_data, r1_data, r2_data;
	logic [31:0] write_back_data;
	// 存取多個暫存器
	logic [31:0] reg0_data, reg1_data, reg2_data, reg3_data, 
		         reg4_data, reg5_data, reg6_data, reg7_data,
			     reg8_data, reg9_data, reg10_data, reg11_data,
				 reg12_data, reg_sp, reg_lr, reg_pc;
	// POP
	logic POP_STATE_regsfile_wr_en;
	// STM
	logic STM_STATE_regsfile_wr_en;		
	// LDM
	logic LDM_STATE_regsfile_wr_en;
	
	register_file #(
		.DATA_N(32),
		.SIZE(16)
	) register_file_u ( 
		.clk(clk),
		.rst(rst),
		.r0_addr(r0_addr), // 來源1
		.r1_addr(r1_addr), // 來源2
		.w_addr(write_back_addr), // 目的暫存器地址 寫入
		.read_addr(w_addr),       // 目的暫存器地址 讀取
		.w_data(write_back_data), // 寫入的資料
		.wr_en(MEM_WB_regsfile_wr_en | POP_STATE_regsfile_wr_en | STM_STATE_regsfile_wr_en | LDM_STATE_regsfile_wr_en),   // 是否能寫入
		.r0_data(r0_data),
		.r1_data(r1_data),
		.r2_data(r2_data),  // 目的暫存器
		.reg0_data(reg0_data), 
		.reg1_data(reg1_data), 
		.reg2_data(reg2_data), 
		.reg3_data(reg3_data), 
		.reg4_data(reg4_data), 
		.reg5_data(reg5_data), 
		.reg6_data(reg6_data), 
		.reg7_data(reg7_data), 
		.reg8_data(reg8_data), 
		.reg9_data(reg9_data), 
		.reg10_data(reg10_data),
		.reg11_data(reg11_data),
		.reg12_data(reg12_data),
		.reg_sp(reg_sp), // TODO: 需更改
		.reg_lr(reg_lr), // TODO: 需更改
		.reg_pc(reg_pc)  // TODO: 需更改
	);
	
	/*---------
	   Stack
	---------*/
	logic [3:0] stk_ptr;
	logic [31:0] stack_in,  // 堆疊輸入資料
				 stack_out; // 堆疊輸出資料	
				 
	// PUSH 依照暫存器編號 由大到小
	logic push;
	logic rst_push_reg, load_push_reg;
	logic [3:0] cnt_push_reg;
	logic [3:0] sel_push_addr;
	
	always_ff @(posedge clk) begin
		if (rst | rst_push_reg) begin
			cnt_push_reg <= 4'b1111;
		end
		else if (load_push_reg) begin
			cnt_push_reg <= cnt_push_reg - 1;
		end
	end
	
	always_comb begin
		unique case(sel_push_addr)
			4'b0000: stack_in = reg0_data;
			4'b0001: stack_in = reg1_data;
			4'b0010: stack_in = reg2_data;
			4'b0011: stack_in = reg3_data;
			4'b0100: stack_in = reg4_data;
			4'b0101: stack_in = reg5_data;
			4'b0110: stack_in = reg6_data;
			4'b0111: stack_in = reg7_data;
			4'b1000: stack_in = reg8_data;
			4'b1001: stack_in = reg9_data;
			4'b1010: stack_in = reg10_data;
			4'b1011: stack_in = reg11_data;
			4'b1100: stack_in = reg12_data;
			4'b1110: stack_in = reg_lr; // Link Register
		endcase
	end
	
	// POP 依照暫存器編號 由小到大
	logic rst_pop_reg, load_pop_reg;
	logic [3:0] cnt_pop_reg;
	
	always_ff @(posedge clk) begin
		if (rst | rst_pop_reg) begin
			cnt_pop_reg <= 4'b0000;
		end
		else if (load_pop_reg) begin
			cnt_pop_reg <= cnt_pop_reg + 1;
		end
	end
	
	logic [3:0] sel_pop_addr;
	always_comb begin
		unique case(sel_pop_addr)
			4'b0000:  pop_reg_addr = 4'b0000;
			4'b0001:  pop_reg_addr = 4'b0001;
			4'b0010:  pop_reg_addr = 4'b0010;
			4'b0011:  pop_reg_addr = 4'b0011;
			4'b0100:  pop_reg_addr = 4'b0100;
			4'b0101:  pop_reg_addr = 4'b0101;
			4'b0110:  pop_reg_addr = 4'b0110;
			4'b0111:  pop_reg_addr = 4'b0111;
			4'b1111:  pop_reg_addr = 4'b1111;
		endcase
	end
	
	Stack Stack_u(
		.clk(clk),
		.rst(rst),
		.stack_in(stack_in),
		.push(push),
		.pop(pop),
		.stk_ptr(stk_ptr), // 堆疊指標位址
		.stack_out(stack_out)
	);
	
	/*------------------
	   STM ( R0 ~ R7 )
	------------------*/
	logic rst_stm_reg, load_stm_reg;
	logic load_cnt_stm_regs;
	logic [2:0] cnt_stm_regs; // 使用的暫存器數量
	logic [3:0] cnt_stm_reg;
	
	always_ff @(posedge clk) begin
		if (rst | rst_stm_reg) begin
			cnt_stm_reg <= 4'b0000;
			cnt_stm_regs <= 3'b000;
		end
		if (load_stm_reg) begin
			cnt_stm_reg <= cnt_stm_reg + 1;
		end
		if (load_cnt_stm_regs) begin
			cnt_stm_regs <= cnt_stm_regs + 1;
		end
	end
	
	logic [3:0] sel_stm_addr;
	logic [31:0] stm_reg_data;
	always_comb begin
		unique case(sel_stm_addr)
			3'b000: stm_reg_data = reg0_data;
			3'b001: stm_reg_data = reg1_data;
			3'b010: stm_reg_data = reg2_data;
			3'b011: stm_reg_data = reg3_data;
			3'b100: stm_reg_data = reg4_data;
			3'b101: stm_reg_data = reg5_data;
			3'b110: stm_reg_data = reg6_data;
			3'b111: stm_reg_data = reg7_data;
		endcase
	end
	
	/*------------------
	   LDM ( R0 ~ R7 )
	------------------*/	
	logic rst_ldm_reg, load_ldm_reg;
	logic load_cnt_ldm_regs;
	logic [2:0] cnt_ldm_regs; // 使用的暫存器數量
	logic [3:0] cnt_ldm_reg;  // 遍歷所有暫存器
	
	always_ff @(posedge clk) begin
		if(rst | rst_ldm_reg) begin
			cnt_ldm_reg <= 4'b0000;
			cnt_ldm_regs <= 3'b000;
		end
		if(load_ldm_reg) begin
			cnt_ldm_reg <= cnt_ldm_reg + 1;
		end
		if(load_cnt_ldm_regs) begin 
			cnt_ldm_regs <= cnt_ldm_regs + 1;
		end
	end
	
	logic [3:0] sel_ldm_addr;
	always_comb begin
		unique case(sel_ldm_addr)
			4'b0000: ldm_reg_addr = 4'b0000;
			4'b0001: ldm_reg_addr = 4'b0001;
			4'b0010: ldm_reg_addr = 4'b0010;
			4'b0011: ldm_reg_addr = 4'b0011;
			4'b0100: ldm_reg_addr = 4'b0100;
			4'b0101: ldm_reg_addr = 4'b0101;
			4'b0110: ldm_reg_addr = 4'b0110;
			4'b0111: ldm_reg_addr = 4'b0111;
		endcase
	end
	
	/*------------------------------------------------------------------------
	       Fordwarding Unit_2 ( Solve  Register File Read After Write )
	------------------------------------------------------------------------*/	
	logic ID_data_hazard_op1, 
	      ID_data_hazard_op2,
		  ID_data_hazard_op3;
	
	logic [31:0] tmp_r0_data,
	             tmp_r1_data,
				 tmp_r2_data;

	always_comb begin
		ID_data_hazard_op1 = 0;
		if (MEM_WB_regsfile_wr_en && (MEM_WB_w_addr == r0_addr)) begin
			ID_data_hazard_op1 = 1;
		end
	end	  
	
	logic sel_reg_imm;	  
	always_comb begin
		ID_data_hazard_op2 = 0;
		if (MEM_WB_regsfile_wr_en && (MEM_WB_w_addr == r1_addr) && ~sel_reg_imm) begin // 非立即數
			ID_data_hazard_op2 = 1;		
		end
	end	 
	
	logic ram_wr_en;
	always_comb begin
		ID_data_hazard_op3 = 0;
		if (ram_wr_en && (MEM_WB_w_addr == r1_addr)) begin // 寫入記憶體
			ID_data_hazard_op3 = 1;		
		end
	end
	
	always_comb begin
		case(ID_data_hazard_op1)
			0: tmp_r0_data = r0_data;
			1: tmp_r0_data = write_back_data;
		endcase
	end
	
	always_comb begin
		unique case({ID_data_hazard_op2, sel_reg_imm})
			2'b00: tmp_r1_data = r1_data;
			2'b01: tmp_r1_data = imm;
			2'b10: tmp_r1_data = write_back_data;
		endcase
	end
	
	always_comb begin
		case(ID_data_hazard_op3)
			0: tmp_r2_data = r2_data;
			1: tmp_r2_data = write_back_data;
		endcase
	end
	
	/*--------------------------
	    ID to EX Register 
	--------------------------*/
	logic load_ID_EX_reg;
	
	// IF
	logic [3:0] ID_EX_w_addr;
	logic [31:0] ID_EX_r1_data, ID_EX_r2_data, ID_EX_imm;
	logic [31:0] ID_EX_pc;
	
	// EX
	logic setFlags, ID_EX_setFlags;
	logic sel_pc_reg, ID_EX_sel_pc_reg;
	logic ID_EX_sel_reg_imm;
	logic [4:0] opcode, ID_EX_opcode;
	
	// MEM
	logic ID_EX_ram_wr_en; 
	logic ram_rd_en, ID_EX_ram_rd_en; 
	logic [1:0] size, ID_EX_size;
	
	// WB
	logic regsfile_wr_en, ID_EX_regsfile_wr_en;
	logic [3:0] sel_write_back_data, ID_EX_sel_write_back_data;
	logic sp_wr_en, ID_EX_sp_wr_en;
	
	// PUSH POP LDM STM
	logic [15:0] ID_EX_registers;
	
	// Fordwarding ( Data hazard )
	logic  [3:0] ID_EX_r0_addr, ID_EX_r1_addr;
	
	/*----------------
	    Load Hazard
	----------------*/
	logic stall;
	always_comb begin
		stall = 0;
		if (ID_EX_ram_rd_en && ((ID_EX_w_addr == r0_addr) || (ID_EX_w_addr == r1_addr))) begin
			stall = 1;
		end
	end
	
	always_ff @(posedge clk) begin 
		if (rst) begin // 初始化控制線
			ID_EX_sel_pc_offset <= 0;
			ID_EX_sel_pc <= 0;
			ID_EX_load_pc <= 0;
			ID_EX_flush <= 0;
		end
		else if (stall || ID_EX_flush) begin
			// Fordwarding
			ID_EX_r0_addr <= 0;
			ID_EX_r1_addr <= 0;
			
			// Data
			ID_EX_r0_data <= 0;
			ID_EX_r1_data <= 0;
			ID_EX_r2_data <= 0;
			ID_EX_imm <= 0;
			
			// EX
			ID_EX_opcode <= 0;
			ID_EX_sel_reg_imm <= 0;
			ID_EX_setFlags <= 0;
			
			
			ID_EX_sel_pc_offset <= 0;
			ID_EX_sel_pc <= 0;
			ID_EX_load_pc <= 0;
			ID_EX_flush <= 0;
			
			// MEM
			ID_EX_ram_wr_en <= 0;
			ID_EX_ram_rd_en <= 0;
			ID_EX_size <= 0;
			
			//WB
			ID_EX_sel_write_back_data <= 0;
			ID_EX_regsfile_wr_en <= 0;
			ID_EX_w_addr <= 0;
		end
		else if (load_ID_EX_reg) begin
			// Fordwarding
			ID_EX_r0_addr <= r0_addr;
			ID_EX_r1_addr <= r1_addr;
			
			// Data
			ID_EX_r0_data <= tmp_r0_data;
			ID_EX_r1_data <= tmp_r1_data;
			ID_EX_r2_data <= r2_data;
			ID_EX_imm <= imm;
			
			// EX
			ID_EX_opcode <= opcode;
			ID_EX_sel_reg_imm <= sel_reg_imm;
			ID_EX_setFlags <= setFlags;
			
			ID_EX_sel_pc_reg <= sel_pc_reg;
			ID_EX_pc <= IF_ID_pc;
			ID_EX_sel_pc_offset <= sel_pc_offset;
			ID_EX_sel_pc <= sel_pc;
			ID_EX_load_pc <= load_pc;
			ID_EX_flush <= flush;
			
			// MEM
			ID_EX_ram_wr_en <= ram_wr_en;
			ID_EX_ram_rd_en <= ram_rd_en;
			ID_EX_size <= size;
			
			//WB
			ID_EX_sel_write_back_data <= sel_write_back_data;
			ID_EX_regsfile_wr_en <= regsfile_wr_en;
			ID_EX_w_addr <= w_addr;
			ID_EX_sp_wr_en <= sp_wr_en;
			
			// PUSH
			ID_EX_registers <= registers;
		end
	end
	
	/*--------------------
	    Muiti Registers 
	--------------------*/	
	logic load_registers;
	logic [15:0] multi_registers;
	logic [31:0] base_reg;
	
	always_ff @(posedge clk) begin
		if(load_registers) begin
			multi_registers <= registers;
			base_reg <= r0_data; // 最後會寫回基底暫存器
			base_reg_addr <= r0_addr; // 用來當作記憶體位址
		end
	end

	/*--------------
	    Excute
	--------------*/
	
	/*------------------------
	    Fordwarding Unit_1
	------------------------*/
	logic EX_data_hazard_op1,
		  EX_data_hazard_op2,
		  EX_data_hazard_op3,
		  MEM_data_hazard_op1,
		  MEM_data_hazard_op2,
		  MEM_data_hazard_op3;
	  
	logic EX_MEM_regsfile_wr_en;
	logic [3:0] EX_MEM_w_addr;
	
	always_comb begin
		EX_data_hazard_op1 = 0;
		MEM_data_hazard_op1 = 0;
		if (EX_MEM_regsfile_wr_en && (EX_MEM_w_addr == ID_EX_r0_addr)) begin
			EX_data_hazard_op1 = 1; 
		end			
		else if (MEM_WB_regsfile_wr_en && (MEM_WB_w_addr == ID_EX_r0_addr)) begin
			MEM_data_hazard_op1 = 1;
		end
	end
	
	always_comb begin
		EX_data_hazard_op2 = 0;
		MEM_data_hazard_op2 = 0;	
		if (EX_MEM_regsfile_wr_en && (EX_MEM_w_addr == ID_EX_r1_addr) && (~ID_EX_sel_reg_imm)) begin
			EX_data_hazard_op2 = 1; 
		end
		else if (MEM_WB_regsfile_wr_en && (MEM_WB_w_addr == ID_EX_r1_addr) && (~ID_EX_sel_reg_imm)) begin
			MEM_data_hazard_op2 = 1; 
		end	
	end
	
	always_comb begin
		EX_data_hazard_op3 = 0;
		MEM_data_hazard_op3 = 0;	
		if(ID_EX_ram_wr_en) begin
			if (EX_MEM_regsfile_wr_en && (EX_MEM_w_addr == ID_EX_w_addr)) begin
				EX_data_hazard_op3 = 1;
			end
			else if (MEM_WB_regsfile_wr_en && (MEM_WB_w_addr == ID_EX_w_addr)) begin
				MEM_data_hazard_op3 = 1;
			end
		end		
	end
	
	logic [31:0] op1, op2;
	logic [31:0] EX_MEM_alu_result;
	logic [31:0] MEM_WB_alu_result;
	
	always_comb begin
		unique case({ID_EX_sel_pc_reg, MEM_data_hazard_op1, EX_data_hazard_op1}) 
			3'b000: op1 = ID_EX_r0_data;
			3'b001: op1 = EX_MEM_alu_result;
			3'b010: op1 = write_back_data;   
			3'b100: op1 = pc; // ADR   
		endcase
	end
	
	always_comb begin
		unique case({MEM_data_hazard_op2, EX_data_hazard_op2}) 
			2'b00: op2 = ID_EX_r1_data; 
			2'b01: op2 = EX_MEM_alu_result; 
			2'b10: op2 = write_back_data;  // alu result / mem read
		endcase
	end
	
	logic [31:0] tmp_ID_EX_r2_data;
	always_comb begin
		unique case({MEM_data_hazard_op3, EX_data_hazard_op3}) 
			2'b00: tmp_ID_EX_r2_data = ID_EX_r2_data; 
			2'b01: tmp_ID_EX_r2_data = EX_MEM_alu_result; 
			2'b10: tmp_ID_EX_r2_data = write_back_data;
		endcase
	end
	
	always_comb begin
		unique casez({ID_EX_sel_reg_imm, MEM_data_hazard_op1, EX_data_hazard_op1}) 
			3'b000: branch_offset_ = ID_EX_r0_data;
			3'b001: branch_offset_ = EX_MEM_alu_result;
			3'b010: branch_offset_ = write_back_data; 
			3'b1??: branch_offset_ = ID_EX_imm;		
		endcase
	end
	
	// ALU
	logic [3:0] apsr_flag, alu_flag;
	
	ALU ALU_1(    
		.opcode(ID_EX_opcode),
		.op1(op1),
		.op2(op2),
		.flag(apsr_flag),
		.flag_q(alu_flag),
		.result(alu_result)
	);
	
	// APSR
	APSR APSR_1(	
		.clk(clk),
		.rst(rst),
		.setFlags(ID_EX_setFlags), 
		.flag(alu_flag),
		.flag_q(apsr_flag)
	);
	
	//  Conditional execution APSR N,Z,C,V 規格書 P.99
	logic sel_cond; // 1: 符合跳躍情形
	always_comb begin
		sel_cond = 0;
		unique case(cond) // TODO: 確認FLAG是否正確 (原apsr_flag)
			4'b0000: begin if(alu_flag[2] == 1) sel_cond = 1; end // BEQ 相等
			4'b0001: begin if(alu_flag[2] == 0) sel_cond = 1; end // BNE 不相等
			4'b0010: begin if(alu_flag[1] == 1) sel_cond = 1; end // BCS 無符號數大於或等於
			4'b0011: begin if(alu_flag[1] == 0) sel_cond = 1; end // BCC 無符號數小於      
			4'b0100: begin if(alu_flag[3] == 1) sel_cond = 1; end // BMI 負數              
			4'b0101: begin if(alu_flag[3] == 0) sel_cond = 1; end // BPL 正數或零          
			4'b0110: begin if(alu_flag[0] == 1) sel_cond = 1; end // BVS 溢出              
			4'b0111: begin if(alu_flag[0] == 0) sel_cond = 1; end // BVC 沒有溢出          
			4'b1000: begin if(alu_flag[2] == 0 && alu_flag[1] == 1) sel_cond = 1; end // BHI 無符號數大於
			4'b1001: begin if(alu_flag[1] == 0) sel_cond = 1; end // BLS 無符號數小於或等於    
			4'b1010: begin if(alu_flag[3] == alu_flag[2]) sel_cond = 1; end // BGE 有符號數大於或等於    
			4'b1011: begin if(alu_flag[3] != alu_flag[2]) sel_cond = 1; end // BLT 有符號數小於          
			4'b1100: begin if(alu_flag[2] == 0 && alu_flag[3] == alu_flag[0]) sel_cond = 1; end // BGT 有符號數大於     
			4'b1101: begin if(alu_flag[2] == 1) sel_cond = 1; end // BLE 有符號數小於或等於    
			4'b1110: begin sel_cond = 1; end // BAL 無條件執行            
		endcase
	end
	
	/*--------------------------
	    EX to MEM Register 
	--------------------------*/
	logic load_EX_MEM_reg;
	// Data
	logic [31:0] EX_MEM_r2_data;
	// MEM
	logic EX_MEM_ram_wr_en; 
	logic [1:0] EX_MEM_size;
	// WB
	logic [3:0] EX_MEM_sel_write_back_data;
	logic EX_MEM_sp_wr_en;
	// PUSH
	logic [15:0] EX_MEM_registers;
	
	always_ff @(posedge clk) begin
		if (load_EX_MEM_reg) begin
			// Data
			EX_MEM_alu_result <= alu_result;
			EX_MEM_r2_data <= tmp_ID_EX_r2_data;
			
			// MEM
			EX_MEM_ram_wr_en <= ID_EX_ram_wr_en;
			EX_MEM_size <= ID_EX_size;
			
			// WB
			EX_MEM_sel_write_back_data <= ID_EX_sel_write_back_data;
			EX_MEM_regsfile_wr_en <= ID_EX_regsfile_wr_en;
			EX_MEM_w_addr <= ID_EX_w_addr;
			EX_MEM_sp_wr_en <= ID_EX_sp_wr_en;
			
			// PUSH 
			EX_MEM_registers <= ID_EX_registers;
		end
	end
	
	/*---------------------
	    Memory Access
	---------------------*/
	logic [31:0] mem_read;
	logic [31:0] mem_address;
	// STM
	logic [1:0] STM_STATE_size;
	logic STM_STATE_ram_wr_en;
	// LDM
	logic [1:0] LDM_STATE_size;
	
	always_comb begin // TODO: 0 要改成 base register data
		unique case({load_cnt_ldm_regs, load_cnt_stm_regs})
			2'b00: mem_address = EX_MEM_alu_result;
			2'b01: mem_address = 0 + 4 * cnt_stm_regs; // Rn 
			2'b10: mem_address = 0 + 4 * cnt_ldm_regs;
		endcase
	end
	
	single_port_ram single_port_ram_1(
		.clk(clk),           
		.rst(rst),           
		.size(EX_MEM_size | STM_STATE_size),  // 資料大小選擇（00: 8-bit, 01: 16-bit, 10: 32-bit）
		.write_enable(EX_MEM_ram_wr_en | STM_STATE_ram_wr_en | LDM_STATE_size),  
		.address(mem_address),   // 記憶體位址 
		.write_data(load_cnt_stm_regs ? stm_reg_data : EX_MEM_r2_data), 
		.read_data(mem_read)      
	);
		
	/*--------------------------
	   MEM to WB Register 
	--------------------------*/
	logic load_MEM_WB_reg;
	logic [31:0] MEM_WB_mem_read;
	logic [3:0] MEM_WB_sel_write_back_data;
	// PUSH
	logic [15:0] MEM_WB_registers;
	
	always_ff @(posedge clk) begin
		if (rst) begin
			MEM_WB_regsfile_wr_en <= 0;
			MEM_WB_sp_wr_en <= 0;
		end
		else if (load_MEM_WB_reg) begin
			MEM_WB_mem_read <= mem_read;
			MEM_WB_alu_result <= EX_MEM_alu_result;
			
			// WB
			MEM_WB_sel_write_back_data <= EX_MEM_sel_write_back_data;
			MEM_WB_regsfile_wr_en <= EX_MEM_regsfile_wr_en;
			MEM_WB_w_addr <= EX_MEM_w_addr;
			MEM_WB_sp_wr_en <= EX_MEM_sp_wr_en;
			
			// PUSH 
			MEM_WB_registers <= EX_MEM_registers;
			
		end
	end
 
	/*------------------
	    Write Back
	------------------*/
	
	// Mux: 選擇寫回暫存器的資料
	logic [31:0] PC_minus_2;
	assign PC_minus_2 = ID_EX_pc + 2;
	// POP
	logic [3:0] POP_STATE_sel_write_back_data;
	// STM
	logic [3:0] STM_STATE_sel_write_back_data;
	// LDM
	logic [3:0] LDM_STATE_sel_write_back_data;
	
	
	always_comb begin
		unique case(MEM_WB_sel_write_back_data | POP_STATE_sel_write_back_data | STM_STATE_sel_write_back_data | LDM_STATE_sel_write_back_data)
			4'b0000: write_back_data = MEM_WB_alu_result; // ALU 的計算結果
			4'b0001: write_back_data = MEM_WB_mem_read;
			4'b0010: write_back_data = {{24{MEM_WB_mem_read[7]}}, MEM_WB_mem_read[7:0]}; // 8 bits sign extend
			4'b0011: write_back_data = {{16{MEM_WB_mem_read[15]}}, MEM_WB_mem_read[15:0]}; // 16 bits sign extend
			4'b0100: write_back_data = stack_out;
			4'b0101: write_back_data = {28'b0, stk_ptr};
			4'b0110: write_back_data = base_reg + 4 * cnt_stm_regs; // 需修改 
			4'b0111: write_back_data = EX_MEM_alu_result + 4 * cnt_ldm_regs; // 需修改 
			4'b1000: write_back_data = mem_read;
			4'b1001: write_back_data = PC_minus_2;
		endcase 
	end
	
	/*-------------------------------------
	    T1 ~ T5:  five stage pipeline
	    T6 ~ T10: interrupt
 	-------------------------------------*/
	typedef enum { T0, T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, 
	               PUSH_STATE, PUSH_WAIT, POP_STATE, 
	               STM_STATE, STM_WAIT, LDM_STATE, LDM_WAIT} state_t;
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
		// IF
		load_pc = 0;
		sel_pc_offset = 0;
		sel_pc = 0;
		load_IF_ID_reg = 0;
		
		// ID	
		sel_mem_0 = 0;  // 程式記憶體
		sel_mem_1 = 0;  // 程式記憶體
		sel_r0_addr = 0;
		sel_w_addr = 0;
		sel_reg_imm = 0;
		load_ID_EX_reg = 0;
		
		// EX
		opcode = 0;
		sel_pc_reg = 0;
		flush = 0;
		load_EX_MEM_reg = 0;
		
		// MEM
		ram_wr_en = 0;
		ram_rd_en = 0;
		size = 0;
		setFlags = 0;
		load_MEM_WB_reg = 0;
		
		// WB
		sel_write_back_data = 0;
		regsfile_wr_en = 0; // register file
		sp_wr_en = 0;
		
		// PUSH
		push = 0;
		rst_push_reg = 0;
		load_push_reg = 0;
		sel_push_addr = 0;
		load_registers = 0;
		
		// POP
		pop = 0;
		rst_pop_reg = 0;
		load_pop_reg = 0;
		sel_pop_addr = 0;
		sp_wr_en = 0;
		POP_STATE_regsfile_wr_en = 0;
		POP_STATE_sel_write_back_data = 0;
		
		// STM
		rst_stm_reg = 0;
		load_stm_reg = 0;
		sel_stm_addr = 0;
		load_cnt_stm_regs = 0;
		stm_wr_en = 0;
		STM_STATE_sel_write_back_data = 0;		
		STM_STATE_regsfile_wr_en = 0;
		STM_STATE_ram_wr_en = 0;
		STM_STATE_size = 0;
		
		// LDM
		rst_ldm_reg = 0;
		load_ldm_reg = 0;
		sel_ldm_addr = 0;
		load_cnt_ldm_regs = 0;		
		ldm_wr_en = 0;
		LDM_STATE_sel_write_back_data = 0;
		LDM_STATE_regsfile_wr_en = 0;
		LDM_STATE_size = 0;		
		
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
					//if (thumb1_flag) begin
						if (pc[1]) begin
							sel_mem_0 = 2;
							sel_mem_1 = 0;
						end
						else begin
							sel_mem_0 = 0;
							sel_mem_1 = 1;
						end
					//end
					load_IF_ID_reg = 1;
					ns = T2;
				end
			T2: // ID
				begin
					// IF	
					sel_pc_offset = 0;
					sel_pc = 0;
					load_pc = 1;
					//if (thumb1_flag) begin
						if (pc[1]) begin
							sel_mem_0 = 2;
							sel_mem_1 = 0;
						end
						else begin
							sel_mem_0 = 0;
							sel_mem_1 = 1;
						end
					//end
					load_IF_ID_reg = 1;		
					
					// ID
					case(cmd)
						/* BASIC_OP */
						LSL_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSL_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						LSR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;						
						end
						ASR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = ASR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						ADD_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;						
							// EX
							opcode = ADD_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;						
						end	
						SUB_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;														
							// EX
							opcode = SUB_OP2;
							setFlags = 1;		
							// WB
							regsfile_wr_en = 1;	
						end		
						ADD_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = ADD_OP;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;							
						end
						SUB_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = SUB_OP1;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;								
						end						
						MOV_imm8: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = MOV_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;
						end		
						CMP_imm8: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0; // 未使用
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP1;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0;	 // 不寫入暫存器							
						end
						ADD_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end				
						SUB_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end	
						/* DATA_PROCESSING */
						AND_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
						EOR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = XOR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    LSL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end						
					    LSR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ASR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ASR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ADC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_FLAG_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    SBC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_FLAG_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ROR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ROR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    TST_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    RSB_imm: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP2;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1; 							
						end
					    CMP_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    CMN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果							
						end
					    ORR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = OR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    MUL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = MUL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    BIC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = BIC_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end					
						MVN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;
							// EX
							opcode = BNOT_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;						
						
						end
						/* SPECIAL_DATA_INSTRUCTIONS_and_BX */
						ADD_regs: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						CMP_reg2: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0; // 未使用 
							sel_reg_imm = 0;
							// EX
							opcode = SUB_OP1;							
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 0; // 捨棄結果
						end
						MOV_reg: begin
							// ID						
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // Rd
							sel_reg_imm = 0;
							// EX
							opcode = MOV_OP;							
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						BX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // 未使用
							sel_reg_imm = 0;
							// EX
							sel_pc_offset = 2;
							sel_pc = 1;
							load_pc = 1;
							flush = 1;
						end						
						BLX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // LR 	
							// EX
							sel_pc_offset = 2;
							sel_pc = 0;
							load_pc = 1;
                            flush = 1;							
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end
						/* LDR_LITERAL */
						LDR_lit: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10;
							ram_rd_en = 1;		
							// WB
							sel_write_back_data = 0; // PC + imm8							
							regsfile_wr_en = 1;	
						end
						/* LOAD_STORE */
						STR_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits	
						end						
						STRH_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						STRB_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits						
						end
						LDRSB_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits	
							ram_rd_en = 1;		
							//WB
							sel_write_back_data = 2; // 8 bits sign extend
							regsfile_wr_en = 1;											
						end						
						LDR_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end
						LDRH_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						LDRB_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;										
						end						
						LDRSH_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						STR_imm5: begin
							// ID							
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;	
							setFlags = 0;								
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits
						end		
						LDR_imm5: begin
							// ID		
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;						
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end	
						STRB_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits	
						end
						LDRB_imm5: begin // 自動補0
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;							
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						LDRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b01; // 16 bits	
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;						
							// MEM
							size = 2'b10; // 32 bits	
						end
						LDR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							// MEM	
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;								
						end
						/* GEN_PC_ADDR */						
						ADR: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							sel_pc_reg = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						/* GEN_SP_ADDR */
						ADD_SP_imm8: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						/* MISCELLANEOUS */
						ADD_SP_imm7: begin	
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = ADD_OP;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;								
						end
						SUB_SP_imm7: begin
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = SUB_OP1;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;		
						end
						SXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end		
						SXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						PUSH_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end 									
						REV: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end			
						REV16: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV16_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						REVSH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REVSH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						POP_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end
						/* STORE_MULTIPLE_REGISTERS */
						STM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end
						/* LOAD_MULTIPLE_REGISTERS */
						LDM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end
						/* CONDITIONAL_BRANCH_and_SUPERVISOR_CALL */
						B_COND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1; // 立即數
							// EX
							if(sel_cond) begin
								sel_pc_offset = 2;
								sel_pc = 0; // branch offset
								load_pc = 1;
								flush = 1;
							end
						end
						/* UNCONDITIONAL_BRANCH */
						B_UNCOND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1;   // 立即數
							// EX						
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1; 
							flush = 1;								
							// WB							
						end
						/* BRANCH_and_MISCELLANEOUS_CONTROL*/
						/*MSR_reg: begin
						
						end*/
						/*MRS: begin
						
						end*/		
						BL: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;	 // LR 	
							sel_reg_imm = 1;   // 立即數
							// EX
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1;
							flush = 1;	
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end	
					endcase
					load_ID_EX_reg = 1;
					
					ns = T3;
				end
			T3: // EX
				begin
					// IF											
					sel_pc_offset = 0;
					sel_pc = 0;
					if (stall == 0) load_pc = 1;
											
					//if (thumb1_flag) begin
						if (pc[1]) begin
							sel_mem_0 = 2;
							sel_mem_1 = 0;
						end
						else begin
							sel_mem_0 = 0;
							sel_mem_1 = 1;
						end
					//end	
					if (stall == 1 | ID_EX_flush == 1) load_IF_ID_reg = 0;		
					else load_IF_ID_reg = 1;
					
					// ID
					case(cmd)
						/* BASIC_OP */
						LSL_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSL_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						LSR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;						
						end
						ASR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = ASR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						ADD_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;						
							// EX
							opcode = ADD_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;						
						end	
						SUB_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;														
							// EX
							opcode = SUB_OP2;
							setFlags = 1;		
							// WB
							regsfile_wr_en = 1;	
						end		
						ADD_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = ADD_OP;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;							
						end
						SUB_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = SUB_OP1;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;								
						end						
						MOV_imm8: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = MOV_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;
						end		
						CMP_imm8: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0; // 未使用
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP1;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0;	 // 不寫入暫存器							
						end
						ADD_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end				
						SUB_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end	
						/* DATA_PROCESSING */
						AND_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
						EOR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = XOR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    LSL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end						
					    LSR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ASR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ASR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ADC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_FLAG_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    SBC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_FLAG_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ROR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ROR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    TST_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    RSB_imm: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP2;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1; 							
						end
					    CMP_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    CMN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果							
						end
					    ORR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = OR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    MUL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = MUL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    BIC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = BIC_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end					
						MVN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;
							// EX
							opcode = BNOT_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;						
						
						end
						/* SPECIAL_DATA_INSTRUCTIONS_and_BX */
						ADD_regs: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						CMP_reg2: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0; // 未使用 
							sel_reg_imm = 0;
							// EX
							opcode = SUB_OP1;							
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 0; // 捨棄結果
						end
						MOV_reg: begin
							// ID						
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // Rd
							sel_reg_imm = 0;
							// EX
							opcode = MOV_OP;							
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						BX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // 未使用
							sel_reg_imm = 0;
							// EX
							sel_pc_offset = 2;
							sel_pc = 1;
							load_pc = 1;
							flush = 1;
						end						
						BLX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // LR 	
							// EX
							sel_pc_offset = 2;
							sel_pc = 0;
							load_pc = 1;
                            flush = 1;							
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end				
						/* LDR_LITERAL */
						LDR_lit: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10;
							ram_rd_en = 1;		
							// WB
							sel_write_back_data = 0; // PC + imm8							
							regsfile_wr_en = 1;	
						end
						/* LOAD_STORE */
						STR_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits	
						end						
						STRH_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						STRB_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits						
						end
						LDRSB_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits	
							ram_rd_en = 1;		
							//WB
							sel_write_back_data = 2; // 8 bits sign extend
							regsfile_wr_en = 1;											
						end						
						LDR_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end
						LDRH_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						LDRB_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;										
						end						
						LDRSH_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						STR_imm5: begin
							// ID							
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;	
							setFlags = 0;								
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits
						end		
						LDR_imm5: begin
							// ID		
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;						
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end	
						STRB_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits	
						end
						LDRB_imm5: begin // 自動補0
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;							
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						LDRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b01; // 16 bits	
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;						
							// MEM
							size = 2'b10; // 32 bits	
						end
						LDR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							// MEM	
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;								
						end
						/* GEN_PC_ADDR */						
						ADR: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							sel_pc_reg = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end										
						/* GEN_SP_ADDR */
						ADD_SP_imm8: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end					
						/* MISCELLANEOUS */
						ADD_SP_imm7: begin	
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = ADD_OP;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;								
						end
						SUB_SP_imm7: begin
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = SUB_OP1;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;		
						end
						SXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end		
						SXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						PUSH_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end 								
						REV: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end			
						REV16: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV16_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						REVSH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REVSH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						POP_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end
						/* STORE_MULTIPLE_REGISTERS */
						STM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end
						/* LOAD_MULTIPLE_REGISTERS */
						LDM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end											
						/* CONDITIONAL_BRANCH_and_SUPERVISOR_CALL */
						B_COND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1; // 立即數
							// EX
							if(sel_cond) begin
								sel_pc_offset = 2;
								sel_pc = 0; // branch offset
								load_pc = 1;
								flush = 1;
							end
						end						
						/* UNCONDITIONAL_BRANCH */
						B_UNCOND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1;   // 立即數
							// EX						
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1; 
							flush = 1;								
							// WB
							
						end						
						/* BRANCH_and_MISCELLANEOUS_CONTROL*/
						BL: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;	 // LR 	
							sel_reg_imm = 1;   // 立即數
							// EX
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1;
							flush = 1;	
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end							
					endcase
					if (ID_EX_flush == 0) load_ID_EX_reg = 1;
					
					// EX
					load_EX_MEM_reg	= 1;
					
					if(cmd == PUSH_REGS) ns = PUSH_STATE;
					else if(cmd == POP_REGS) ns = POP_STATE;
					else if(cmd == STM) ns = STM_WAIT;
					else if(cmd == LDM) ns = LDM_STATE;
					else ns = T4;
				end
			T4: // MEM
				begin
					// IF										
					sel_pc_offset = 0;
					sel_pc = 0;
					if (stall == 0) load_pc = 1;
					//if (thumb1_flag) begin
						if (pc[1]) begin
							sel_mem_0 = 2;
							sel_mem_1 = 0;
						end
						else begin
							sel_mem_0 = 0;
							sel_mem_1 = 1;
						end
					//end					
					if (stall == 1 | ID_EX_flush == 1) load_IF_ID_reg = 0;		
					else load_IF_ID_reg = 1;
					
					// ID
					case(cmd)
						/* BASIC_OP */
						LSL_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSL_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						LSR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;						
						end
						ASR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = ASR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						ADD_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;						
							// EX
							opcode = ADD_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;						
						end	
						SUB_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;														
							// EX
							opcode = SUB_OP2;
							setFlags = 1;		
							// WB
							regsfile_wr_en = 1;	
						end		
						ADD_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = ADD_OP;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;							
						end
						SUB_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = SUB_OP1;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;								
						end						
						MOV_imm8: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = MOV_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;
						end		
						CMP_imm8: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0; // 未使用
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP1;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0;	 // 不寫入暫存器							
						end
						ADD_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end				
						SUB_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end	
						/* DATA_PROCESSING */
						AND_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
						EOR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = XOR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    LSL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end						
					    LSR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ASR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ASR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ADC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_FLAG_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    SBC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_FLAG_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ROR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ROR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    TST_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    RSB_imm: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP2;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1; 							
						end
					    CMP_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    CMN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果							
						end
					    ORR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = OR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    MUL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = MUL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    BIC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = BIC_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end					
						MVN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;
							// EX
							opcode = BNOT_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;						
						
						end
						/* SPECIAL_DATA_INSTRUCTIONS_and_BX */
						ADD_regs: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						CMP_reg2: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0; // 未使用 
							sel_reg_imm = 0;
							// EX
							opcode = SUB_OP1;							
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 0; // 捨棄結果
						end
						MOV_reg: begin
							// ID						
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // Rd
							sel_reg_imm = 0;
							// EX
							opcode = MOV_OP;							
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						BX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // 未使用
							sel_reg_imm = 0;
							// EX
							sel_pc_offset = 2;
							sel_pc = 1;
							load_pc = 1;
							flush = 1;
						end						
						BLX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // LR 	
							// EX
							sel_pc_offset = 2;
							sel_pc = 0;
							load_pc = 1;
                            flush = 1;							
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end			
						/* LDR_LITERAL */
						LDR_lit: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10;
							ram_rd_en = 1;		
							// WB
							sel_write_back_data = 0; // PC + imm8							
							regsfile_wr_en = 1;	
						end
						/* LOAD_STORE */
						STR_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits	
						end						
						STRH_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						STRB_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits						
						end
						LDRSB_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits	
							ram_rd_en = 1;		
							//WB
							sel_write_back_data = 2; // 8 bits sign extend
							regsfile_wr_en = 1;											
						end						
						LDR_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end
						LDRH_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						LDRB_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;										
						end						
						LDRSH_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						STR_imm5: begin
							// ID							
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;	
							setFlags = 0;								
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits
						end		
						LDR_imm5: begin
							// ID		
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;						
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end	
						STRB_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits	
						end
						LDRB_imm5: begin // 自動補0
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;							
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						LDRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b01; // 16 bits	
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;						
							// MEM
							size = 2'b10; // 32 bits	
						end
						LDR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							// MEM	
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;								
						end
						/* GEN_PC_ADDR */						
						ADR: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							sel_pc_reg = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end						
						/* GEN_SP_ADDR */
						ADD_SP_imm8: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end						
						/* MISCELLANEOUS */
						ADD_SP_imm7: begin	
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = ADD_OP;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;								
						end
						SUB_SP_imm7: begin
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = SUB_OP1;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;		
						end
						SXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end		
						SXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						PUSH_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end 									
						REV: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end			
						REV16: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV16_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						REVSH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REVSH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						POP_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end
						/* STORE_MULTIPLE_REGISTERS */
						STM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end
						/* LOAD_MULTIPLE_REGISTERS */
						LDM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end						
						/* CONDITIONAL_BRANCH_and_SUPERVISOR_CALL */
						B_COND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1; // 立即數
							// EX
							if(sel_cond) begin
								sel_pc_offset = 2;
								sel_pc = 0; // branch offset
								load_pc = 1;
								flush = 1;
							end
						end						
						/* UNCONDITIONAL_BRANCH */
						B_UNCOND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1;   // 立即數
							// EX						
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1; 
							flush = 1;								
							// WB
							
						end						
						/* BRANCH_and_MISCELLANEOUS_CONTROL*/
						BL: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;	 // LR 	
							sel_reg_imm = 1;   // 立即數
							// EX
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1;
							flush = 1;	
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end							
					endcase
					if (ID_EX_flush == 0) load_ID_EX_reg = 1;
					
					// EX
					load_EX_MEM_reg	= 1;

					// MEM
					load_MEM_WB_reg = 1;
					
					if(cmd == PUSH_REGS) ns = PUSH_STATE;
					else if(cmd == POP_REGS) ns = POP_STATE;
					else if(cmd == STM) ns = STM_WAIT;
					else if(cmd == LDM) ns = LDM_STATE;
					else ns = T5;
				end
			T5: // WB
				begin
					// IF										
					sel_pc_offset = 0;
					sel_pc = 0;
					if (stall == 0) load_pc = 1;							
					//if (thumb1_flag) begin
						if (pc[1]) begin
							sel_mem_0 = 2;
							sel_mem_1 = 0;
						end
						else begin
							sel_mem_0 = 0;
							sel_mem_1 = 1;
						end
					//end			
					if (stall == 1 | ID_EX_flush == 1) load_IF_ID_reg = 0;		
					else load_IF_ID_reg = 1;
					// ID
					case(cmd)
						/* BASIC_OP */
						LSL_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSL_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end
						LSR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = LSR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;						
						end
						ASR_imm5: begin
							// ID
							sel_r0_addr = 0;							
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = ASR_OP;
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end
						ADD_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;						
							// EX
							opcode = ADD_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;						
						end	
						SUB_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;														
							// EX
							opcode = SUB_OP2;
							setFlags = 1;		
							// WB
							regsfile_wr_en = 1;	
						end		
						ADD_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = ADD_OP;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;							
						end
						SUB_imm3: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;														
							// EX
							opcode = SUB_OP1;
							setFlags = 1;		
							// WB
							sel_write_back_data = 0;							
							regsfile_wr_en = 1;								
						end						
						MOV_imm8: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 1;
							// EX
							opcode = MOV_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;
						end		
						CMP_imm8: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0; // 未使用
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP1;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0;	 // 不寫入暫存器							
						end
						ADD_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end				
						SUB_imm8: begin
							// ID
							sel_r0_addr = 1; // Rdn
							sel_w_addr = 0;
							sel_reg_imm = 1;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end	
						/* DATA_PROCESSING */
						AND_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
						EOR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = XOR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    LSL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end						
					    LSR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = LSR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ASR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ASR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ADC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_FLAG_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    SBC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_FLAG_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    ROR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ROR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    TST_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = AND_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    RSB_imm: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 0;	
							sel_reg_imm = 1;
							// EX
							opcode = SUB_OP2;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1; 							
						end
					    CMP_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = SUB_OP1;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果						
						end
					    CMN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = ADD_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 0; // 捨棄結果							
						end
					    ORR_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = OR_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    MUL_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = MUL_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end
					    BIC_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;							
							// EX
							opcode = BIC_OP;
							setFlags = 1;	
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;							
						end					
						MVN_reg: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;
							sel_reg_imm = 0;
							// EX
							opcode = BNOT_OP;
							setFlags = 1;							
							// WB
							sel_write_back_data = 0;								
							regsfile_wr_en = 1;						
						
						end
						/* SPECIAL_DATA_INSTRUCTIONS_and_BX */
						ADD_regs: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0;	
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						CMP_reg2: begin
							// ID
							sel_r0_addr = 0;
							sel_w_addr = 0; // 未使用 
							sel_reg_imm = 0;
							// EX
							opcode = SUB_OP1;							
							setFlags = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 0; // 捨棄結果
						end
						MOV_reg: begin
							// ID						
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // Rd
							sel_reg_imm = 0;
							// EX
							opcode = MOV_OP;							
							setFlags = 0;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end
						BX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // 未使用
							sel_reg_imm = 0;
							// EX
							sel_pc_offset = 2;
							sel_pc = 1;
							load_pc = 1;
							flush = 1;
						end						
						BLX: begin
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;	 // LR 	
							// EX
							sel_pc_offset = 2;
							sel_pc = 0;
							load_pc = 1;
                            flush = 1;							
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end						
						/* LDR_LITERAL */
						LDR_lit: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10;
							ram_rd_en = 1;		
							// WB
							sel_write_back_data = 0; // PC + imm8							
							regsfile_wr_en = 1;	
						end
						/* LOAD_STORE */
						STR_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits	
						end						
						STRH_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						STRB_reg: begin
							// ID
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;		
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits						
						end
						LDRSB_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits	
							ram_rd_en = 1;		
							//WB
							sel_write_back_data = 2; // 8 bits sign extend
							regsfile_wr_en = 1;											
						end						
						LDR_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end
						LDRH_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						LDRB_reg: begin // 自動補0
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;										
						end						
						LDRSH_reg: begin
							// ID						
							sel_r0_addr = 0; 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 0;
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b01; // 16 bits
							ram_rd_en = 1;
							//WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;									
						end						
						STR_imm5: begin
							// ID							
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;	
							setFlags = 0;								
							// MEM
							ram_wr_en = 1;
							size = 2'b10; // 32 bits
						end		
						LDR_imm5: begin
							// ID		
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;						
							// EX
							opcode = ADD_OP;
							setFlags = 0;							
							// MEM
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;
							
						end	
						STRB_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b00; // 8 bits	
						end
						LDRB_imm5: begin // 自動補0
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b00; // 8 bits
							ram_rd_en = 1;							
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							setFlags = 0;	
							// MEM
							ram_wr_en = 1;
							size = 2'b01; // 16 bits					
						end						
						LDRH_imm5: begin
							// ID
							sel_r0_addr = 1; // Rn
							sel_w_addr = 1; // Rt	
                            sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							// MEM
							size = 2'b01; // 16 bits	
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;							
						end
						STR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;						
							// MEM
							size = 2'b10; // 32 bits	
						end
						LDR_imm8_SP: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 1; // Rt
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;
							// MEM	
							size = 2'b10; // 32 bits
							ram_rd_en = 1;
							// WB
							sel_write_back_data = 1;
							regsfile_wr_en = 1;								
						end
						/* GEN_PC_ADDR */						
						ADR: begin
							// ID
							sel_r0_addr = 3; // PC
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;
							// EX
							opcode = ADD_OP;							
							sel_pc_reg = 1;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end						
						/* GEN_SP_ADDR */
						ADD_SP_imm8: begin
							// ID
							sel_r0_addr = 2; // SP 
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;							
							// EX
							opcode = ADD_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;							
						end					
						/* MISCELLANEOUS */
						ADD_SP_imm7: begin	
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = ADD_OP;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;								
						end
						SUB_SP_imm7: begin
							// ID
							sel_r0_addr = 2; // SP
							sel_w_addr = 0; // Rd
							sel_reg_imm = 1;	
							// EX
							opcode = SUB_OP1;							
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;		
						end
						SXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;	
						end		
						SXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = SEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						UXTB: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = UEB_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						PUSH_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end 										
						REV: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end			
						REV16: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REV16_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						REVSH: begin	
							// ID
							sel_r0_addr = 0; // Rm
							sel_w_addr = 0;  // Rd	
							sel_reg_imm = 0; // 未使用
							// EX
							opcode = REVSH_OP;
							// WB
							sel_write_back_data = 0;
							regsfile_wr_en = 1;
						end		
						POP_REGS: begin	
							// ID
							load_registers = 1;
							// WB
							sel_write_back_data = 5; // 堆疊指標位址
							regsfile_wr_en = 1; 
							sp_wr_en = 1;
						end
						/* STORE_MULTIPLE_REGISTERS */
						STM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end
						/* LOAD_MULTIPLE_REGISTERS */
						LDM: begin
							// ID
							load_registers = 1;	
							sel_r0_addr = 1; // Rn							
						end	
						/* CONDITIONAL_BRANCH_and_SUPERVISOR_CALL */
						B_COND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1; // 立即數
							// EX
							if(sel_cond) begin
								sel_pc_offset = 2;
								sel_pc = 0; // branch offset
								load_pc = 1;
								flush = 1;
							end
						end						
						/* UNCONDITIONAL_BRANCH */
						B_UNCOND: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;  // 未使用
							sel_reg_imm = 1;   // 立即數
							// EX						
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1; 
							flush = 1;								
							// WB
							
						end						
						/* BRANCH_and_MISCELLANEOUS_CONTROL*/
						BL: begin
							// ID
							sel_r0_addr = 0; // 未使用 
							sel_w_addr = 0;	 // LR 	
							sel_reg_imm = 1;   // 立即數
							// EX
							sel_pc_offset = 2;
							sel_pc = 0; // branch offset
							load_pc = 1;
							flush = 1;	
							// WB
							sel_write_back_data = 9;
							regsfile_wr_en = 1;							
						end							
					endcase
					if (ID_EX_flush == 0) load_ID_EX_reg = 1;
					
					// EX
					load_EX_MEM_reg	= 1;					

					// MEM
					load_MEM_WB_reg = 1;

					// WB								
						
					if(cmd == PUSH_REGS) ns = PUSH_WAIT;
					else if(cmd == POP_REGS) ns = POP_STATE;
					else if(cmd == STM) ns = STM_WAIT;
					else if(cmd == LDM) ns = LDM_WAIT;
					else ns = T5;
				end
				
			PUSH_WAIT:
				begin // 等待前一個指令寫入
					load_MEM_WB_reg = 1;										
					ns = PUSH_STATE;
				end
				
			PUSH_STATE:	
				begin // 規格: 暫存器由大到小 PUSH
					if(cnt_push_reg <= 0) begin
						if (multi_registers[cnt_push_reg]) begin
							sel_push_addr = cnt_push_reg;
							push = 1;
						end	
						else begin
							rst_push_reg = 1;
						end
						ns = T5;
					end
					else begin
						load_push_reg = 1;
						if (multi_registers[cnt_push_reg]) begin
							sel_push_addr = cnt_push_reg;
							push = 1;
						end	
					end
				end
				
			POP_STATE: 
				begin  // 規格: 暫存器由小到大 POP
					if(cnt_pop_reg >= 15) begin
						rst_pop_reg = 1;
						ns = T5;
					end
					else begin
						load_pop_reg = 1;
						if (multi_registers[cnt_pop_reg]) begin
							sel_pop_addr = cnt_pop_reg;
							POP_STATE_regsfile_wr_en = 1;
							POP_STATE_sel_write_back_data = 4;
							pop = 1;
						end	
					end
				end
				
			STM_WAIT: begin
					load_MEM_WB_reg = 1;										
					ns = STM_STATE;			
			end
			
			STM_STATE: begin
				if(cnt_stm_reg >= 8) begin
				
					stm_wr_en = 1;
					STM_STATE_sel_write_back_data = 6;
					STM_STATE_regsfile_wr_en = 1;  
					rst_stm_reg = 1;	
					
					ns = T5;
				end
				else begin // R0 ~ R7
					load_stm_reg = 1;
					if (multi_registers[cnt_stm_reg]) begin
						load_cnt_stm_regs = 1;
						sel_stm_addr = cnt_stm_reg;
						
						STM_STATE_size = 2'b10; // 32 bits
						STM_STATE_ram_wr_en = 1;
						
					end	
				end				
			end
			
			LDM_WAIT: begin
					load_MEM_WB_reg = 1;										
					ns = LDM_STATE;					
			end
			
			LDM_STATE: begin
				if(cnt_ldm_reg >= 8) begin
				
					stm_wr_en = 1;
					LDM_STATE_sel_write_back_data = 7;
					LDM_STATE_regsfile_wr_en = 1; 
					rst_ldm_reg = 1;
							
					ns = T5;
				end
				else begin // R0 ~ R7
					load_ldm_reg = 1;
					if (multi_registers[cnt_ldm_reg]) begin
						load_cnt_ldm_regs = 1;
						ldm_wr_en = 1;
						LDM_STATE_regsfile_wr_en = 1;
						LDM_STATE_sel_write_back_data = 8;
						LDM_STATE_size = 2'b10; // 32 bits
						sel_ldm_addr = cnt_ldm_reg;
					end	
				end				
			end
			
		endcase
	end

endmodule