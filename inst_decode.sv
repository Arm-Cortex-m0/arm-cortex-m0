module inst_decode
    (
		input  logic 			isThumb,
		input  logic [15:0] 	ir_q0,
		input  logic [15:0] 	ir_q1,
		output logic [6:0] 		cmd,
		// 低權限指令只能存取低位暫存器 0~7 (3bit)
		// 高權限指令才可存取高位暫存器 >8  (4bit)
		output logic [3:0] 		Rm,
		output logic [3:0] 		Rn,
		output logic [3:0] 		Rd,
		output logic [3:0] 		Rt,
		output logic [31:0] 	imm 
		// 雖然論文的 16bit 指令中，最大的立即數為 11 個 bit ( imm11 ) ( 這種寫法以後 signed bit 有問題再改 )
	);
	`include "cmds.sv"
	
	// 規格書 p.84
	typedef enum 
	  {
		BASIC_OP,        
		DATA_PROCESSING,
		SPECIAL_DATA_INSTRUCTIONS_and_BX,
		LDR,
		LOAD_STORE
	  } inst_types;

	inst_types inst_type;
	
	
	always_comb begin
		if(isThumb) begin// 規格書 p.84
			unique casez(ir_q0[15:10])
				6'b00????: inst_type = BASIC_OP;
				6'b010000: inst_type = DATA_PROCESSING;
				6'b010001: inst_type = SPECIAL_DATA_INSTRUCTIONS_and_BX;
				6'b01001?: inst_type = LDR;
				6'b0101??: inst_type = LOAD_STORE; // 這三個都是 Load/Store
				6'b011???: inst_type = LOAD_STORE; // 這三個都是 Load/Store
				6'b100???: inst_type = LOAD_STORE; // 這三個都是 Load/Store
			endcase
		end
		else begin
			inst_type = BASIC_OP;
		end
	end
	
	always_comb begin
		cmd = 0;
		casez(inst_type)
			BASIC_OP: //規格書 p.85
				casez(ir_q0[13:9])
					5'b000??: cmd = LSL_imm5; //0
					5'b001??: cmd = LSR_imm5; //1
					5'b010??: cmd = ASR_imm5; //2
					5'b01100: cmd = ADD_reg ; //3
					5'b01101: cmd = SUB_reg ; //4
					5'b01110: cmd = ADD_imm3; //5
					5'b01111: cmd = SUB_imm3; //6
					5'b100??: cmd = MOV_imm8; //7
					5'b101??: cmd = CMP_imm8; //8
					5'b110??: cmd = ADD_imm8; //9
					5'b111??: cmd = SUB_imm8; //10
				endcase
			DATA_PROCESSING: //規格書 p.86
				casez(ir_q0[9:6])
					4'b0000: cmd = AND_reg;
					4'b0001: cmd = EOR_reg;
					4'b0010: cmd = LSL_reg;
					4'b0011: cmd = LSR_reg;
					4'b0100: cmd = ASR_reg;
					4'b0101: cmd = ADC_reg;
					4'b0110: cmd = SBC_reg;
					4'b0111: cmd = ROR_reg;
					4'b1000: cmd = TST_reg;
					4'b1001: cmd = RSB_imm;
					4'b1010: cmd = CMP_reg;
					4'b1011: cmd = CMN_reg;
					4'b1100: cmd = ORR_reg;
					4'b1101: cmd = MUL_reg;
					4'b1110: cmd = BIC_reg;
					4'b1111: cmd = MVN_reg;
				endcase
			SPECIAL_DATA_INSTRUCTIONS_and_BX: //規格書 p.87
				casez(ir_q0[9:6])
					4'b00??: cmd = ADD_regs;
					4'b0100: cmd = UPREDCT ;
					4'b011?: cmd = CMP_reg2;
					4'b10??: cmd = MOV_reg ;
					4'b110?: cmd = BX      ;
					4'b111?: cmd = BLX     ;			
				endcase
			LOAD_STORE: // 規格書 p.88
				casez({ir_q0[15:12],ir_q0[11:9]}) // {opA,opB}
					{4'b0101, 3'b000}: cmd = STR_reg    ;
					{4'b0101, 3'b001}: cmd = STRH_reg   ;
					{4'b0101, 3'b010}: cmd = STRB_reg   ;
					{4'b0101, 3'b011}: cmd = LDRSB_reg  ;
					{4'b0101, 3'b100}: cmd = LDR_reg    ;
					{4'b0101, 3'b101}: cmd = LDRH_reg   ;
					{4'b0101, 3'b110}: cmd = LDRB_reg   ;
					{4'b0101, 3'b111}: cmd = LDRSH_reg  ;
					{4'b0110, 3'b0??}: cmd = STR_imm5   ;
					{4'b0110, 3'b1??}: cmd = LDR_imm5   ;
					{4'b0111, 3'b0??}: cmd = STRB_imm5  ;
					{4'b0111, 3'b1??}: cmd = LDRB_imm5  ;
					{4'b1000, 3'b0??}: cmd = STRH_imm5  ;
					{4'b1000, 3'b1??}: cmd = LDRH_imm5  ;
					{4'b1001, 3'b0??}: cmd = STR_imm8_SP;
					{4'b1001, 3'b1??}: cmd = LDR_imm8_SP;
				endcase
		endcase
	end
	
	logic [2:0] imm3;
	logic [4:0] imm5;
	logic [7:0] imm8;
	logic [10:0] imm10;
	always_comb begin
		// 預設值防止產生latch
		imm3 = 0;
		imm5 = 0;
		imm8 = 0;
		imm10 = 0;
		Rm = 0;
		Rn = 0;
		Rd = 0;
		Rt = 0;
		case(cmd)
			// BASIC_OP 規格書 p.85
			LSL_imm5: begin imm5 = ir_q0[10:6]; Rm = ir_q0[5:3]; Rd = ir_q0[2:0];  end // 規格書 p.150  LSLS{<q>} <Rd>, <Rm>, #<imm5>
			LSR_imm5: begin imm5 = ir_q0[10:6]; Rm = ir_q0[5:3]; Rd = ir_q0[2:0];  end // 規格書 p.152  LSRS{<q>} <Rd>, <Rm>, #<imm5>
			ASR_imm5: begin imm5 = ir_q0[10:6]; Rm = ir_q0[5:3]; Rd = ir_q0[2:0];  end // 規格書 p.117  ASRS{<q>} <Rd>, <Rm>, #<imm5>
			ADD_reg : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rd = ir_q0[2:0];     end // 規格書 p.109  ADDS <Rd>,<Rn>,<Rm>            見註解[1] 
			SUB_reg : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rd = ir_q0[2:0];     end // 規格書 p.187  SUBS <Rd>,<Rn>,<Rm>
			ADD_imm3: begin imm3 = ir_q0[8:6]; Rn = ir_q0[5:3]; Rd = ir_q0[2:0];   end // 規格書 p.107  ADDS <Rd>,<Rn>,#<imm3>
			SUB_imm3: begin imm3 = ir_q0[8:6]; Rn = ir_q0[5:3]; Rd = ir_q0[2:0];   end // 規格書 p.185  SUBS <Rd>,<Rn>,#<imm3>
			MOV_imm8: begin Rd = ir_q0[10:8]; imm8 = ir_q0[7:0];                   end // 規格書 p.154  MOVS{<q>} <Rd>,#<const>
			CMP_imm8: begin Rn = ir_q0[10:8]; imm8 = ir_q0[7:0];                   end // 規格書 p.127  CMP <Rn>,#<imm8>
			ADD_imm8: begin Rd = ir_q0[10:8]; Rn = ir_q0[10:8]; imm8 = ir_q0[7:0]; end // 規格書 p.107  ADDS <Rdn>,#<imm8>
			SUB_imm8: begin Rd = ir_q0[10:8]; Rn = ir_q0[10:8]; imm8 = ir_q0[7:0]; end // 規格書 p.185  SUBS <Rdn>,#<imm8>
			// DATA_PROCESSING 規格書 p.86
			AND_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.116  ANDS{<q>} {<Rd>,} <Rn>, <Rm>
			EOR_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.135  EORS{<q>} {<Rd>,} <Rn>, <Rm>
			LSL_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.151  LSLS{<q>} <Rd>, <Rn>, <Rm>
			LSR_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.153  LSRS{<q>} <Rd>, <Rn>, <Rm>
			ASR_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.118  ASRS{<q>} <Rd>, <Rn>, <Rm>
			ADC_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.106  ADCS{<q>} {<Rd>,} <Rn>, <Rm>
			SBC_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.173  SBCS{<q>} {<Rd>,} <Rn>, <Rm>
			ROR_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.171  RORS{<q>} <Rd>, <Rn>, <Rm>
			TST_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0];                  end // 規格書 p.192  TST{<q>} <Rn>, <Rm>
			RSB_imm: begin Rm = ir_q0[5:3]; Rd = ir_q0[2:0];                  end // 規格書 p.172  RSBS{<q>} {<Rd>,} <Rn>, #<const>
			CMP_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0];                  end // 規格書 p.129  CMP{<q>} <Rn>, <Rm>
			CMN_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0];                  end // 規格書 p.126  CMN{<q>} <Rn>, <Rm>
			ORR_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.164  ORRS{<q>} {<Rd>,} <Rn>, <Rm>
			MUL_reg: begin Rn = ir_q0[5:3]; Rm = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.159  MULS{<q>} {<Rd>,} <Rn>, <Rm>
			BIC_reg: begin Rm = ir_q0[5:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.121  BICS{<q>} {<Rd>,} <Rn>, <Rm>
			MVN_reg: begin Rm = ir_q0[5:3]; Rd = ir_q0[2:0];                  end // 規格書 p.161  MVNS{<q>} <Rd>, <Rm>
			// SPECIAL_DATA_INSTRUCTIONS_and_BX 規格書 p.87
			ADD_regs: begin Rm = ir_q0[6:3]; Rn = ir_q0[2:0]; Rd = ir_q0[2:0]; end // 規格書 p.109  ADD{S}{<q>} {<Rd>,} <Rn>, <Rm>   DN ?
			UPREDCT:  begin end // 規格書 p.(沒東西)
			CMP_reg2: begin Rm = ir_q0[6:3]; Rn = ir_q0[2:0]; end // 規格書 p.129  CMP{<q>} <Rn>, <Rm>     N ?
			MOV_reg:  begin Rm = ir_q0[6:3]; Rd = ir_q0[2:0]; end // 規格書 p.155  MOV{S}{<q>} <Rd>, <Rm>  D ?
			BX:       begin Rm = ir_q0[6:3];                  end // 規格書 p.125  BX{<q>} <Rm>            [2:0] (0)(0)(0) ?
			BLX:      begin Rm = ir_q0[6:3];                  end // 規格書 p.124  BLX{<q>} <Rm>
			// LOAD_STORE 規格書 p.88
			STR_reg    : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.179  STR{<q>} <Rt>, [<Rn>, <Rm>]
			STRH_reg   : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.183  STRH{<q>} <Rt>, [<Rn>, <Rm>]
			STRB_reg   : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.181  STRB{<q>} <Rt>, [<Rn>, <Rm> {, LSL #<shift>}]
			LDRSB_reg  : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.148  LDRSB{<q>} <Rt>, [<Rn>, <Rm>]
			LDR_reg    : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.143  LDR{<q>} <Rt>, [<Rn>, <Rm>]
			LDRH_reg   : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.147  LDRH{<q>} <Rt>, [<Rn>, <Rm>]
			LDRB_reg   : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.145  LDRB{<q>} <Rt>, [<Rn>, <Rm>]
			LDRSH_reg  : begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0];    end // 規格書 p.149  LDRSH{<q>} <Rt>, [<Rn>, <Rm>]
			STR_imm5   : begin imm5 = ir_q0[10:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0]; end // 規格書 p.177  STR{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
			LDR_imm5   : begin imm5 = ir_q0[10:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0]; end // 規格書 p.139  LDR{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
			STRB_imm5  : begin imm5 = ir_q0[10:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0]; end // 規格書 p.180  STRB{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
			LDRB_imm5  : begin imm5 = ir_q0[10:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0]; end // 規格書 p.144  LDRB{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
			STRH_imm5  : begin imm5 = ir_q0[10:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0]; end // 規格書 p.182  STRH{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
			LDRH_imm5  : begin imm5 = ir_q0[10:6]; Rn = ir_q0[5:3]; Rt = ir_q0[2:0]; end // 規格書 p.146  LDRH{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
			STR_imm8_SP: begin Rt = ir_q0[10:8]; imm8 = ir_q0[7:0];                  end // 規格書 p.177  STR{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
			LDR_imm8_SP: begin Rt = ir_q0[10:8]; imm8 = ir_q0[7:0];                  end // 規格書 p.139  LDR{<q>} <Rt>, [<Rn> {, #+/-<imm>}]
		endcase
	end

	// sign extend
	always_comb begin
		imm = 0;
		case(cmd)
		ADD_imm3,
		SUB_imm3: imm = {{29{imm3[2]}}, imm3};
		LSL_imm5,
		LSR_imm5,
		ASR_imm5,
		STR_imm5,
		LDR_imm5,
		STRB_imm5,
		LDRB_imm5,
		STRH_imm5,
		LDRH_imm5: imm = {{27{imm5[4]}}, imm5};
		MOV_imm8,
		CMP_imm8,
		ADD_imm8,
		SUB_imm8,
		STR_imm8_SP,
		LDR_imm8_SP: imm = {{26{imm8[7]}}, imm8};
		endcase	
	end
	
endmodule
/*
註：
[1]: p.109 的 T2 是 p.87 的另外一組 parameter，放牛班說 T1, T2 只是第一種第二種不是 Thumb1 Thumb2
[2]: Rdn 是指 Rd 和 Rn 相同
[3]: casez 如果某些位子的值為高阻 z，那麼對這些位子的比較就會忽略，我們的 ir_q 都是精確值。
[4]: 因為是 Thumb-1 和 Thumb-2 混合的 decoder 所以有可能解出 16bit 指令或是 32bit 指令，所以 input 是 32bit
*/