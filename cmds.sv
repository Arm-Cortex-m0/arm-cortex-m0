// 規格書 p.85
parameter 	LSL_imm5 = 7'd0,
			LSR_imm5 = 7'd1,
			ASR_imm5 = 7'd2,
			ADD_reg  = 7'd3,
			SUB_reg  = 7'd4,
			ADD_imm3 = 7'd5,
			SUB_imm3 = 7'd6,
			MOV_imm8 = 7'd7,
			CMP_imm8 = 7'd8,
			ADD_imm8 = 7'd9,
			SUB_imm8 = 7'd10;
		    
// 規格書 p.86
parameter 	AND_reg = 7'd11,
			EOR_reg = 7'd12,
			LSL_reg = 7'd13,
			LSR_reg = 7'd14,
			ASR_reg = 7'd15,
			ADC_reg = 7'd16,
			SBC_reg = 7'd17,
			ROR_reg = 7'd18,
			TST_reg = 7'd19,
			RSB_imm = 7'd20,
			CMP_reg = 7'd21,
			CMN_reg = 7'd22,
			ORR_reg = 7'd23,
			MUL_reg = 7'd24, //???
			BIC_reg = 7'd25,
			MVN_reg = 7'd26;

// 規格書 p.87
parameter 	ADD_regs = 7'd27, // 這是另外一條 ADD_reg
			UPREDCT  = 7'd28, // ???
			CMP_reg2 = 7'd29, // 這是另外一條 CMP_reg
			MOV_reg  = 7'd30,
			BX       = 7'd31,
			BLX      = 7'd32;
			
// 規格書 p.88
parameter 	STR_reg     = 7'd33,
			STRH_reg    = 7'd34,
			STRB_reg    = 7'd35,
			LDRSB_reg   = 7'd36,
			LDR_reg     = 7'd37,
			LDRH_reg    = 7'd38,
			LDRB_reg    = 7'd39,
			LDRSH_reg   = 7'd40,
			STR_imm5    = 7'd41,
			LDR_imm5    = 7'd42,
			STRB_imm5   = 7'd43,
			LDRB_imm5   = 7'd44,
			STRH_imm5   = 7'd45,
			LDRH_imm5   = 7'd46,
			STR_imm8_SP = 7'd47,
			LDR_imm8_SP = 7'd48;
			
parameter   DATA_N = 32,
	        SIZE   = 16;