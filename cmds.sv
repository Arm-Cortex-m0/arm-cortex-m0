// 規格書 p.85
parameter 	LSL_imm5 		= 7'd0,
			LSR_imm5 		= 7'd1,
			ASR_imm5 		= 7'd2,
			ADD_reg  		= 7'd3,
			SUB_reg  		= 7'd4,
			ADD_imm3 		= 7'd5,
			SUB_imm3 		= 7'd6,
			MOV_imm8 		= 7'd7,
			CMP_imm8 		= 7'd8,
			ADD_imm8 		= 7'd9,
			SUB_imm8 		= 7'd10;
		    
// 規格書 p.86
parameter 	AND_reg 		= 7'd11,
			EOR_reg 		= 7'd12,
			LSL_reg 		= 7'd13,
			LSR_reg 		= 7'd14,
			ASR_reg 		= 7'd15,
			ADC_reg 		= 7'd16,
			SBC_reg 		= 7'd17,
			ROR_reg 		= 7'd18,
			TST_reg 		= 7'd19,
			RSB_imm 		= 7'd20,
			CMP_reg 		= 7'd21,
			CMN_reg 		= 7'd22,
			ORR_reg 		= 7'd23,
			MUL_reg 		= 7'd24, //???
			BIC_reg 		= 7'd25,
			MVN_reg 		= 7'd26;

// 規格書 p.87
parameter 	ADD_regs 		= 7'd27, // 這是另外一條 ADD_reg
			UPREDCT  		= 7'd28, // ???
			CMP_reg2 		= 7'd29, // 這是另外一條 CMP_reg
			MOV_reg  		= 7'd30,
			BX       		= 7'd31,
			BLX      		= 7'd32;
			
// 規格書 p.141
parameter 	LDR_lit  		= 7'd33;
			
// 規格書 p.88
parameter 	STR_reg     	= 7'd34,
			STRH_reg    	= 7'd35,
			STRB_reg    	= 7'd36,
			LDRSB_reg   	= 7'd37,
			LDR_reg     	= 7'd38,
			LDRH_reg    	= 7'd39,
			LDRB_reg    	= 7'd40,
			LDRSH_reg   	= 7'd41,
			STR_imm5    	= 7'd42,
			LDR_imm5    	= 7'd43,
			STRB_imm5   	= 7'd44,
			LDRB_imm5   	= 7'd45,
			STRH_imm5   	= 7'd46,
			LDRH_imm5   	= 7'd47,
			STR_imm8_SP 	= 7'd48,
			LDR_imm8_SP 	= 7'd49;
			
// 規格書 p.115
parameter 	ADR 			= 7'd50;

// 規格書 p.111
parameter	ADD_SP_imm8 	= 7'd51;

// 規格書 p.89
parameter	ADD_SP_imm7 	= 7'd52,
			SUB_SP_imm7  	= 7'd53,
			SXTH		 	= 7'd54,
			SXTB		 	= 7'd55,
			UXTH		 	= 7'd56,
			UXTB		 	= 7'd57,
			PUSH_REGS		= 7'd58,
			CPS			 	= 7'd59,
			REV			 	= 7'd60,
			REV16		 	= 7'd61,
			REVSH		 	= 7'd62,
			POP_REGS		= 7'd63,
			BKPT		 	= 7'd63,
			HINT		 	= 7'd64;
			
// 規格書 p.90
parameter 	UNDEFINED		= 7'd65,
			NOP				= 7'd66,
			YIELD			= 7'd67,
			WFE				= 7'd68,
			WFI				= 7'd69,
			SEV				= 7'd70;
			
// 規格書 p.175
parameter 	STM				= 7'd71;

// 規格書 p.137
parameter	LDM				= 7'd72;

// 規格書 p.119
parameter	B_COND			= 7'd73;

// 規格書 p.119
parameter	B_UNCOND		= 7'd74;
		
// 規格書 p.91
parameter   MSR_reg			= 7'd75,
			MRS				= 7'd76,		
			BL 				= 7'd77;
		
		
parameter   DATA_N = 32,
	        SIZE   = 16;
			
parameter   ADD_OP       = 5'b00000,
            ADD_FLAG_OP  = 5'b00001,
			SUB_OP1      = 5'b00010,
			SUB_OP2      = 5'b00011,
			SUB_FLAG_OP1 = 5'b00100,
			AND_OP       = 5'b00101,
			OR_OP        = 5'b00110,
			XOR_OP       = 5'b00111,
			MOV_OP       = 5'b01000,
			INC_OP       = 5'b01001,
			DEC_OP       = 5'b01010,
			ZERO_OP      = 5'b01011,
			BNOT_OP      = 5'b01100, // bitwise NOT
			LSL_OP       = 5'b01101, // Logical Shift Left
			LSR_OP       = 5'b01110, // Logical Shift Right
			ASR_OP       = 5'b01111, // Arithmetic Shift Right 
			ROR_OP       = 5'b10000, // Rotate Right 
			MUL_OP       = 5'b10001, // Multiply
			BIC_OP       = 5'b10010, // Bit Clear
			SEH_OP       = 5'b10011, // Signed Extend Halfword
			SEB_OP       = 5'b10100, // Signed Extend Byte
			UEH_OP       = 5'b10101, // UnSigned Extend Halfword
			UEB_OP       = 5'b10110, // UnSigned Extend Byte
			REV_OP       = 5'b10111, // Byte-Reverse Word
			REV16_OP     = 5'b11000, // Byte-Reverse Packed Halfword
			REVSH_OP     = 5'b11001; // Byte-Reverse Signed Halfword
        