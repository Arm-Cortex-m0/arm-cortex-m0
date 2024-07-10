module ALU
(	
	input logic  [4:0]opcode,
	input logic  [31:0]op1, op2,
	input logic  [3:0]flag,
	output logic [3:0]flag_q,
	output logic [31:0]result
);
	`include "cmds.sv"
	
	logic [32:0] res_temp;
		
	always_comb
	begin 
		unique case(opcode)
			ADD_OP       : res_temp = op1 + op2;
			ADD_FLAG_OP  : res_temp = op1 + op2 + flag[1];
			SUB_OP1      : res_temp = op1 - op2;
			SUB_OP2      : res_temp = op2 - op1;
			SUB_FLAG_OP1 : res_temp = op1 - op2 - flag[1];
			AND_OP       : res_temp = op1 & op2;
			OR_OP        : res_temp = op1 | op2;
			XOR_OP       : res_temp = op1 ^ op2;
			MOV_OP       : res_temp = op2;
			INC_OP       : res_temp = op1 + 1;
			DEC_OP       : res_temp = op1 - 1;
			ZERO_OP      : res_temp = 0;
			BNOT_OP      : res_temp = ~op1;
			LSL_OP       : res_temp = op1 << op2;
			LSR_OP       : res_temp = op1 >> op2;
			ASR_OP       : res_temp = $signed(op1) >>> op2;
			ROR_OP       : res_temp = (op1 >> op2) | (op1 << (32 - op2));
			MUL_OP       : res_temp = op1 * op2;
			BIC_OP       : res_temp = ~op1 & op2;
		endcase
		// flag N,Z,C,V
		
			flag_q[3] = res_temp[31];
			flag_q[2] = res_temp[31:0] == 0;
		//位移還沒加
		case(opcode)
			ADD_OP       : flag_q[1] = res_temp[32];
			ADD_FLAG_OP  : flag_q[1] = res_temp[32];
			SUB_OP1      : flag_q[1] = ~res_temp[32];
			SUB_OP2      : flag_q[1] = ~res_temp[32];
			SUB_FLAG_OP1 : flag_q[1] = ~res_temp[32];
			default: flag_q[1] = flag[1];
		endcase

		case(opcode)
			ADD_OP       : flag_q[0] = ((op1[31] == 0 && op2[31] == 0 && res_temp[31] == 1) || (op1[31] == 1 && op2[31] == 1 && res_temp[31] == 0));
			ADD_FLAG_OP  : flag_q[0] = ((op1[31] == 0 && op2[31] == 0 && res_temp[31] == 1) || (op1[31] == 1 && op2[31] == 1 && res_temp[31] == 0));
			SUB_OP1      : flag_q[0] = ((op1[31] == 1 && op2[31] == 0 && res_temp[31] == 0) || (op1[31] == 0 && op2[31] == 1 && res_temp[31] == 1));
			SUB_OP2      : flag_q[0] = ((op1[31] == 1 && op2[31] == 0 && res_temp[31] == 1) || (op1[31] == 0 && op2[31] == 1 && res_temp[31] == 0));
			SUB_FLAG_OP1 : flag_q[0] = ((op1[31] == 1 && op2[31] == 0 && res_temp[31] == 0) || (op1[31] == 0 && op2[31] == 1 && res_temp[31] == 1));
			default: flag_q[0] = flag[0];
		endcase
		
		result = res_temp [31:0];
	end
endmodule