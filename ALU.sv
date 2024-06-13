module ALU
(	
	input logic  [3:0]opcode,
	input logic  [31:0]op1,op2,
	input logic  [3:0]flag,
	output logic [3:0]flag_q,
	output logic [31:0]result
);
	logic [32:0] res_temp;
	/*parameter ADD = 0,
			  ADC = 1,
			  SUB = 2,
			  SUB_r = 3,
			  SUBC = 4,
			  AND = 5,
			  OR = 6,
			  XOR = 7,
			  MOV = 8,
			  ADD1 = 9,
			  SUB1 = 10,
			  ZERO = 11,
			  NOR = 12;*/
		
	always_comb
	begin 
		unique case(opcode)
			4'h0: res_temp = op1 + op2;
			4'h1: res_temp = op1 + op2 + flag[1];
			4'h2: res_temp = op1 - op2;
			4'h3: res_temp = op2 - op1;
			4'h4: res_temp = op1 - op2 - flag[1];
			4'h5: res_temp = op1 & op2;
			4'h6: res_temp = op1 | op2;
			4'h7: res_temp = op1 ^ op2;
			4'h8: res_temp = op1;
			4'h9: res_temp = op1 + 1;
			4'hA: res_temp = op1 - 1;
			4'hB: res_temp = 0;
			4'hC: res_temp = ~op1;
		endcase
		// flag N,Z,C,V
		
			flag_q[3] = res_temp[31];
			flag_q[2] = res_temp[31:0] == 0;
		//位移還沒加
		case(opcode)
			4'h0: flag_q[1] = res_temp[32];
			4'h1: flag_q[1] = res_temp[32];
			4'h2: flag_q[1] = ~res_temp[32];
			4'h3: flag_q[1] = ~res_temp[32];
			4'h4: flag_q[1] = ~res_temp[32];
			default: flag_q[1] = flag[1];
		endcase

		case(opcode)
			4'h0: flag_q[0] = ((op1[31] == 0 && op2[31] == 0 && res_temp[31] == 1) || (op1[31] == 1 && op2[31] == 1 && res_temp[31] == 0));
			4'h1: flag_q[0] = ((op1[31] == 0 && op2[31] == 0 && res_temp[31] == 1) || (op1[31] == 1 && op2[31] == 1 && res_temp[31] == 0));
			4'h2: flag_q[0] = ((op1[31] == 1 && op2[31] == 0 && res_temp[31] == 0) || (op1[31] == 0 && op2[31] == 1 && res_temp[31] == 1));
			4'h3: flag_q[0] = ((op1[31] == 1 && op2[31] == 0 && res_temp[31] == 1) || (op1[31] == 0 && op2[31] == 1 && res_temp[31] == 0));
			4'h4: flag_q[0] = ((op1[31] == 1 && op2[31] == 0 && res_temp[31] == 0) || (op1[31] == 0 && op2[31] == 1 && res_temp[31] == 1));
			default: flag_q[0] = flag[0];
		endcase
		
		
		
		result = res_temp [31:0];
	end
endmodule