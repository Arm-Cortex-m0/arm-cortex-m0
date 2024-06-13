module inst_decoder
    (
		input logic [15:0] ir_q0,
		input logic [15:1] ir_q1,
		output logic [6:0] cmd,
		// 現在只會用到低位暫存器 0~7 但如果有權限較高的指令用到高位暫存器，就會需要 4bit 如規格書 ADD_reg 的 T2
		output logic [3:0] Rm,  // 低階指令只能存取權限較低的暫存器 0~7，所以是 3 個 bit
		output logic [3:0] Rn,  // 低階指令只能存取權限較低的暫存器 0~7，所以是 3 個 bit
		output logic [3:0] Rd,  // 低階指令只能存取權限較低的暫存器 0~7，所以是 3 個 bit
		output logic [31:0] imm 
		// 雖然論文的 16bit 指令中，最大的立即數為 11 個 bit ( imm11 ) ( 這種寫法以後 signed bit 有問題再改 )
	);
	
	
	parameter LSL_imm  = 7'd0,
			  LSR_imm  = 7'd1,
			  ASR_imm  = 7'd2, 
			  ADD_reg  = 7'd3,
			  SUB_reg  = 7'd4,
			  ADD_imm3 = 7'd5, 
			  SUB_imm3 = 7'd6,  
			  MOV_imm  = 7'd7, 
			  CMP_imm  = 7'd8,  
			  ADD_imm8 = 7'd9,  
			  SUB_imm8 = 7'd10;
	
	assign isThumb = ir_q0[15:11] != 5'b11101 &&
					 ir_q0[15:11] != 5'b11110 &&
					 ir_q0[15:11] != 5'b11111;
	
	always_comb begin
		if(isThumb) 
			begin
				// casez 如果某些位子的值為高阻 z，那麼對這些位子的比較就會忽略
				// 我們的 ir_q 都是精確值。
				unique casez(ir_q0[15:14])			
					2'b00: //規格書 p.85
					begin
						unique casez(ir_q0[13:9])
							//5'b000xx: cmd = LSL_imm;  //0
							//5'b001xx: cmd = LSR_imm;  //1
							//5'b010xx: cmd = ASR_imm;  //2
							5'b01100: cmd = ADD_reg;  //3
							5'b01101: cmd = SUB_reg;  //4
							5'b01110: cmd = ADD_imm3; //5
							5'b01111: cmd = SUB_imm3; //6
							5'b100??: cmd = MOV_imm;  //7
							//5'b101xx: cmd = CMP_imm;  //8
							//5'b110xx: cmd = ADD_imm8; //9
							//5'b111xx: cmd = SUB_imm8; //10
							default: cmd = 0;
						endcase
					end
					default: cmd = 0;
				endcase
			end
		else 
			begin
				// 32bit
			end
	end

	
	
	
	logic [4:0] imm5;
	logic [7:0] imm8;
	logic [10:0] imm10;
	always_comb begin
		case(cmd)
			LSL_imm:  begin imm5 = ir_q0[10:6]; Rm = ir_q0[5:3]; Rd = ir_q0[2:0]; end //規格書 p.150 
			LSR_imm:  begin imm5 = ir_q0[10:6]; Rm = ir_q0[5:3]; Rd = ir_q0[2:0]; end //規格書 p.152
			ASR_imm:  begin imm5 = ir_q0[10:6]; Rm = ir_q0[5:3]; Rd = ir_q0[2:0]; end //規格書 p.117
			ADD_reg:  begin Rm = ir_q0[8:6]; Rn = ir_q0[5:3]; Rd = ir_q0[2:0]; end //規格書 p.109 見註解[1] 
			SUB_reg:  begin end //規格書 p. 
			ADD_imm3: begin end //規格書 p.
			SUB_imm3: begin end //規格書 p.
			MOV_imm:  begin Rd = ir_q0[10:8]; imm8 = ir_q0[7:0]; end //規格書 p.154
			CMP_imm:  begin end //規格書 p.
			ADD_imm8: begin end //規格書 p.
			SUB_imm8: begin end //規格書 p.
			default: begin
				// Do nothing
			end
		endcase
	end
	
	assign imm = imm8;

	/*
	always_comb 
    begin
        casex(ir_q[15:10])
            6'b00xxxx: 
                casex
                    
                end
            
    
        endcase
    end
	
	*/
	// 因為是 Thumb-1 和 Thumb-2 混合的 decoder
    // 所以有可能解出 16bit 指令或是 32bit 指令
    // 所以 input 是 32bit	
endmodule


/*
assign Data_processing = 6'b010000; // Data processing 010000XXXX p.86
assign AND     = ir_q[15:6] == {Data_processing,4'b0000};
assign EOR 	   = ir_q[15:6] == {Data_processing,4'b0001};
assign LSL_reg = ir_q[15:6] == {Data_processing,4'b0010};
assign LSR_reg = ir_q[15:6] == {Data_processing,4'b0011};
assign ASR_reg = ir_q[15:6] == {Data_processing,4'b0100};
assign ADC 	   = ir_q[15:6] == {Data_processing,4'b0101};
assign SBC     = ir_q[15:6] == {Data_processing,4'b0110};
assign ROR     = ir_q[15:6] == {Data_processing,4'b0111};
assign TST     = ir_q[15:6] == {Data_processing,4'b1000};
assign RSB_imm = ir_q[15:6] == {Data_processing,4'b1001};
assign CMP_reg = ir_q[15:6] == {Data_processing,4'b1010} | {SBE,2'b01}; //from Encoding T2 in p.129 ;
assign CMN     = ir_q[15:6] == {Data_processing,4'b1011};
assign ORR     = ir_q[15:6] == {Data_processing,4'b1100};
assign MUL     = ir_q[15:6] == {Data_processing,4'b1101};
assign BIC     = ir_q[15:6] == {Data_processing,4'b1110};
assign MVN     = ir_q[15:6] == {Data_processing,4'b1111};

assign  SBE = 6'b010001; // Special data instructions and branch and exchange 010001XXXX p.87

assign ADD_reg = ir_q[15:6] == {SBE,2'b00};
//assign CMP_reg  add to Data_processing  p.129
assign MOV_reg = ir_q[15:6] == {SBE,2'b10};
assign BX      = ir_q[15:6] == {SBE,3'b110};
assign BLX     = ir_q[15:6] == {SBE,3'b111};

// Load/store single data item p.88

assign STR_reg   = ir_q[15:9]  == 7'b0101000; 
assign STRH_reg  = ir_q[15:9]  == 7'b0101001; 
assign STRB_reg  = ir_q[15:9]  == 7'b0101010;  
assign LDRSB_reg = ir_q[15:9]  == 7'b0101011;
assign LDR_reg   = ir_q[15:9]  == 7'b0101100; 
assign LDRH_reg  = ir_q[15:9]  == 7'b0101101; 
assign LDRB_reg  = ir_q[15:9]  == 7'b0101110;  
assign LDRSH_imm = ir_q[15:9]  == 7'b0101111;
assign STR_imm   = ir_q[15:11] == 5'b01100 | ; 
assign LDR_imm   = ir_q[15:11] == 5'b01101 | ; 
assign STRB_imm  = ir_q[15:11] == 5'b01110; 
assign LDRB_imm  = ir_q[15:11] == 5'b01111; 




assign LSL_imm  = ir_q[15:11] == {Shift,3'b000}; 
assign LSR_imm  = ir_q[15:11] == {Shift,3'b001}; 
assign ASR_imm  = ir_q[15:11] == {Shift,3'b010};  
assign ADD_reg  = ir_q[15:9]  == {Shift,5'b01100}; 
assign SUB_reg  = ir_q[15:9]  == {Shift,5'b01101}; 
assign ADD_imm3 = ir_q[15:9]  == {Shift,5'b01110};  
assign SUB_imm3 = ir_q[15:9]  == {Shift,5'b01111};   
assign MOV_imm  = ir_q[15:11] == {Shift,3'b100};  
assign CMP_imm  = ir_q[15:11] == {Shift,3'b101};   
assign ADD_imm8 = ir_q[15:11] == {Shift,3'b110};   
assign SUB_imm8 = ir_q[15:11] == {Shift,3'b111};  


*/

/*
註解：
[1]: p.109 的 T2 是 p.87 的另外一組 parameter，放牛班說 T1, T2 只是第一種第二種不是 Thumb1 Thumb2

*/