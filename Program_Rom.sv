module Program_Rom
	(
		input [13:0] Rom_addr_in,
		input pc_1,
		input sel_mem_1,
		input [1:0] sel_mem_0,
		output logic [15:0] IR_0,
		output logic [15:0] IR_1
	);
    
	// Mem_0
	logic [13:0] mem_0_addr;
	logic [15:0] data_0;
	assign mem_0_addr = Rom_addr_in + pc_1;
    always_comb begin
        case (mem_0_addr)
			10'h0 : data_0 = 16'h2002; // MOVS R0, #2
			10'h1 : data_0 = 16'h2209; // MOVS R2, #9
			default: data_0 = 16'h0;   
		endcase
    end
	
	// Mem_1
	logic [13:0] mem_1_addr;
	logic [15:0] data_1;
	assign mem_1_addr = Rom_addr_in;
	always_comb begin
        case (mem_1_addr)
			10'h0 : data_1 = 16'h2105; // MOVS R1, #5
			10'h1 : data_1 = 16'h1800; // ADDS R0, R0, R0
			default: data_1 = 16'h0;   
		endcase
    end

	// Mux1
	always_comb begin
		case (sel_mem_1)
		0: IR_1 = data_0;
		1: IR_1 = data_1;
		endcase
	end
	
	// Mux2
	always_comb begin
		unique case (sel_mem_0)
		0: IR_0 = data_0;
		1: IR_0 = IR_1;
		2: IR_0 = data_1;
		endcase
	end

endmodule
