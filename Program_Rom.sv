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
            10'h0 : data_0 = 16'h2014; // MOVS     r0,#0x14
            10'h1 : data_0 = 16'h6008; // STR      r0,[r1,#0x00]
            10'h2 : data_0 = 16'h680C; // LDR      r4,[r1,#0x00]
            default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'h0 : data_1 = 16'h2104; // MOVS     r1,#0x04
            10'h1 : data_1 = 16'h2428; // MOVS     r4,#0x28
            10'h2 : data_1 = 16'hE7FE; // B        0x0000000A            default: data_1 = 16'h0;
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