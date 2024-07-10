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
            10'd0 : data_0 = 16'h2501; // MOVS     r5,#0x01
            10'd1 : data_0 = 16'h2090; // MOVS     r0,#0x90
            10'd2 : data_0 = 16'h2190; // MOVS     r1,#0x90
            10'd3 : data_0 = 16'h1842; // ADDS     r2,r0,r1
            default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'd0 : data_1 = 16'h2601; // MOVS     r6,#0x01
            10'd1 : data_1 = 16'h0600; // LSLS     r0,r0,#24
            10'd2 : data_1 = 16'h0609; // LSLS     r1,r1,#24
            10'd3 : data_1 = 16'h4175; // ADCS     r5,r5,r6            
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