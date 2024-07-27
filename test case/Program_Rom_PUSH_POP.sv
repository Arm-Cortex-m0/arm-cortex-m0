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
            10'd0 : data_0 = 16'h210A; // MOVS     r1,#0x0A
            10'd1 : data_0 = 16'h230E; // MOVS     r3,#0x0E
            10'd2 : data_0 = 16'hBC70; // POP      {r4-r6}            
			default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'd0 : data_1 = 16'h220C; // MOVS     r2,#0x0C
            10'd1 : data_1 = 16'hB40E; // PUSH     {r1-r3}
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