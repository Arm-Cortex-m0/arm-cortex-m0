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
            10'd0 : data_0 = 16'h2000; // MOVS     r0,#0x00
            10'd1 : data_0 = 16'h4288; // CMP      r0,r1
            10'd2 : data_0 = 16'hD100; // BNE      0x0000000C
            10'd3 : data_0 = 16'h2201; // MOVS     r2,#0x01
            10'd4 : data_0 = 16'h2202; // MOVS     r2,#0x02
            default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'd0 : data_1 = 16'h2105; // MOVS     r1,#0x05
            10'd1 : data_1 = 16'hD003; // BEQ      0x00000010
            10'd2 : data_1 = 16'hE7FE; // B        0x0000000A
            10'd3 : data_1 = 16'hE7FC; // B        0x0000000A
            10'd4 : data_1 = 16'hE7FA; // B        0x0000000A            
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