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
            10'h0 : data_0 = 16'h210A; // MOVS     r1,#0x0A
            10'h1 : data_0 = 16'h0953; // LSRS     r3,r2,#5
            10'h2 : data_0 = 16'h184D; // ADDS     r5,r1,r1
            10'h3 : data_0 = 16'h1D4D; // ADDS     r5,r1,#5
            10'h4 : data_0 = 16'h357D; // ADDS     r5,r5,#0x7D
            10'h5 : data_0 = 16'h2009; // MOVS     r0,#0x09
            10'h6 : data_0 = 16'h014A; // LSLS     r2,r1,#5
            10'h7 : data_0 = 16'h1154; // ASRS     r4,r2,#5          
			default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'h0 : data_1 = 16'h014A; // LSLS     r2,r1,#5
            10'h1 : data_1 = 16'h1154; // ASRS     r4,r2,#5
            10'h2 : data_1 = 16'h1A6E; // SUBS     r6,r5,r1
            10'h3 : data_1 = 16'h1F4E; // SUBS     r6,r1,#5
            10'h4 : data_1 = 16'h3D0F; // SUBS     r5,r5,#0x0F
            10'h5 : data_1 = 16'h43C1; // MVNS     r1,r0
            10'h6 : data_1 = 16'h0953; // LSRS     r3,r2,#5
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