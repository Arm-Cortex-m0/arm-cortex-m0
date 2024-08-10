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
            10'd1 : data_0 = 16'h0209; // LSLS     r1,r1,#8
            10'd2 : data_0 = 16'h2256; // MOVS     r2,#0x56
            10'd3 : data_0 = 16'h3278; // ADDS     r2,r2,#0x78
            10'd4 : data_0 = 16'h021B; // LSLS     r3,r3,#8
            10'd5 : data_0 = 16'h24DE; // MOVS     r4,#0xDE
            10'd6 : data_0 = 16'h34F0; // ADDS     r4,r4,#0xF0
            10'd7 : data_0 = 16'h2655; // MOVS     r6,#0x55
            10'd8 : data_0 = 16'h023F; // LSLS     r7,r7,#8
            10'd9 : data_0 = 16'h7304; // STRB     r4,[r0,#0x0C]
            10'd10 : data_0 = 16'h7B01; // LDRB     r1,[r0,#0x0C]
            default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'd0 : data_1 = 16'h2112; // MOVS     r1,#0x12
            10'd1 : data_1 = 16'h3134; // ADDS     r1,r1,#0x34
            10'd2 : data_1 = 16'h0212; // LSLS     r2,r2,#8
            10'd3 : data_1 = 16'h239A; // MOVS     r3,#0x9A
            10'd4 : data_1 = 16'h33BC; // ADDS     r3,r3,#0xBC
            10'd5 : data_1 = 16'h0224; // LSLS     r4,r4,#8
            10'd6 : data_1 = 16'h25A5; // MOVS     r5,#0xA5
            10'd7 : data_1 = 16'h273C; // MOVS     r7,#0x3C
            10'd8 : data_1 = 16'h3778; // ADDS     r7,r7,#0x78
            10'd9 : data_1 = 16'h7407; // STRB     r7,[r0,#0x10]
            10'd10 : data_1 = 16'h7C02; // LDRB     r2,[r0,#0x10]            default: data_1 = 16'h0;
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