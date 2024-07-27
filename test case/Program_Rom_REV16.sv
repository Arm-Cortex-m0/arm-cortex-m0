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
            10'd0 : data_0 = 16'h2012; // MOVS     r0,#0x12
            10'd1 : data_0 = 16'h2134; // MOVS     r1,#0x34
            10'd2 : data_0 = 16'h4308; // ORRS     r0,r0,r1
            10'd3 : data_0 = 16'h0209; // LSLS     r1,r1,#8
            10'd4 : data_0 = 16'h2178; // MOVS     r1,#0x78
            10'd5 : data_0 = 16'hBA02; // REV      r2,r0
            10'd6 : data_0 = 16'h0600; // LSLS     r0,r0,#24
            10'd7 : data_0 = 16'h0409; // LSLS     r1,r1,#16
            10'd8 : data_0 = 16'h2156; // MOVS     r1,#0x56
            10'd9 : data_0 = 16'h4308; // ORRS     r0,r0,r1
            10'd10 : data_0 = 16'h4308; // ORRS     r0,r0,r1
            10'd11 : data_0 = 16'h2012; // MOVS     r0,#0x12
            10'd12 : data_0 = 16'h2134; // MOVS     r1,#0x34
            10'd13 : data_0 = 16'hBAC2; // REVSH    r2,r0            
			default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'd0 : data_1 = 16'h0600; // LSLS     r0,r0,#24
            10'd1 : data_1 = 16'h0409; // LSLS     r1,r1,#16
            10'd2 : data_1 = 16'h2156; // MOVS     r1,#0x56
            10'd3 : data_1 = 16'h4308; // ORRS     r0,r0,r1
            10'd4 : data_1 = 16'h4308; // ORRS     r0,r0,r1
            10'd5 : data_1 = 16'h2012; // MOVS     r0,#0x12
            10'd6 : data_1 = 16'h2134; // MOVS     r1,#0x34
            10'd7 : data_1 = 16'h4308; // ORRS     r0,r0,r1
            10'd8 : data_1 = 16'h0209; // LSLS     r1,r1,#8
            10'd9 : data_1 = 16'h2178; // MOVS     r1,#0x78
            10'd10 : data_1 = 16'hBA42; // REV16    r2,r0
            10'd11 : data_1 = 16'h0200; // LSLS     r0,r0,#8
            10'd12 : data_1 = 16'h4308; // ORRS     r0,r0,r1
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