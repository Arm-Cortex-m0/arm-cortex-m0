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
            10'd0 : data_0 = 16'h210F; // MOVS     r1,#0x0F
            10'd1 : data_0 = 16'h4011; // ANDS     r1,r1,r2
            10'd2 : data_0 = 16'h22F0; // MOVS     r2,#0xF0
            10'd3 : data_0 = 16'h2101; // MOVS     r1,#0x01
            10'd4 : data_0 = 16'h4091; // LSLS     r1,r1,r2
            10'd5 : data_0 = 16'h2202; // MOVS     r2,#0x02
            10'd6 : data_0 = 16'h21F0; // MOVS     r1,#0xF0
            10'd7 : data_0 = 16'h4111; // ASRS     r1,r1,r2
            10'd8 : data_0 = 16'h2201; // MOVS     r2,#0x01
            10'd9 : data_0 = 16'h2102; // MOVS     r1,#0x02
            10'd10 : data_0 = 16'h4191; // SBCS     r1,r1,r2
            10'd11 : data_0 = 16'h2201; // MOVS     r2,#0x01
            10'd12 : data_0 = 16'h210F; // MOVS     r1,#0x0F
            10'd13 : data_0 = 16'h4211; // TST      r1,r2
            10'd14 : data_0 = 16'h424A; // RSBS     r2,r1,#0
            10'd15 : data_0 = 16'h2201; // MOVS     r2,#0x01
            10'd16 : data_0 = 16'h2101; // MOVS     r1,#0x01
            10'd17 : data_0 = 16'h42D1; // CMN      r1,r2
            10'd18 : data_0 = 16'h22F0; // MOVS     r2,#0xF0
            10'd19 : data_0 = 16'h2102; // MOVS     r1,#0x02
            10'd20 : data_0 = 16'h4351; // MULS     r1,r2,r1
            10'd21 : data_0 = 16'h2210; // MOVS     r2,#0x10
            10'd22 : data_0 = 16'h210F; // MOVS     r1,#0x0F
            default: data_0 = 16'h0;
        endcase
    end

    // Mem_1
    logic [13:0] mem_1_addr;
    logic [15:0] data_1;
    assign mem_1_addr = Rom_addr_in;
    always_comb begin
        case (mem_1_addr)
            10'd0 : data_1 = 16'h22F0; // MOVS     r2,#0xF0
            10'd1 : data_1 = 16'h210F; // MOVS     r1,#0x0F
            10'd2 : data_1 = 16'h4051; // EORS     r1,r1,r2
            10'd3 : data_1 = 16'h2202; // MOVS     r2,#0x02
            10'd4 : data_1 = 16'h2108; // MOVS     r1,#0x08
            10'd5 : data_1 = 16'h40D1; // LSRS     r1,r1,r2
            10'd6 : data_1 = 16'h2202; // MOVS     r2,#0x02
            10'd7 : data_1 = 16'h2101; // MOVS     r1,#0x01
            10'd8 : data_1 = 16'h4151; // ADCS     r1,r1,r2
            10'd9 : data_1 = 16'h2201; // MOVS     r2,#0x01
            10'd10 : data_1 = 16'h2101; // MOVS     r1,#0x01
            10'd11 : data_1 = 16'h41D1; // RORS     r1,r1,r2
            10'd12 : data_1 = 16'h22F0; // MOVS     r2,#0xF0
            10'd13 : data_1 = 16'h2101; // MOVS     r1,#0x01
            10'd14 : data_1 = 16'h2101; // MOVS     r1,#0x01
            10'd15 : data_1 = 16'h4291; // CMP      r1,r2
            10'd16 : data_1 = 16'h22FF; // MOVS     r2,#0xFF
            10'd17 : data_1 = 16'h210F; // MOVS     r1,#0x0F
            10'd18 : data_1 = 16'h4311; // ORRS     r1,r1,r2
            10'd19 : data_1 = 16'h2203; // MOVS     r2,#0x03
            10'd20 : data_1 = 16'h2111; // MOVS     r1,#0x11
            10'd21 : data_1 = 16'h4391; // BICS     r1,r1,r2
            10'd22 : data_1 = 16'h43CA; // MVNS     r2,r1            
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