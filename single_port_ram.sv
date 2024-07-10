module single_port_ram
	(
		input  logic        clk,           
		input  logic        rst,           
		input  logic [1:0]  size,          // 資料大小選擇（00: 8-bit, 01: 16-bit, 10: 32-bit）
		input  logic        write_enable,  
		input  logic [9:0]  address,       // 記憶體位址
		input  logic [31:0] write_data,    
		output logic [31:0] read_data      
	);
	
	// 四塊8位元寬度的記憶體
	// reg [DATA_WIDTH-1:0] ram[2**ADDR_WIDTH-1:0];
    logic [7:0] mem0 [255:0];
    logic [7:0] mem1 [255:0];
    logic [7:0] mem2 [255:0];
    logic [7:0] mem3 [255:0];
	
	 // 暫存讀取數據
    logic [31:0] temp_read_data;
	
    // 寫操作
    always_ff @(posedge clk) begin
        if (write_enable) begin
            case (size)  // 論文裡的8/16/32_Arbitrator 訊號 選擇寫入幾bits資料
                2'b00: begin // 8-bit
					case (address[1:0])
						2'b00: mem0[address[9:2]] <= write_data[7:0];
						2'b01: mem1[address[9:2]] <= write_data[7:0];
						2'b10: mem2[address[9:2]] <= write_data[7:0];
						2'b11: mem3[address[9:2]] <= write_data[7:0];
					endcase
                end
                2'b01: begin // 16-bit
                    if (address[1] == 1'b0) begin
                        mem0[address[9:2]] <= write_data[7:0];
                        mem1[address[9:2]] <= write_data[15:8];
                    end else begin
                        mem2[address[9:2]] <= write_data[7:0];
                        mem3[address[9:2]] <= write_data[15:8];
                    end
                end
                2'b10: begin // 32-bit
                    mem0[address[9:2]] <= write_data[7:0];
                    mem1[address[9:2]] <= write_data[15:8];
                    mem2[address[9:2]] <= write_data[23:16];
                    mem3[address[9:2]] <= write_data[31:24];
                end
            endcase
        end
    end

    // 讀操作
    always_ff @(negedge clk) begin
        if (rst) begin
            temp_read_data <= 32'b0;
        end 
		else begin
            case (size) // 論文裡的8/16/32_Arbitrator 訊號 選擇讀取出幾bits資料
                2'b00: begin // 8-bit
                    case (address[1:0]) 
                        2'b00: temp_read_data <= {24'b0, mem0[address[9:2]]};
                        2'b01: temp_read_data <= {24'b0, mem1[address[9:2]]};
                        2'b10: temp_read_data <= {24'b0, mem2[address[9:2]]};
                        2'b11: temp_read_data <= {24'b0, mem3[address[9:2]]};
                    endcase
                end
                2'b01: begin // 16-bit
                    if (address[1] == 1'b0) begin
                        temp_read_data <= {16'b0, mem1[address[9:2]], mem0[address[9:2]]};
                    end else begin
                        temp_read_data <= {16'b0, mem3[address[9:2]], mem2[address[9:2]]};
                    end
                end
                2'b10: begin // 32-bit
                    temp_read_data <= {mem3[address[9:2]], mem2[address[9:2]], mem1[address[9:2]], mem0[address[9:2]]};
                end
            endcase
        end
    end

    // 讀取資料輸出
    assign read_data = temp_read_data;
	
endmodule