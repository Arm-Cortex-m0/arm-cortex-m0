w_file = open("Program_Rom.sv", "w")
r_file = open("b_test2.txt", "r")

mem0 = []
com0 = []
mem1 = []
com1 = []

i = 0
for line in r_file.readlines():
    if i % 2 == 0:
        mem0.append(line[11:15])
        com0.append(line[21:])
    else:
        mem1.append(line[11:15])
        com1.append(line[21:])
    i = i + 1

print(mem0)
print(com0)
print(mem1)
print(com1)


w_file.write("module Program_Rom\n")
w_file.write("	  (\n")
w_file.write("        input [13:0] Rom_addr_in,\n")
w_file.write("        input pc_1,\n")
w_file.write("        input sel_mem_1,\n")
w_file.write("        input [1:0] sel_mem_0,\n")
w_file.write("        output logic [15:0] IR_0,\n")
w_file.write("        output logic [15:0] IR_1\n")
w_file.write("    );\n")

w_file.write("\n")
w_file.write("    // Mem_0\n")
w_file.write("    logic [13:0] mem_0_addr;\n")
w_file.write("    logic [15:0] data_0;\n")
w_file.write("    assign mem_0_addr = Rom_addr_in + pc_1;\n")
w_file.write("    always_comb begin\n")
w_file.write("        case (mem_0_addr)\n")
for i in range(len(mem0)):
    w_file.write("            10'd{idx} : data_0 = 16'h{inst}; // {cmt}".format(idx=i, inst=mem0[i], cmt=com0[i]))
w_file.write("            default: data_0 = 16'h0;\n")
w_file.write("        endcase\n")
w_file.write("    end\n\n")

w_file.write("    // Mem_1\n")
w_file.write("    logic [13:0] mem_1_addr;\n")
w_file.write("    logic [15:0] data_1;\n")
w_file.write("    assign mem_1_addr = Rom_addr_in;\n")
w_file.write("    always_comb begin\n")
w_file.write("        case (mem_1_addr)\n")
for i in range(len(mem1)):
    w_file.write("            10'd{idx} : data_1 = 16'h{inst}; // {cmt}".format(idx=i, inst=mem1[i], cmt=com1[i]))
w_file.write("            default: data_1 = 16'h0;\n")
w_file.write("        endcase\n")
w_file.write("    end\n\n")

w_file.write("    // Mux1\n")
w_file.write("    always_comb begin\n")
w_file.write("        case (sel_mem_1)\n")
w_file.write("        0: IR_1 = data_0;\n")
w_file.write("        1: IR_1 = data_1;\n")
w_file.write("        endcase\n")
w_file.write("    end\n")

w_file.write("    // Mux2\n")
w_file.write("    always_comb begin\n")
w_file.write("        unique case (sel_mem_0)\n")
w_file.write("        0: IR_0 = data_0;\n")
w_file.write("        1: IR_0 = IR_1;\n")
w_file.write("        2: IR_0 = data_1;\n")
w_file.write("        endcase\n")
w_file.write("    end\n")


w_file.write("\n")
w_file.write("endmodule")

r_file.close()
w_file.close()
