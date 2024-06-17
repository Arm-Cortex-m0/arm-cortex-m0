'''
Author : Yuhui Chen
Instruction set : RISC-V
Target : Translating machine code to Rom.sv
Tool : VSCode
Package : RISC-V Venus Simulator
'''

w_file = open("Program_Rom.sv", "w")
r_file = open("./asm/ass.txt", "r")

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


w_file.write("always_comb begin\n")
w_file.write("    case (mem_0_addr)\n")
for i in range(len(mem0)):
    w_file.write("        10'h{idx} : data_0 = 16'h{inst}; // {cmt}".format(idx=i, inst=mem0[i], cmt=com0[i]))
w_file.write("        default: data_0 = 16'h0;\n")
w_file.write("    endcase\n")
w_file.write("end\n\n")


w_file.write("always_comb begin\n")
w_file.write("    case (mem_1_addr)\n")
for i in range(len(mem1)):
    w_file.write("        10'h{idx} : data_1 = 16'h{inst}; // {cmt}".format(idx=i, inst=mem1[i], cmt=com1[i]))
w_file.write("        default: data_1 = 16'h0;\n")
w_file.write("    endcase\n")
w_file.write("end\n\n")

r_file.close()
w_file.close()


# print("网站名：{name}, 地址 {url}".format(name="菜鸟教程", url="www.runoob.com"))
