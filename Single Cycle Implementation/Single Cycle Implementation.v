`timescale 1ns / 1ps

module CPU_A(clk,PC);

input clk;
wire [31:0]ins_out;
wire RegDst ,Jump, ALUsrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch;
wire [1:0] ALUop;
wire [4:0]write_reg;
wire [31:0]write_data, read_data1, read_data2, address_out, alu_mux_out;
wire [31:0]ALU_out;
wire zero;
wire [3:0]ALU_ctrl;
wire [31:0]DataMem_read_data;
wire [31:0] t0,t1,t2,t3,t4,t5,t6,t7;

output reg [31:0] PC;

initial
    begin
        PC <= 0;
    end

always@(posedge clk)
    begin
        if(Jump)
            begin 
                PC <= PC + 4;
                PC <= { PC [31:28] , ins_out [25:0] , 2'b00 } ; 
            end
        else
            if(Branch && zero)
                begin
                    PC <= PC + 4;
                    PC <= PC +  (address_out << 2);
                end        
            else
                PC <= PC + 4;    
    end
       

ins_memory U2(.clk(clk), .read_address(PC), .instruction(ins_out));
control U3(.signal(ins_out[31:26]), .RegDst(RegDst), .Jump(Jump), .ALUsrc(ALUsrc), 
            .MemtoReg(MemtoReg), .RegWrite(RegWrite), .MemRead(MemRead), .MemWrite(MemWrite), .Branch(Branch), .ALUop(ALUop));
            
register_file U4(.clk(clk) , .RegWrite(RegWrite), .read_reg1(ins_out[25:21]), 
                    .read_reg2(ins_out[20:16]), .write_reg(write_reg), .write_data(write_data), 
                    .read_data1(read_data1), .read_data2(read_data2),
                    .t0(t0),.t1(t1),.t2(t2),.t3(t3),.t4(t4),.t5(t5),.t6(t6),.t7(t7)
                    );
                    
write_register_mux U5(.RegDst(RegDst), .rd(ins_out[15:11]), .rt(ins_out[20:16]), .write_reg(write_reg));
sign_extend U6(.address_in(ins_out[15:0]), .address_out(address_out));
reg2alu_mux U7(.alusrc(ALUsrc), .reg2(read_data2), .extended(address_out), .muxo(alu_mux_out));
alu U8(.alu_ctr(ALU_ctrl), .a(read_data1), .b(alu_mux_out), .alu_out(ALU_out), .zero(zero));
alu_control U9(.func(ins_out[5:0]), .aluop(ALUop), .alu_Ctr(ALU_ctrl));
data_memory U10(.clk(clk), .MemWrite(MemWrite), .MemRead(MemRead), .address(ALU_out), .write_data(read_data2), .read_data(DataMem_read_data)
                );
write_data_mux U11(.MemtoReg(MemtoReg), .read_data(DataMem_read_data), .ALU_result(ALU_out), .write_data(write_data));

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module ins_memory(clk,read_address,instruction);

input clk;
input [31:0]read_address; // PC
output reg[31:0]instruction;

reg [31:0] instruction_memoroy [31:0];

initial
    begin
        // Loop
        instruction_memoroy[0] = 32'b100011_01011_01001_0000000000000000;	 // lw $t1 ,0($t3)
        instruction_memoroy[1] = 32'b001000_01010_01101_0000000000000100;   // addi $t5 ,$t2 , 4
        instruction_memoroy[2] = 32'b000000_01001_01011_01110_00000_100010;   // sub $t6 ,$t1 ,$t3
        instruction_memoroy[3] = 32'b000000_01101_01110_01111_00000_100101;  // or $t7 ,$t5 ,$t6
        instruction_memoroy[4] = 32'b000000_01001_01101_01011_00000_100100;   // and $t3,$t1,$t5 
        instruction_memoroy[5] = 32'b000100_01110_01111_0000000000000001;   // beq $t6 ,$t7,LOOP2
        instruction_memoroy[6] = 32'b000010_00000000000000000000000111;     // j LOOP2
        // Loop2
        instruction_memoroy[7] = 32'b000000_01010_01100_01010_00000_100000; // add $t2 ,$t2 ,$t4
        instruction_memoroy[8] = 32'b101011_01011_01010_0000000000010100;	// sw $t2 ,20($t3)
        instruction_memoroy[9] = 32'b000000_01000_01101_01100_00000_101010; // slt $t4,$t0,$t5
        instruction_memoroy[10] = 32'b000010_00000000000000000000000000;     // j LOOP 
    end

always@(*)
    begin
        instruction <= instruction_memoroy[read_address/4];
    end
          
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module control(signal, RegDst,Jump,ALUsrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUop);
input signal;
output RegDst,ALUsrc,Jump,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUop;
reg[1:0] ALUop;
reg RegDst,ALUsrc,Jump,MemtoReg,RegWrite,MemRead,MemWrite,Branch;
wire[5:0] signal;

always@(signal)
    begin
        case(signal)
            6'b000000:begin //R format
                        RegDst=1'b1; 
                        ALUsrc=1'b0;
                        Jump=1'b0; 
                        MemtoReg=1'b0; 
                        RegWrite=1'b1; 
                        MemRead=1'b0; 
                        MemWrite=1'b0; 
                        Branch=1'b0;
                        ALUop=2'b10; 
                     end
            6'b100011:begin // lw
                        RegDst=1'b0; 
                        ALUsrc=1'b1;
                        Jump=1'b0; 
                        MemtoReg=1'b1;
                        RegWrite=1'b1; 
                        MemRead=1'b1; 
                        MemWrite=1'b0; 
                        Branch=1'b0;
                        ALUop=2'b00; 
                     end
            6'b101011:begin // sw
                        RegDst=1'bX; 
                        ALUsrc=1'b1;
                        Jump=1'b0;  
                        MemtoReg=1'bX; 
                        RegWrite=1'b0; 
                        MemRead=1'b0; 
                        MemWrite=1'b1; 
                        Branch=1'b0;
                        ALUop=2'b00; 
                     end 
            6'b000100:begin // beq
                        RegDst=1'bX; 
                        ALUsrc=1'b0;
                        Jump=1'b0;  
                        MemtoReg=1'bX; 
                        RegWrite=1'b0; 
                        MemRead=1'b0; 
                        MemWrite=1'b0; 
                        Branch=1'b1;
                        ALUop=2'b01; 
                     end
            6'b001000:begin // addi
                        RegDst=1'b0; 
                        ALUsrc=1'b1;
                        Jump=1'b0;  
                        MemtoReg=1'b0;
                        RegWrite=1'b1; 
                        MemRead=1'b0; 
                        MemWrite=1'b0; 
                        Branch=1'b0;
                        ALUop=2'b00; 
                     end
            6'b000010:begin // jump
                        RegDst=1'bX; 
                        ALUsrc=1'bX;
                        Jump=1'b1;  
                        MemtoReg=1'bX; 
                        RegWrite=1'b0; 
                        MemRead=1'b0; 
                        MemWrite=1'b0; 
                        Branch=1'b0;
                        ALUop=2'bXX; 
                     end
            default: begin 
                        RegDst=1'bX; 
                        ALUsrc=1'bX; 
                        MemtoReg=1'bX; 
                        RegWrite=1'bX; 
                        MemRead=1'bX; 
                        MemWrite=1'bX; 
                        Branch=1'bX;
                        ALUop=2'bXX; 
                    end       
        endcase
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module register_file(clk,RegWrite,read_reg1,read_reg2,write_reg,write_data,read_data1,read_data2,t0,t1,t2,t3,t4,t5,t6,t7);

input clk;

input RegWrite; // Control

// read_reg1,read_reg2,write_reg 存 address
input [4:0] read_reg1; // rs
input [4:0] read_reg2; //  R-type : rt , lw : X
input [4:0] write_reg; // R-type : rd , lw : rt

// ALU operation 需要 32-bit data
input [31:0] write_data;
output reg [31:0] read_data1;
output reg [31:0] read_data2;

output reg [31:0] t0,t1,t2,t3,t4,t5,t6,t7;

initial
    begin
        t0 = 0;
        t1 = 1;
        t2 = 2;
        t3 = 3;
        t4 = 4;
        t5 = 5;
        t6 = 6;
        t7 = 7;
    end
    
always@(*)
    begin
        case(read_reg1)
        8 : read_data1 = t0;
        9 : read_data1 = t1;
        10 : read_data1 = t2;
        11 : read_data1 = t3;
        12 : read_data1 = t4;
        13 : read_data1 = t5;
        14 : read_data1 = t6;
        15 : read_data1 = t7;
        default : read_data1 = 32'dX;
        endcase
    end

always@(*)
    begin
        case(read_reg2)
        8 : read_data2 = t0;
        9 : read_data2 = t1;
        10 : read_data2 = t2;
        11 : read_data2 = t3;
        12 : read_data2 = t4;
        13 : read_data2 = t5;
        14 : read_data2 = t6;
        15 : read_data2 = t7;
        default : read_data2 = 32'dX;
        endcase 
    end
    
always@(posedge clk)
    begin
        if(RegWrite)
            begin
                case(write_reg)
                    8 : t0 <= write_data;
                    9 : t1 <= write_data;
                    10 : t2 <= write_data;
                    11 : t3 <= write_data;
                    12 : t4 <= write_data;
                    13 : t5 <= write_data;
                    14 : t6 <= write_data;
                    15 : t7 <= write_data;
                endcase    
            end
        else
            begin
                t0 <= t0;
                t1 <= t1;
                t2 <= t2;
                t3 <= t3;
                t4 <= t4;
                t5 <= t5;
                t6 <= t6;
                t7 <= t7;
            end
    end    
 
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module write_register_mux(RegDst,rd,rt,write_reg);

input RegDst; // Control
input [4:0] rd; // R-type destination
input [4:0] rt; // lw destination

output reg [4:0] write_reg;

always@(*)
    begin
        case(RegDst) // 根據 opcode 得知 Instruction 的 type
            0 : write_reg = rt; // lw
            1 : write_reg = rd; // R-type
            default : write_reg = 5'dX;
        endcase
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module sign_extend(address_in,address_out);

// address 在 MIPS code 裡為 16-bit ,需 sign-extension 成 32-bit , 才能進行ALU運算
input [15:0] address_in;
output [31:0] address_out;

assign address_out = { {16{address_in[15]}} , address_in}; // 正數補0 負數補1

endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module reg2alu_mux(alusrc,reg2,extended,muxo);

input alusrc;
input [31:0]reg2,extended;
output reg [31:0] muxo;

always@(alusrc or reg2 or extended)
    begin
        if(alusrc) 
            muxo = extended;    
        else 
            muxo = reg2;     
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module alu( alu_ctr,a,b,alu_out,zero);
    
    input [31:0] a,b;
    input[3:0] alu_ctr;
    output reg [31:0] alu_out;
    output reg zero;
    
    always@(a or b or alu_ctr)
        begin
            case (alu_ctr)
                4'b0000: alu_out = a & b;
                4'b0001: alu_out = a | b;
                4'b0010: alu_out = a + b;
                4'b0110: alu_out = a - b;
                4'b0111: alu_out = a < b;
                4'b1100: alu_out = a ~^ b;
                default: alu_out = 32'dX;
            endcase 
    end
    
    always@(alu_out)
        begin
            if (alu_out==32'd0)
                zero=1'b1;
            else 
                zero=1'b0;
            end
    
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module alu_control( func,aluop,alu_Ctr);
input [5:0] func;
input [1:0] aluop;
output reg [3:0] alu_Ctr;


always@(func or aluop)
    begin
        case (aluop)
            2'b00: alu_Ctr=4'b0010; // lw,sw
            2'b01: alu_Ctr=4'b0110; // beq
            2'b10:
                begin     
                    if(func==6'b100000) // add
                        alu_Ctr=4'b0010;
                    else if (func==6'b100010) // sub
                        alu_Ctr=4'b0110;
                    else if (func==6'b100100) // AND
                        alu_Ctr=4'b0000;
                    else if (func==6'b100101) // OR
                        alu_Ctr=4'b0001;
                    else if (func==6'b101010) // slt
                        alu_Ctr=4'b0111;
                    else
                        alu_Ctr=4'b1111;
                end
            default: alu_Ctr=4'b1111; 
        endcase
    end


endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module data_memory(clk,MemWrite,MemRead,address,write_data,read_data);

input clk;

input MemWrite; // Control for sw
input MemRead; // Control for lw

input [31:0] address; //  rt address
input [31:0] write_data; // sw
output reg [31:0] read_data; // lw

reg [31:0] memory_data [31:0]; // 32 個 register 的 32-bit data 

initial
    begin
        memory_data[0] = 0;  
        memory_data[1] = 1;  
        memory_data[2] = 2;  
        memory_data[3] = 3;  
        memory_data[4] = 4; 
        memory_data[5] = 5;  
        memory_data[6] = 6;  
        memory_data[7] = 7;  
        memory_data[8] = 0;  
        memory_data[9] = 0;
        memory_data[10] = 0;  
        memory_data[11] = 0;
        memory_data[12] = 0; 
        memory_data[13] = 0;  
        memory_data[14] = 0;  
        memory_data[15] = 0;  
        memory_data[16] = 0;  
        memory_data[17] = 0;  
        memory_data[18] = 0;  
        memory_data[19] = 0;  
        memory_data[20] = 0; 
        memory_data[21] = 0;  
        memory_data[22] = 0;  
        memory_data[23] = 0;  
        memory_data[24] = 0;  
        memory_data[25] = 0;  
        memory_data[26] = 0;  
        memory_data[27] = 0;  
        memory_data[28] = 0; 
        memory_data[29] = 0;  
        memory_data[30] = 0;  
        memory_data[31] = 0;  
    end
    
always@(negedge clk)
    begin
        if(MemWrite && ! MemRead) // sw
             memory_data[address] <= write_data;    
        else 
             memory_data[address] <= memory_data[address]; 
    end
    
always@(*)
    begin
        if(MemRead && !MemWrite) // lw
            read_data = memory_data[address];    
        else 
            read_data = 32'dX; 
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module write_data_mux(MemtoReg,read_data,ALU_result,write_data);

input MemtoReg; // Control

input [31:0] read_data; // lw
input [31:0] ALU_result; // R-format

output reg [31:0] write_data;

always@(*)
    begin
        case(MemtoReg)
            0 : write_data = ALU_result; // R-format
            1 : write_data = read_data; // lw
            default : write_data = 32'dX;
        endcase
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
