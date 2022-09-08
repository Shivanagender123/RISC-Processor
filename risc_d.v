`timescale 1ns / 1ps
`include "alu_d.v"

// File name : risc_d.v
//
// Author : Lopelli Shiva Nagender Rao
//
// Description : This file contains 32bit Plipelined RISC Processor with the following modules
//    		 - Instruction Memory
//        	 - GPR Register File	
//		 - Pipeline register
//               - Fetch Unit
//		 - Decode Unit
//               - Exceution Unit
//		 - Write Back Unit

//Instruction unit Implemented by behaviuoral model
//	- Size of 128 Bytes.
//      - Each Instruction has 4 Bytes
module Instruction_Memory #(parameter INST_SIZE = 32, parameter PC_SIZE = 5) (CLK,RST_N,read_en,PC,INSTRUCTION);
input                        CLK;
input                        RST_N;
input                        read_en;
input      [PC_SIZE-1   : 0] PC; 
output reg [INST_SIZE-1 : 0] INSTRUCTION;

reg  [7:0] memory [127:0]; //As mentioned in PDF
 
	always@(posedge CLK or negedge RST_N)
	begin
		if(~RST_N)
		begin
			INSTRUCTION <= 0;
		end
		else 
		begin
			if((read_en == 1'b1) && (PC[1:0] == 2'b00)) //Address 0 or 4 aligned and read is true
			begin	
				INSTRUCTION <= {memory[PC+3],memory[PC+2],memory[PC+1],memory[PC+0]};
			end	
		end
	end

endmodule

//GPR Register File Implemented by behavioural model
//	- Contains 32 muber of 32bit registers
//	- Value stored in 2's complement 
//	- A & B Ports for reading
//	- C for datab Data writing
module GPR_Register_File #(parameter REG_ADDR_SIZE = 5, parameter REG_DATA_SIZE =32) (CLK, reg_write_en, reg_read_en, reg_write_dest_C, reg_write_data_C, reg_read_addr_A, reg_read_data_A, reg_read_addr_B, reg_read_data_B);
input                          CLK;
input                          reg_read_en;
// write port
input                          reg_write_en;
input      [REG_ADDR_SIZE-1:0] reg_write_dest_C;
input      [REG_DATA_SIZE-1:0] reg_write_data_C;
//Read port A
input      [REG_ADDR_SIZE-1:0] reg_read_addr_A;
output reg [REG_DATA_SIZE-1:0] reg_read_data_A;
//Read port B
input      [REG_ADDR_SIZE-1:0] reg_read_addr_B;
output reg [REG_DATA_SIZE-1:0] reg_read_data_B;

reg [REG_DATA_SIZE-1:0] reg_array [(2**REG_ADDR_SIZE)-1:0];
reg [31:0] data,data_A,data_B;
 
	//Write into C
	always @ (negedge CLK )
	begin
		if(reg_write_en) 
		begin
			data = 0-reg_write_data_C; //Stored in 2's complement
			reg_array[reg_write_dest_C] <= data;
		end
	end

	//Read from A
	always @ (posedge CLK )
	begin
		if(reg_read_en) 
		begin
			data_A = 0-reg_array[reg_read_addr_A]; //Stored in 2's complement	
			reg_read_data_A <= data_A;
		end
	end

        //Read from B
	always @ (posedge CLK )
	begin
		if(reg_read_en) 
		begin
			data_B = 0-reg_array[reg_read_addr_B]; //Stored in 2's complement
			reg_read_data_B <= data_B;
		end
	end

endmodule

// Pipeline Register 
// Used to pipeline the signal
module Pipeline_Register (CLK,IN,OUT);
input      CLK;
input      IN;
output reg OUT;

	always @(posedge CLK)
	begin	
		OUT <= IN;
	end
	
endmodule
 

// Instruction fetch unit implemented by behaviour/structural model
// which responsible for following tasks :
//	- PC increment
// 	- Instrunction data fetch from Instruction memory
// It has 
//	- Instruction memory in it.
module Fetch_Unit #( parameter INST_SIZE = 32,	parameter PC_SIZE = 5)(CLK, RST_N, en, INSTRUCTION);
input                       CLK;
input                       RST_N;
input                       en; // enable instruction fetch from control unit !!
output    [INST_SIZE -1: 0] INSTRUCTION;

reg [PC_SIZE-1:0] PC;

Instruction_Memory #(INST_SIZE, PC_SIZE) inst_mem(.CLK(CLK),.RST_N(RST_N),.read_en(en),.PC(PC),.INSTRUCTION(INSTRUCTION));

	always @(posedge CLK or negedge RST_N)
	begin
		if(~RST_N) 
		begin
			PC <= 'd0;
		end
		else 
		begin
			if(en == 1'b1)PC <= PC+4;//As each instruction is located in 4locations
		end
	end

endmodule

// Instrucntion decode unit is implemented by behaviour model
// It performs following tasks :
//	- Decoding instruction
//	- Providing register data to execute unit
//	- Enable signal generation for execution unit
//	- Write register with updated results
// It has following modules
//	- GPR_Register_File
module Decode_Unit #(parameter INST_SIZE = 32, parameter REG_ADDR_SIZE = 5, parameter REG_DATA_SIZE = 32)(CLK,RST_N,en,INSTRUCTION,reg_wr_addr,reg_wr_en,reg_wr_data,reg_A_val,reg_B_val,mul_en,shift_en,xor_en,nor_en,INSTRUCTION_o);
input      CLK;
input      RST_N;
input      en;
input      [INST_SIZE-1 : 0] INSTRUCTION;
input                            reg_wr_en;
input      [REG_ADDR_SIZE-1 : 0] reg_wr_addr;
input      [REG_DATA_SIZE-1 : 0] reg_wr_data;
output     [REG_DATA_SIZE-1 : 0] reg_A_val;
output     [REG_DATA_SIZE-1 : 0] reg_B_val;
output reg                       mul_en;
output reg                       shift_en;
output reg                       xor_en;
output reg                       nor_en;
output reg [INST_SIZE-1 : 0] INSTRUCTION_o;

wire mul_en_i,shift_en_i,xor_en_i,nor_en_i;

assign mul_en_i = ({INSTRUCTION[31],INSTRUCTION[17:15]} == 4'b0001) ? 1'b1 : 1'b0;
assign shift_en_i = ({INSTRUCTION[31],INSTRUCTION[17:15]} == 4'b0010) ? 1'b1 : 1'b0;
assign xor_en_i = ({INSTRUCTION[31],INSTRUCTION[17:15]} == 4'b0011) ? 1'b1 : 1'b0;
assign nor_en_i = ({INSTRUCTION[31],INSTRUCTION[17:15]} == 4'b0100) ? 1'b1 : 1'b0;

GPR_Register_File #(REG_ADDR_SIZE, REG_DATA_SIZE) reg_file (.CLK(CLK), .reg_write_en(reg_wr_en), .reg_read_en(en), .reg_write_dest_C(reg_wr_addr), .reg_write_data_C(reg_wr_data), .reg_read_addr_A(INSTRUCTION[4:0]), .reg_read_data_A(reg_A_val), .reg_read_addr_B(INSTRUCTION[9:5]), .reg_read_data_B(reg_B_val));

	always @(posedge CLK or negedge RST_N)
	begin
		if(~RST_N) 
		begin
			mul_en   <= 'd0;
			shift_en <= 'd0;
			xor_en   <= 'd0;
			nor_en   <= 'd0;
			INSTRUCTION_o <= 'd0;
		end
		else
		begin
			mul_en   <= mul_en_i;
			shift_en <= shift_en_i;
			xor_en   <= xor_en_i;
			nor_en   <= nor_en_i;
			INSTRUCTION_o <= INSTRUCTION;
		end
	end

endmodule

// Exceution unit is implemented by behaviour model
// It perfomrs follwoing operations :
//	- Performing alu operations
//	- Generate output on alu-out register
//	- Provide that to writeback unit
// It has following modules
//  	- ALU 
//      - Barrel Shifter
//      - Multipler
//      - XOR and NOR
module Execute_unit#(parameter INST_SIZE = 32, parameter REG_ADDR_SIZE = 5, parameter REG_DATA_SIZE = 32) (CLK,RST_N,en,INSTRUCTION,reg_A_val,reg_B_val,mul_en,shift_en,xor_en,nor_en,RESULT,INSTRUCTION_o);
input      CLK;
input      RST_N;
input      en;
input      [INST_SIZE-1:0]       INSTRUCTION;
input      [REG_DATA_SIZE-1 : 0] reg_A_val;
input      [REG_DATA_SIZE-1 : 0] reg_B_val;
input                            mul_en;
input                            shift_en;
input                            xor_en;
input                            nor_en;
output reg [REG_DATA_SIZE-1 : 0] RESULT;
output reg [INST_SIZE-1 : 0] INSTRUCTION_o;

reg [REG_DATA_SIZE-1 : 0] MUL_Out;
reg [REG_DATA_SIZE-1 : 0] XOR_Out;
reg [REG_DATA_SIZE-1 : 0] NOR_Out;

wire [REG_DATA_SIZE-1 : 0] SHIFTER_Out;
wire [REG_DATA_SIZE-1 : 0] ALU_Out;
wire [REG_DATA_SIZE-1 : 0] RESULT_i;

BARREL_SHIFTER_32bit#(.DATA_WIDTH(REG_DATA_SIZE),.CTRL_WIDTH(5)) shifter(.A(reg_A_val[4:0]),.B(reg_B_val),.OPR(0),.CNTR(0),.OUT(SHIFTER_Out));//opr=0,cntr=0 SLL we can  change or give control

ALU_TOP_32bit#(.DATA_WIDTH(REG_DATA_SIZE),.CTRL_WIDTH(4)) alu(.A(reg_A_val),.B(reg_B_val),.ALUC(INSTRUCTION[18:15]),.OUT(ALU_Out));

//Assignment1 is 16*9 so implemnting new one
	always @ (*)
	begin
		MUL_Out = reg_A_val*reg_B_val ;
	end

	always @ (*)
	begin
		XOR_Out = reg_A_val^reg_B_val ;
	end

	always @ (*)
	begin
		NOR_Out = ~(reg_A_val | reg_B_val) ;
	end

assign RESULT_i = (mul_en ? MUL_Out : (shift_en ? SHIFTER_Out : (xor_en ? XOR_Out : (nor_en ? NOR_Out : ALU_Out))));


	always @(posedge CLK or negedge RST_N)
	begin
		if(~RST_N)
		begin
			RESULT <= 'd0;
			INSTRUCTION_o <= 'd0;
		end
		else
		begin
		       	RESULT <= RESULT_i;
			INSTRUCTION_o <= INSTRUCTION;		
		end
	end

endmodule

// Writeback unit is implemented by behaviour modelling
// It performs the following tasks:
//	- Writing Result data into register or memory
//	- Sampling data from execute unit
module Writeback_unit #(parameter INST_SIZE = 32, parameter REG_ADDR_SIZE = 5, parameter REG_DATA_SIZE = 32) (CLK,RST_N,en,INSTRUCTION,RESULT,reg_wr_en,reg_wr_addr,reg_wr_data);
input      CLK;
input      RST_N;
input      en;
input      [INST_SIZE-1:0]       INSTRUCTION;
input      [REG_DATA_SIZE-1 : 0] RESULT;
output reg                       reg_wr_en;
output reg [REG_ADDR_SIZE-1 : 0] reg_wr_addr;
output reg [REG_DATA_SIZE-1 : 0] reg_wr_data;

	always @(posedge CLK or negedge RST_N) 
	begin
		if(~RST_N || ~en) 
		begin
			reg_wr_en   <= 0;
			reg_wr_addr <= 0;
			reg_wr_data <= 0;
		end
		else 
		begin
			reg_wr_en   <= 1;
			reg_wr_addr <= INSTRUCTION[14:10];
			reg_wr_data <= RESULT;
		end
	end

endmodule

// RISC processor design implemented using structional modelling
// which includes
//	- Instruction Fetch
//	- Decode
//	- Execute
//	- Writeback
module RISC_Processor #(parameter INST_SIZE = 32, parameter PC_SIZE = 5, parameter REG_ADDR_SIZE = 5, parameter REG_DATA_SIZE = 32, parameter FETCH_CYC = 1, parameter DECODE_CYC =1, parameter EXE_CYC =1, parameter WB_CYC =1) (CLK,RST_N,EN);
input     CLK;
input     RST_N;
input     EN;

wire [FETCH_CYC:0]  fetch_en;// 0 bit is for self and n-1 is for next operation
wire [DECODE_CYC:0] decode_en;
wire [EXE_CYC:0]    exe_en;
wire                wb_en; // As last instruction

wire [INST_SIZE-1 : 0] INSTRUCTION;
wire [INST_SIZE-1 : 0] INSTRUCTION_1st_stage;
wire [INST_SIZE-1 : 0] INSTRUCTION_2nd_stage;


wire [REG_DATA_SIZE-1 : 0] reg_A_val;
wire [REG_DATA_SIZE-1 : 0] reg_B_val;

wire                       reg_wr_en;
wire [REG_ADDR_SIZE-1 : 0] reg_wr_addr;
wire [REG_DATA_SIZE-1 : 0] reg_wr_data;

wire                       mul_en;
wire                       shift_en;
wire                       xor_en;
wire                       nor_en;
wire [REG_DATA_SIZE-1 : 0]  RESULT;

// Just to understand the operation process
wire debug_fetch_operation;
wire debug_decode_operation;
wire debug_execute_opertation;
wire debug_writeback_operation;


	assign fetch_en[0]  = EN;
	assign decode_en[0] = fetch_en[FETCH_CYC];
	assign exe_en[0]    = decode_en[DECODE_CYC];
	assign wb_en        = exe_en[EXE_CYC];

	assign debug_fetch_operation = fetch_en[0];
	assign debug_decode_operation = decode_en[0];
	assign debug_execute_operation = exe_en[0];
	assign debug_writeback_operation = wb_en;

	genvar i,j,k;
	
	// Pipelined EN delay for deocde operation
	generate
		for (i=0; i < FETCH_CYC; i=i+1 ) 
		begin :fetch_i
			Pipeline_Register fetch_pipeline(.CLK(CLK),.IN(fetch_en[i]),.OUT(fetch_en[i+1]));
		end
	endgenerate

	// Pipelined EN delay for execute operation
	generate
		for (j=0; j < DECODE_CYC; j=j+1 ) 
		begin :decode_i
			Pipeline_Register decode_pipeline(.CLK(CLK),.IN(decode_en[j]),.OUT(decode_en[j+1]));
		end
	endgenerate

	// Pipelined EN delay for write back operation
	generate
		for (k=0; k < EXE_CYC; k=k+1 ) 
		begin :exe_i
			Pipeline_Register exe_pipeline(.CLK(CLK),.IN(exe_en[k]),.OUT(exe_en[k+1]));
		end
	endgenerate

Fetch_Unit #( INST_SIZE, PC_SIZE) fetch_inst(.CLK(CLK), .RST_N(RST_N), .en(fetch_en[0]), .INSTRUCTION(INSTRUCTION));
Decode_Unit #(INST_SIZE, REG_ADDR_SIZE, REG_DATA_SIZE) decode_inst(.CLK(CLK),.RST_N(RST_N),.en(decode_en[0]),.INSTRUCTION(INSTRUCTION),.reg_wr_addr(reg_wr_addr),.reg_wr_en(reg_wr_en),.reg_wr_data(reg_wr_data),.reg_A_val(reg_A_val),.reg_B_val(reg_B_val),.mul_en(mul_en),.shift_en(shift_en),.xor_en(xor_en),.nor_en(nor_en),.INSTRUCTION_o(INSTRUCTION_1st_stage));
Execute_unit #(INST_SIZE, REG_ADDR_SIZE, REG_DATA_SIZE) exe_inst (.CLK(CLK),.RST_N(RST_N),.en(exe_en[0]),.INSTRUCTION(INSTRUCTION_1st_stage),.reg_A_val(reg_A_val),.reg_B_val(reg_B_val),.mul_en(mul_en),.shift_en(shift_en),.xor_en(xor_en),.nor_en(nor_en),.RESULT(RESULT),.INSTRUCTION_o(INSTRUCTION_2nd_stage));
Writeback_unit #(INST_SIZE, REG_ADDR_SIZE, REG_DATA_SIZE) wb_inst (.CLK(CLK),.RST_N(RST_N),.en(wb_en),.INSTRUCTION(INSTRUCTION_2nd_stage),.RESULT(RESULT),.reg_wr_en(reg_wr_en),.reg_wr_addr(reg_wr_addr),.reg_wr_data(reg_wr_data));

endmodule
