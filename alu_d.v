`timescale 1ns / 1ps

// File name : alu_d.v
//
// Author : Lopelli Shiva Nagender Rao
//
// Description : This file contains 32bit ALU with the following modules
//    		 - AND Gate
//        	 - OR  Gate	
//               - XOR Gate
//		 - Multiplexors  both 4x1,2x1
//               - Carry look Ahead Adder/Subtractor
//		 - LUI

//AND Gate using behaviour model- using 32bit bitwise and operator
module ANDGate_32bit#(parameter DATA_WIDTH = 32) (A,B,OUT);

input      [DATA_WIDTH -1:0] A;
input      [DATA_WIDTH -1:0] B;
output reg [DATA_WIDTH -1:0] OUT;

	always@(*) 
	begin
		OUT = A & B;
	end

endmodule


//OR Gate using behaviour model- using 32bit bitwise or operator
module ORGate_32bit#(parameter DATA_WIDTH = 32) (A,B,OUT);

input      [DATA_WIDTH -1:0] A;
input      [DATA_WIDTH -1:0] B;
output reg [DATA_WIDTH -1:0] OUT;
	
	always@(*)
	begin
		OUT = A | B;
	end

endmodule

//XOR Gate using behaviour model- using 32bit bitwise xor operator
module XORGate_32bit#(parameter DATA_WIDTH = 32) (A,B,OUT);

input      [DATA_WIDTH -1:0] A;
input      [DATA_WIDTH -1:0] B;
output reg [DATA_WIDTH -1:0] OUT;
	
	always@(*)
	begin
		OUT = A ^ B;
	end

endmodule

//2x1 Multiplexor for 32bits using behaviour model
//           - 0-A,1-B
module MUX_2x1_32bit#(  parameter DATA_WIDTH = 32 ) (A,B,SEL,OUT);

input      [DATA_WIDTH -1:0] A;
input      [DATA_WIDTH -1:0] B;
input                        SEL;
output reg [DATA_WIDTH -1:0] OUT;

	always @(A,B,SEL)
	begin
		case(SEL)
			1'b0:     OUT = A;
			1'b1:     OUT = B;
			default:  OUT = A; //for latch free LATCH Free
		endcase
	end

endmodule

//4x1 Multiplexor for 32bits using behaviour model
//           - 00-A,01-B,10-C,11-D
module MUX_4x1_32bit#(  parameter DATA_WIDTH = 32 ) (A,B,C,D,SEL,OUT);

input      [DATA_WIDTH -1:0] A;
input      [DATA_WIDTH -1:0] B;
input      [DATA_WIDTH -1:0] C;
input      [DATA_WIDTH -1:0] D;
input      [1:0]             SEL;
output reg [DATA_WIDTH -1:0] OUT;

	always @(A,B,C,D,SEL) 
	begin
		case(SEL)
			2'b00:     OUT = A;
			2'b01:     OUT = B;
			2'b10:     OUT = C;
			2'b11:     OUT = D;
			default:   OUT = A; //for latch free LATCH Free
		endcase
	end

endmodule


// 4-sel Barrel shifter using behaviour model
// 		-OPR=1 -> ALUC[3] -> Shift Right Arthematic
//              -OPR=0,CNTR=1 -> ALUC[2]=1 -> Shift Right Logical 
//              -OPR=0,CNTR=0 -> ALUC[2]=0 -> Shift Left Logical  
module BARREL_SHIFTER_32bit#(  parameter DATA_WIDTH = 32, parameter CTRL_WIDTH = 5 ) (A,B,OPR,CNTR,OUT);

input      [CTRL_WIDTH -1:0] A;//Control input 5
input      [DATA_WIDTH -1:0] B;
input                        OPR;
input                        CNTR;
output reg [DATA_WIDTH -1:0] OUT;

	reg signed [DATA_WIDTH -1:0] Loc_B;
	
	always @( A,B,OPR,CNTR) 
	begin
		if(OPR == 1'b1)
		begin			
			Loc_B = $signed(B);
			if(CNTR == 1'b1) OUT   = Loc_B >>> A;// A=5 to shift by 5
			//else Latch as OPR=1 and CNTR=0 is not possible 
		end
		else 
		begin
			if(CNTR == 1'b1)  OUT = B >> A;
			else              OUT = B << A;
		end
	end

endmodule

// Carry look ahead adder and subtractor for 32bit 
//     - EN decides adder or subtractor 
//     - En = 1, subtraction, EN=0 Addition
module CLAADDER_SUBTRACTOR_32bit#(  parameter DATA_WIDTH = 32 ) (A,B,EN,OUT,CARRY);

input      [DATA_WIDTH -1:0] A;
input      [DATA_WIDTH -1:0] B;
input                        EN;
output reg [DATA_WIDTH -1:0] OUT;
output reg                   CARRY;

	wire [DATA_WIDTH:0] C;

	assign C[0] = 0;

	genvar i;
	
	// Carry genrator using propagate and generate
	generate
		for (i=0; i < DATA_WIDTH; i=i+1 ) 
		begin :Carry_i
			assign C[i+1] = (A[i] & B[i]) | ((A[i] ^ B[i]) & C[i]);
		end
	endgenerate


	always @(*) 
	begin
		if(EN == 1'b1)
		begin
			OUT =  (A >= B) ? (A-B) : (B-A) ;
		end
		else 
		begin
			OUT = A ^ B ^ C;
			CARRY = C[DATA_WIDTH];
		end
	end 

endmodule

// LUI for 32bits using behaviour modelling
// Operation: LUI can be done by wiring low 16 bits to high 16 bits.
module LUI_MODULE_32bit#(  parameter DATA_WIDTH = 32, parameter LOC_BIT_WIDTH = 16 ) (B,OUT);

input      [LOC_BIT_WIDTH  -1:0] B;
output reg [DATA_WIDTH -1:0]     OUT;
	
	always@(*)
	begin
		OUT[DATA_WIDTH-1 : DATA_WIDTH-LOC_BIT_WIDTH] = B; //Lowr B to Higher OUT
		OUT[DATA_WIDTH-LOC_BIT_WIDTH-1 : 0]          = 'h0; //Lower OUT is GND
	end

endmodule

// ALU top using structural modelling which includes all the above modules
// Description : The ALU top  performs the below functions
//               ALUC Operation
//               x000 ADD
//               X100 SUB
//               X010 XOR
//               X001 AND
//               X101 OR
//               X110 LUI
//               0011 SLL
//               0111 SRL
//               1111 SRA
