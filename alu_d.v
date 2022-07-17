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