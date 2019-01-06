//http://www.asic-world.com/examples/verilog/serial_crc.html#Serial_CRC

`timescale 1ns/1ns
`define WIDTH 16
//**************************************************************************
module Serial_CRC_wTB();
//**************************************************************************

	//reg in_enable;
	reg in_clk;
	reg in_rst_n;
	reg [`WIDTH-9:0] in_data;
	wire [`WIDTH-1:0] out_crcOutput;

	Serial_CRC TS (.i_data(in_data), .o_crcOutput(out_crcOutput), .i_clk(in_clk), .i_rst_n(in_rst_n));

	initial
		begin
			in_clk = 1'b0 ; in_rst_n = 1'b0; in_data = 8'hF8;
			#1 in_rst_n = 1'b0;
			#2 in_rst_n = 1'b1;
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 0 
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 1
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 2
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 3
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 4
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 5
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 6
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 7
			@(negedge in_clk) in_data = 8'h81;
			@(posedge in_clk) $display("Crc Output :%h",out_crcOutput); // 8
			//@(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			 //@(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			// @(posedge in_clk) $display("Crc Output :%h",out_crcOutput);
			#1 $finish;
		end 

	always 
		#1 in_clk = ~in_clk;

endmodule // Serial_CRC_wTB


//**************************************************************************
module Serial_CRC (input [`WIDTH-9:0] i_data,input i_clk,input i_rst_n, output [`WIDTH-1:0] o_crcOutput);
//**************************************************************************

	parameter		  S0 = 3'b000;		//BIT[0]
	parameter		  S1 = 3'b001;		//BIT[1]
	parameter		  S2 = 3'b010;		//BIT[2]
	parameter		  S3 = 3'b011;		//BIT[3]
	parameter		  S4 = 3'b100;		//BIT[4]
	parameter		  S5 = 3'b101;		//BIT[5]
	parameter		  S6 = 3'b110;		//BIT[6]
	parameter		  S7 = 3'b111;		//BIT[7]
	
	reg [2:0] curr_state;
	reg [2:0] next_state;

	reg [7:0] d;
	reg [15:0] crc;

	// Sequential logic for storing current state
	always @ (posedge i_clk, negedge i_rst_n) begin
		if (~i_rst_n) begin 
			curr_state <= S0;
			crc <= {16{1'b0}}; end
		else 
			curr_state <= next_state;
	end 

	always @ (curr_state, d) begin
		case (curr_state)
			S0: next_state <= S1;
				
			S1: next_state <= S2;
						
			S2: next_state <= S3;	

			S3: next_state <= S4;

			S4: next_state <= S5;
						
			S5: next_state <= S6;

			S6: next_state <= S7;

			S7: next_state <= S0;						
		endcase // curr_state
	end 

 // need to fix the blocking versus the non-blocking case on the state variables
	// Output Logic
	always @ (posedge i_clk) begin 
		
		case(curr_state)
			S0: begin 
				d = i_data;

				if (d[0] == crc[0])
					crc = crc >> 1'b1;
				else if (d[0] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S0
			S1: begin 
				if (d[1] == crc[0])
					crc = crc >> 1'b1;
				else if (d[1] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S1
			S2: begin 
				if (d[2] == crc[0])
					crc = crc >> 1'b1;
				else if (d[2] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S2
			S3: begin 
				if (d[3] == crc[0])
					crc = crc >> 1'b1;
				else if (d[3] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S3
			S4: begin 
				if (d[4] == crc[0])
					crc = crc >> 1'b1;
				else if (d[4] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S4
			S5: begin 
				if (d[5] == crc[0])
					crc = crc >> 1'b1;
				else if (d[5] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S5
			S6: begin 
				if (d[6] == crc[0])
					crc = crc >> 1'b1;
				else if (d[6] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S6
			S7: begin 
				if (d[7] == crc[0])
					crc = crc >> 1'b1;
				else if (d[7] != crc[0]) begin
						crc = crc >> 1'b1;
						crc = crc ^ 16'h1021; end 
					end // S7
		endcase // curr_state
		end //always

assign o_crcOutput = crc;		
endmodule // Serial_CRC