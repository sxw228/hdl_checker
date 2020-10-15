module test_3(clk,rst_n,data,add);
	input clk;
	input rst_n;
	input[3:0] data;
	output[2:0] add;
	
	reg[2:0] add;

	always @ (posedge clk) begin
		if(!rst_n) begin 
		    add <= 0;
		end
		else begin 
	            casex(data)
              		4'b00xx:  add <= 1;
              		4'b01xx:  add <= 2;
              		4'b10xx:  add <= 3;
              		4'b11xx:  add <= 4;
              		default:  add <= 5;
              	    endcase
		end
	end
endmodule
