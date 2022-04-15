module impulseGenerator#(parameter SIZE_PWR = 8)(
	input clk, 
	input reset,
	output reg signed [26:0] out
	);
	
	reg impulse_active;
	reg  [SIZE_PWR-1:0] count= 0;
	wire o_pos_edge_reset;
	
	EdgeDetector ed(clk, reset, o_pos_edge_reset);
	
	always@(posedge clk) begin
	
		if(o_pos_edge_reset) begin 
			impulse_active <= 1;
			count <= 0;
			out <= 0;
		end
		
		count <= count +1;
		if (count == (1<<SIZE_PWR)-1) begin
			impulse_active <= 0;
			out <= 0;
		end
			
		if (impulse_active) begin
			if(count<(count>>1))begin
				out <= count*(1<<(25-SIZE_PWR));//27'h0008_FFFF;
			end
			else out <= ((count>>1)-count)*(1<<(25-SIZE_PWR));
		end
	end
endmodule