module EdgeDetector(clk, i_signal, o_pos_edge);
	input clk;
	input i_signal;
	output reg o_pos_edge;
	
	reg old_sig = 0;
	
	always @ (posedge clk) begin
	
		if( i_signal > old_sig)
			o_pos_edge <= 1;
		else o_pos_edge <= 0;
		
		old_sig <= i_signal;
	end
	
endmodule