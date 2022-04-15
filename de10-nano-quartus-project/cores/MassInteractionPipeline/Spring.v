module Spring(i_m1, i_m2, i_k, o_force);
	input  signed[26:0] i_m1, i_m2, i_k;
	output signed[26:0] o_force;
	
	wire signed[26:0] mult;
	wire signed[26:0] diff = i_m2-i_m1;
	signed_mult_q26 one  (.out(mult), .a( diff ), .b(i_k)); 
	assign o_force = -mult;

endmodule

