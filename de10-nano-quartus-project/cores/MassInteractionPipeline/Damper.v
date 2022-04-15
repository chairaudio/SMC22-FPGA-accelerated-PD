module Damper(i_m1, i_m1_old, i_m2, i_m2_old, i_d, o_force);
	input  signed[26:0] i_m1, i_m2, i_m1_old, i_m2_old, i_d;
	output signed[26:0] o_force;
	
	wire signed[26:0] diff2 = i_m2-i_m2_old;
	wire signed[26:0] diff1 = i_m1-i_m1_old;
	wire signed[26:0] diff3 = diff2 - diff1;

	wire signed[26:0] mult;
	signed_mult_q26 one  (.out(mult), .a( diff3 ), .b(i_d)); 
	assign o_force = -mult;

endmodule

