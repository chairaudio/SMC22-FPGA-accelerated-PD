module MassPositionUpdater(i_m1, i_m1_old, i_force_m1, i_mass_m1, o_m1_update);
	input  signed[26:0] i_m1, i_m1_old, i_force_m1, i_mass_m1;
	output signed[26:0] o_m1_update;
	
	//wire signed[26:0] mult;
	//signed_mult_q26 one  (.out(mult), .a( i_m2-i_m1 ), .b(i_k)); 
	assign o_m1_update = i_m1 - i_m1_old + i_m1 + i_force_m1; // i_force_m1/i_mass_m1  division needs to be implemented!

endmodule