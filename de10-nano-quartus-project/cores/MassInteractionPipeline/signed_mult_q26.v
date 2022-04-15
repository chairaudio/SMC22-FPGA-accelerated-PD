//// signed mult of 0.26 format 2'comp////////////

module signed_mult_q26 (out, a, b);
	output 	signed	[26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	
	wire	signed	[26:0]	out;
	wire 	signed	[53:0]	mult_out;

	assign mult_out = a * b;
	assign out = {mult_out[53], mult_out[52:27]};
endmodule
//////////////////////////////////////////////////