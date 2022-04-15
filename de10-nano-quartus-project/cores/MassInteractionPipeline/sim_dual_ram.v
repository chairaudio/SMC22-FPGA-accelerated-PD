module sim_dual_ram#(parameter SIZE = 27, parameter ADDR_WIDTH = 4 )(
	output reg [SIZE-1:0] q1, q2,
	input [SIZE-1:0] d,
	input [ADDR_WIDTH-1:0] w_addr, r_addr_q1, r_addr_q2,
	input we, clk
	);
	reg signed [SIZE-1:0] mem [2**ADDR_WIDTH-1:0];
	reg [ADDR_WIDTH-1:0] read_address_reg1;
	reg [ADDR_WIDTH-1:0] read_address_reg2;

	
	always @ (posedge clk) begin
		if (we)
			mem[w_addr] = d;
        
        q1 <= mem[read_address_reg1];
		read_address_reg1 <= r_addr_q1;
		q2 <= mem[read_address_reg2];
		read_address_reg2 <= r_addr_q2;
	end
	
	integer i;
	initial begin
		for (i=0;i<=2**ADDR_WIDTH-1;i=i+1)
			mem[i] = 0;
	end
endmodule
