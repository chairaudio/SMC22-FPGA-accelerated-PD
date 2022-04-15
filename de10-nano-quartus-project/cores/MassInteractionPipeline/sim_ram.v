module sim_ram#(parameter SIZE = 27, parameter ADDR_WIDTH = 5 )(
	output reg [SIZE-1:0] q,
	input [SIZE-1:0] d,
	input [ADDR_WIDTH-1:0] addr,
	input we, clk
);
    reg signed [SIZE-1:0] mem [2**ADDR_WIDTH-1:0];
	reg [ADDR_WIDTH-1:0] read_address_reg;
	
	always @ (posedge clk)
	begin
		if (we) 
			mem[addr] <= d;
        q <= mem[read_address_reg];
		read_address_reg <= addr;
	end
		
	integer i;
	initial begin
		for (i=0;i<=2**ADDR_WIDTH-1;i=i+1)
			mem[i] = 0;
	end
	
	  // the "macro" to dump signals
`ifdef COCOTB_SIM
    initial begin
        $dumpfile ("sim_ram.vcd");
        $dumpvars (0, sim_ram);
        #1;
    end
`endif

endmodule
