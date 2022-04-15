module fast_ram#(parameter SIZE = 27, parameter ADDR_WIDTH = 5 )(
	output [SIZE-1:0] q,
	input [SIZE-1:0] d,
	input [ADDR_WIDTH-1:0] addr,
	input we, clk
	);
	reg signed [SIZE-1:0] mem [2**ADDR_WIDTH-1:0] /* synthesis ramstyle = "no_rw_check, M10K" */;
	assign q = mem[addr];
	
	always @ (posedge clk) begin
		if (we)
			mem[addr] = d;
	end
	
	integer i;
	initial begin
		for (i=0;i<=2**ADDR_WIDTH-1;i=i+1)
			mem[i] = 0;
	end
	
	  // the "macro" to dump signals
`ifdef COCOTB_SIM
    initial begin
        $dumpfile ("fast_ram.vcd");
        $dumpvars (0, fast_ram);
        #1;
    end
`endif

endmodule
