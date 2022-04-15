// Simple APB Data Interface
// Author: Clemens Wegener, CHAIR.AUDIO 2020

module generic_apb_io (
		input  wire [4:0]  aps_s0_paddr,   //      aps_s0.paddr
		input  wire        aps_s0_psel,    //            .psel
		input  wire        aps_s0_penable, //            .penable
		input  wire        aps_s0_pwrite,  //            .pwrite
		input  wire [31:0] aps_s0_pwdata,  //            .pwdata
		output reg  [31:0] aps_s0_prdata,  //            .prdata
		output wire        aps_s0_pready,  //            .pready
		input  wire        clock_clk,      //       clock.clk
		input  wire        reset_n,    	  //       reset.reset_n
		output reg  [31:0] data_out,       // conduit_end.data_out
		input  wire [31:0] data_in,        // 				 .data_in
		output reg			 strobe       	  // 				 .strobe
	);

	// Register access
	always @(posedge clock_clk or negedge reset_n)
	begin
		if (~reset_n)
		begin
			data_out <= 0;
		end
		else
		begin
			if (aps_s0_pwrite & ~aps_s0_penable) // data write
			begin
				data_out <= aps_s0_pwdata;
				strobe <= 1'b1;
			end
			if (~aps_s0_pwrite & ~aps_s0_penable) // data input register
			begin
				aps_s0_prdata <= data_in;
				strobe <= 1'b0;
			end
		end
	end
	
	// APB
	assign aps_s0_pready = aps_s0_penable; // always ready (no wait states)	
	

endmodule
