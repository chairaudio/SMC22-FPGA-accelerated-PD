module Counter
(
	clk, 
	reset, 
	count
);
parameter width = 32;
output reg [width-1:0] count = 0;
input clk;
input reset;

always @ (posedge clk) begin
	if(reset) begin
		count = 0;
	end
	else begin
		count = count + 1;
	end
end

endmodule