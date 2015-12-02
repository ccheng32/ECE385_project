module whiteDetect(input logic clk, input logic[15:0] incolor, output logic[15:0] outcolor, input logic reset_n);

logic[15:0] prev2, prev1, curr;
parameter [5:0] diff = 6'd32;

always_ff@( posedge clk)

begin
if(!reset_n)begin
prev2<=0;
prev1<=0;
curr<=0;
end
else begin
prev2<=prev1;
prev1<=curr;
curr<=incolor;
end
end

always_ff@( posedge clk) begin
if(!reset_n)begin
outcolor <= 0;
end

else begin
    if({curr[15:11],1'b0} - prev1[15:11] - prev2[15:11] > diff )
	outcolor[15:11] <= 5'b11111;
	else
	outcolor[15:11] <= 0;
	
    if({curr[10:5],1'b0} - prev1[10:5] - prev2[10:5] > {diff,1'b0} )
	outcolor[10:5] <= 6'b111111;
	else
	outcolor[10:5] <= 0;
	
    if({curr[4:0],1'b0} - prev1[4:0] - prev2[4:0] > diff )
	outcolor[4:0] <= 5'b11111;
	else
	outcolor[4:0] <= 0;
	end
end


endmodule