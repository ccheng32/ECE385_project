module grayscale(input logic clk, input logic[15:0] incolor, output logic[15:0] outcolor, input logic reset_n);


always_ff@( posedge clk) begin
if(!reset_n)begin
outcolor <= 0;
end

else begin
   outcolor[15:11] <= {2'b0, incolor[15:13]} + {2'b0, incolor[10:8]} + {1'b0 ,incolor[10:7]};
	outcolor[10:5] <= {  ({2'b0, incolor[15:13]} + {2'b0, incolor[10:8]} + {1'b0 ,incolor[10:7]}), 1'b0};
	outcolor[4:0] <= {2'b0, incolor[15:13]} + {2'b0, incolor[10:8]} + {1'b0 ,incolor[10:7]};
	end
end


endmodule