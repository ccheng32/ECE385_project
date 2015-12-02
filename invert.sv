module invert(input clk, input logic[23:0] incolor, output logic[23:0] outcolor);

//always_ff@(posedge clk) begin
assign outcolor[23:16] = 8'hff - incolor[23:16];
assign outcolor[15:8] = 8'hff - incolor[15:8];
assign outcolor[7:0] = 8'hff - incolor[7:0];
//end

endmodule