module convolution( input logic clk,
					input logic[23:0] line0, line1, line2,
					output logic[23:0] convout);

		logic[23:0] m00, m01, m02, m10, m11, m12, m20, m21, m22;
					
always_ff@(posedge clk)
begin
    // first row
    m00 <= line0;
	m01 <= m00;
	m02 <= m01;
   
    // second row
	m10 <= line1;
	m11 <= m10;
	m12 <= m11;
	
	// third row
    m20 <= line2;
	m21 <= m20;
	m22 <= m21;
end


always_ff@(posedge clk)
begin
convout[23:16] <= {1'b0,m01[23:17]} + {1'b0,m11[23:17]};
convout[15:8] <= {1'b0,m01[15:9]} + {1'b0,m11[15:9]};
convout[7:0] <= {1'b0,m01[8:1]} + {1'b0,m11[8:1]};
end

endmodule