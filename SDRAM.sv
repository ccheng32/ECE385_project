/*---------------------------------------------------------------------------
  --      SDRAM.sv                                                          --
  --      Christine Chen                                                   --
  --      10/23/2013                                                       --
  --      modified by Zuofu Cheng                                          --
  --      For use with ECE 385                                             --
  --      UIUC ECE Department                                              --
  ---------------------------------------------------------------------------*/
// Top-level module that integrates the Nios II system with the rest of the hardware

module SDRAM(  	  	  input	       CLOCK_50, 
					  input  [3:0]  KEY,
					  input [17:0]  SW,
					  output [7:0]  LEDG,
					  /*output [12:0] DRAM_ADDR,
					  output [1:0]  DRAM_BA,
					  output        DRAM_CAS_N,
					  output		    DRAM_CKE,
					  output		    DRAM_CS_N,
					  inout  [31:0] DRAM_DQ,
					  output  [3:0] DRAM_DQM,
					  output		    DRAM_RAS_N,
					  output		    DRAM_WE_N,
					  output		    DRAM_CLK,*/
					  inout  wire [15:0] SRAM_DQ,       // external_interface.DQ
					  output wire [19:0] SRAM_ADDR,     //                   .ADDR
					  output wire        SRAM_LB_N,     //                   .LB_N
					  output wire        SRAM_UB_N,     //                   .UB_N
					  output wire        SRAM_CE_N,     //                   .CE_N
					  output wire        SRAM_OE_N,     //                   .OE_N
					  output wire        SRAM_WE_N,
					  
					  output [6:0]     HEX0,
					  output [6:0]     HEX1,
					  output [6:0]     HEX2,
					  output [6:0]     HEX3,
					  output [6:0]     HEX4,
					  output [6:0]     HEX5,
					  output [6:0]     HEX6,
					  output [6:0]     HEX7,
					  // VGA Interface 
                      output [7:0]  VGA_R,					//VGA Red
							        VGA_G,					//VGA Green
									VGA_B,					//VGA Blue
					  output        VGA_CLK,				//VGA Clock
							        VGA_SYNC_N,			//VGA Sync signal
									VGA_BLANK_N,			//VGA Blank signal
									VGA_VS,					//VGA vertical sync signal	
									VGA_HS,					//VGA horizontal sync signal
					  inout	[35:0]	GPIO
				  );
				  
//	CCD
wire	[9:0]	CCD_DATA;
wire			CCD_SDAT;
wire			CCD_SCLK;
wire			CCD_FLASH;
wire			CCD_FVAL;
wire			CCD_LVAL;
wire			CCD_PIXCLK;
reg				CCD_MCLK;	//	CCD Master Clock

wire	[15:0]	Read_DATA1;
wire	[15:0]	Read_DATA2;
wire			VGA_CTRL_CLK;
wire			AUD_CTRL_CLK;
wire	[9:0]	mCCD_DATA;
wire			mCCD_DVAL;
wire			mCCD_DVAL_d;
wire	[10:0]	X_Cont;
wire	[10:0]	Y_Cont;
wire	[9:0]	X_ADDR;
wire	[31:0]	Frame_Cont;
wire	[9:0]	mCCD_R;
wire	[9:0]	mCCD_G;
wire	[9:0]	mCCD_B;
wire			DLY_RST_0;
wire			DLY_RST_1;
wire			DLY_RST_2;
//wire			Write;
wire			Read;
reg		[9:0]	rCCD_DATA;
reg				rCCD_LVAL;
reg				rCCD_FVAL;
wire	[9:0]	sCCD_R;
wire	[9:0]	sCCD_G;
wire	[9:0]	sCCD_B;
wire			sCCD_DVAL;

//	For Sensor 1
assign	CCD_DATA[0]	=	GPIO[0];
assign	CCD_DATA[1]	=	GPIO[1];
assign	CCD_DATA[2]	=	GPIO[5];
assign	CCD_DATA[3]	=	GPIO[3];
assign	CCD_DATA[4]	=	GPIO[2];
assign	CCD_DATA[5]	=	GPIO[4];
assign	CCD_DATA[6]	=	GPIO[6];
assign	CCD_DATA[7]	=	GPIO[7];
assign	CCD_DATA[8]	=	GPIO[8];
assign	CCD_DATA[9]	=	GPIO[9];
assign	GPIO[11]	=	CCD_MCLK;
//assign	GPIO_1[15]	=	CCD_SDAT;
//assign	GPIO_1[14]	=	CCD_SCLK;
assign	CCD_FVAL	=	GPIO[13];
assign	CCD_LVAL	=	GPIO[12];
assign	CCD_PIXCLK	=	GPIO[10];

assign CCD_MCLK = ~VGA_CLK;

always@(posedge CCD_PIXCLK)
begin
	rCCD_DATA	<=	CCD_DATA;
	rCCD_LVAL	<=	CCD_LVAL;
	rCCD_FVAL	<=	CCD_FVAL;
end

CCD_Capture			u3	(	.oDATA(mCCD_DATA),
							.oDVAL(mCCD_DVAL),
							.oX_Cont(X_Cont),
							.oY_Cont(Y_Cont),
							.oFrame_Cont(),
							.iDATA(rCCD_DATA),
							.iFVAL(rCCD_FVAL),
							.iLVAL(rCCD_LVAL),
							.iSTART(KEY[3]),
							.iEND(!KEY[2]),
							.iCLK(CCD_PIXCLK),
							.iRST(KEY[0])	);
logic [15:0] camcolor;
RAW2RGB				u4	(	.oRed(mCCD_R),
							.oGreen(mCCD_G),
							.oBlue(mCCD_B),
							.oDVAL(mCCD_DVAL_d),
							.oColor16(),
							.iX_Cont(X_Cont),
							.iY_Cont(Y_Cont),
							.iDATA(mCCD_DATA),
							.iDVAL(mCCD_DVAL),
							.iCLK(CCD_PIXCLK),
							.iRST(KEY[0])	);
assign LEDG[0] = CCD_PIXCLK;
assign LEDG[1] = ~Read;


	
logic [5:0] tempG;
logic [4:0] tempR, tempB;
always_comb
begin
   tempR = mCCD_R[8:4];
   tempB = mCCD_B[8:4];
   tempG = mCCD_G[8:3];
   
if(mCCD_R[9])
	tempR = 5'b11111;
if(mCCD_G[9])
	tempG = 6'b111111;
if(mCCD_B[9])
	tempB = 5'b11111; 
end

always_ff@(posedge VGA_CLK)
begin
/*if(!KEY[0]) begin
	camcolor <= 0;
end*/

/*if(invertenable)
camcolor <= {5'b11111-tempR, 6'b111111-tempG, 5'b11111-tempB};
else if(edgeenable)
camcolor <= edgefilter;
else
camcolor <= {tempR, tempG, tempB};*/

case(filterchoose)
 2'b00: camcolor <= {tempR, tempG, tempB};
 2'b01: camcolor <= /*grayfilter;*/{5'b11111-tempR, 6'b111111-tempG, 5'b11111-tempB};
 2'b10: camcolor <= edgefilter;
 2'b11: camcolor <= whitefilter;
 default: camcolor <= {tempR, tempG, tempB};
endcase
end						
I2C_CCD_Config 		u7	(	//	Host Side
							.iCLK(CLOCK_50),
							.iRST_N(KEY[0]),
							.iExposure(SW[17:2]),
							//	I2C Side
							.I2C_SCLK(GPIO[14]),
							.I2C_SDAT(GPIO[15])	);


				  	logic [19:0] MMaddr;
					logic [15:0] color;
					logic Reset_h, avail;
					assign Reset_h = !KEY[0];
											 

sramcontrol sr0(
		.clk(CLOCK_50),           //                clk.clk
		.reset(!KEY[0]),         //              reset.reset
		.SRAM_DQ(SRAM_DQ),       // external_interface.DQ
		.SRAM_ADDR(SRAM_ADDR),     //                   .ADDR
		.SRAM_LB_N(SRAM_LB_N),     //                   .LB_N
		.SRAM_UB_N(SRAM_UB_N),     //                   .UB_N
		.SRAM_CE_N(SRAM_CE_N),     //                   .CE_N
		.SRAM_OE_N(SRAM_OE_N),     //                   .OE_N
		.SRAM_WE_N(SRAM_WE_N),     //                   .WE_N
		.address(sramaddr),       //  avalon_sram_slave.address
		.byteenable(2'b11),    //                   .byteenable
		.read(Read),          //                   .read
		.write(~Read),         //                   .write
		.writedata(camcolor),     //                   .writedata
		.readdata(color),      //                   .readdata
		.readdatavalid(avail)  //                   .readdatavalid
	);

logic [9:0] drawxsig, drawysig;
logic [19:0] sramaddr;
//logic Read;

always_comb
begin
	if(Read)
		//sramaddr = drawxsig+{drawysig,10'b0000000000};
		sramaddr = {drawysig,drawxsig};
	else
		//sramaddr = X_Cont[10:1] + {Y_Cont[10:1],10'b0000000000};
		sramaddr = {Y_Cont[10:1],X_Cont[10:1]};
end

/*always_ff @(CLOCK_50)
begin
	if(!KEY[0])
	Read<=0;
	else
	Read <= ~Read;
end*/



always_comb begin
Read = 0;
if(drawxsig>80 && drawxsig<560 && drawysig<480 && drawxsig[0])
Read = 1;
end


/*
always_comb begin
Write = 0;
if(Y_Cont[1:0] == 0 || X_Cont[0] == 0)
Write = 1;
end

always_comb begin
Read = !Write;
if(drawxsig<640 && drawysig<480 )
Read = 1;
end
*/


/*color_mapper color_instance(.DrawX(drawxsig), .DrawY(drawysig), .color(color),
                       .Red(VGA_R), .Green(VGA_G), .Blue(VGA_B), .SW(SW[8:0]) );*/
					   

always_comb
begin
	VGA_R = 8'hz;
	VGA_G = 8'hz;
	VGA_B = 8'hz;
if(avail && Read && drawxsig > 80 && drawxsig < 560 && drawxsig[0])begin
 //if(!SW[0])begin
 VGA_R = tempRGB[23:16];//{color[15:11], color[14:12]};
 VGA_G = tempRGB[15:8];//{color[10:8],color[7:5], 2'b00};
 VGA_B = tempRGB[7:0];//{color[4:0], color[3:1]};
 //end
 /*else begin
 VGA_R = edgefilter[23:16];
 VGA_G = edgefilter[15:8];
 VGA_B = edgefilter[7:0];
 end*/
 end
end
logic [23:0] invertfilter, tempRGB;
logic [15:0] edgefilter, whitefilter, grayfilter;
logic [1:0] filterchoose;

always_ff @ (posedge VGA_CLK) begin
	if(!KEY[0]) begin
	tempRGB <= 0;
	end
	else begin
	tempRGB[23:16] <={color[15:11], 3'b0};
	tempRGB[15:8] <={color[10:8],color[7:5], 2'b00};
	tempRGB[7:0] <={color[4:0], 3'b0}; 
	end
end

always_ff @ (posedge VGA_CLK) begin
	filterchoose[1:0] <= SW[1:0];
end
grayscale (VGA_CLK, {tempR,tempG,tempB}, grayfilter, KEY[0]);
edgeDetect (VGA_CLK, {tempR,tempG,tempB}, edgefilter, KEY[0]);
whiteDetect (VGA_CLK, {tempR,tempG,tempB}, whitefilter, KEY[0]);
//invert inv1(VGA_CLK, {{color[15:11], color[14:12]}, {color[10:8],color[7:5], 2'b00}, {color[4:0], color[3:1]}}, invertfilter);

vga_controller vgasync_instance(      .Clk(CLOCK_50),       // 50 MHz clock
                                      .Reset(Reset_h),     // reset signal
                                      .hs(VGA_HS),        // Horizontal sync pulse.  Active low
								      .vs(VGA_VS),        // Vertical sync pulse.  Active low
									  .pixel_clk(VGA_CLK), // 25 MHz pixel clock output
									  .blank(VGA_BLANK_N),     // Blanking interval indicator.  Active low.
									  .sync(VGA_SYNC_N),      // Composite Sync signal.  Active low.  We don't use it in this lab,
												 //   but the video DAC on the DE2 board requires an input for it.
						             .DrawX(drawxsig),     // horizontal coordinate
								     .DrawY(drawysig),
									 .sdram_done(sdram_done)
									 );
											 
				//Instantiate additional FPGA fabric modules as needed	
				
					HexDriver        Hex7 (.In0(camcolor[3:0]),
											     .Out0(HEX6) );
					HexDriver        Hex6 (.In0(camcolor[7:4]),
											     .Out0(HEX7) );
					HexDriver        Hex5 (.In0(camcolor[11:8]),
											     .Out0(HEX4) );
					HexDriver        Hex4 (.In0(camcolor[15:12]),
											     .Out0(HEX5) );
					HexDriver        Hex3 (.In0(mCCD_DATA[7:4]),
											     .Out0(HEX2) );
					HexDriver        Hex2 (.In0(mCCD_DATA[3:0]),
											     .Out0(HEX3) );
					HexDriver        Hex1 (.In0(CCD_DATA[7:4]),
											     .Out0(HEX0) );
					HexDriver        Hex0 (.In0(CCD_DATA[3:0]),
											     .Out0(HEX1) );		
endmodule