`timescale 1ns / 1ns
`include "usb2_0_softphy_name.v"
`include "static_macro_define.v"

module `getname(usb2_0_softphy,`module_name)
        #(
             parameter usb_rst_det = 1'b1
        )
        (
            input  wire       fclk_deg0       ,
            input  wire       clk             ,  // 60mhz
            input  wire       pll_locked      ,  // 60mhz
            input  wire       rstn            ,
            input  wire       phy_tx_mode     ,  // HIGH level for differential io mode (else single-ended)
            output wire       usb_rst         ,
            // Transciever Interface
            input  wire       rxd             ,
            output wire       txdx            ,
            output wire       txoe            ,
            input  wire       rxdp            ,
            input  wire       rxdn            ,
            output wire       pullup_en       ,
            inout  wire       term_Dp         ,
            inout  wire       term_Dn         ,
        /////////////////////////////////////////////////

            // UTMI Interface
            input  wire [7:0] DataOut_i       ,
            input  wire       TxValid_i       ,
            output wire       TxReady_o       ,
            output wire [7:0] DataIn_o        ,
            output wire       RxValid_o       ,
            output wire       RxActive_o      ,
            output wire       RxError_o       ,
            output wire [1:0] LineState_o     ,
            input  wire [1:0] OpMode_i        ,
            input  wire       TermSelect_i    ,
            input  wire [1:0] XcvrSelect_i
            ,output wire w_eop_o
            ,output wire [3:0]  q_offset_o 
            ,output wire sclk
            ,output wire [15 : 0] DRU_dbg_signals_o
        );

  `getname(usb_phy_hs,`module_name)
      #( .usb_rst_det(1) )
      u_usb_phy_hs
   (
      .fclk_deg0        (fclk_deg0   ),
      .clk              (clk         ),// i
      .pll_locked       (pll_locked  ),
      .rstn             (rstn        ),// i
      .phy_tx_mode      (phy_tx_mode ),// i
      .usb_rst          (usb_rst     ),// o
      .rxd              (rxd         ),
      .txdx             (txdx        ),
      .txoe             (txoe        ),
      .rxdp             (rxdp        ),// i
      .rxdn             (rxdn        ),// i
      /////////////////////////////////////////////////
      .pullup_en        (pullup_en   ),
      .term_Dp          (term_Dp     ),
      .term_Dn          (term_Dn     ),
      /////////////////////////////////////////////////
      .DataOut_i        (DataOut_i   ),// i (7 downto 0);
      .TxValid_i        (TxValid_i   ),// i
      .TxReady_o        (TxReady_o   ),// o
      .DataIn_o         (DataIn_o    ),// o (7 downto 0);
      .RxValid_o        (RxValid_o   ),// o
      .RxActive_o       (RxActive_o  ),// o
      .RxError_o        (RxError_o   ),// o
      .LineState_o      (LineState_o ),// o (1 downto 0)
      .OpMode_i         (OpMode_i    ),
      .TermSelect_i     (TermSelect_i),
      .XcvrSelect_i     (XcvrSelect_i)
      ,.w_eop_o         (w_eop_o)
      ,.q_offset_o      (q_offset_o)
      ,.sclk            (sclk)
      ,.DRU_dbg_signals_o(DRU_dbg_signals_o)
  );

endmodule
//--------------------------------------------------------------------
//usb_phy_hs
module `getname(usb_phy_hs,`module_name)
    #(
     parameter usb_rst_det = 1'b1
    )
    (
    input  wire       fclk_deg0       ,
    input  wire       clk             ,  // 60mhz
    input  wire       pll_locked      ,
    input  wire       rstn            ,
    input  wire       phy_tx_mode     ,  // HIGH level for differential io mode (else single-ended)
    output wire       usb_rst         ,

    // Transciever Interface
    input  wire       rxd             ,
    output wire       txdx            ,
    output wire       txoe            ,
    input  wire       rxdp            ,
    input  wire       rxdn            ,
/////////////////////////////////////////////////
    output wire       pullup_en       ,

    inout  wire       term_Dp         ,
    inout  wire       term_Dn         ,
/////////////////////////////////////////////////

    // UTMI Interface
    input  wire [7:0] DataOut_i       ,
    input  wire       TxValid_i       ,
    output wire       TxReady_o       ,
    output wire [7:0] DataIn_o        ,
    output wire       RxValid_o       ,
    output wire       RxActive_o      ,
    output wire       RxError_o       ,
    output wire [1:0] LineState_o     ,
    input  wire [1:0] OpMode_i        ,
    input  wire       TermSelect_i    ,
    input  wire [1:0] XcvrSelect_i
    ,output wire w_eop_o
    ,output wire [3:0]  q_offset_o 
    ,output sclk
    ,output wire [15 : 0] DRU_dbg_signals_o
  );

    reg        q_se0;
    reg [3:0]  state;
    reg        q_term;
    wire       clk60mhz;
    wire [1:0] LineState      ;
    reg  [7:0] rst_cnt        ;
    wire       txoe_out       ;
    reg        usb_rst_out    ;


assign clk60mhz = clk;
assign pullup_en = (!rstn) ? 1'b0 : TermSelect_i ? 1'b1 : 1'bZ;
assign term_Dp = TermSelect_i ? 1'bz : 1'b0;
assign term_Dn = TermSelect_i ? 1'bz : 1'b0;
//======================================================================================--
// Misc Logic                                                                         --
//======================================================================================--

assign usb_rst      = usb_rst_det ? usb_rst_out : 1'b0;
assign LineState_o  = LineState;
//////////////////////////////////////////////////////////////////////////
// reset sync module, it will make all the serial primitive sync correctly.
//oser_rst u_oser_rst(
`getname(oser_rst ,`module_name) u_oser_rst(
  .clk_in     (clk        ), // or any other speed comparble with fabric, DO NOT use HCLK as fabric cannot work at so high speed.
  .rst_n      (rstn       ),
  .pll_lock   (pll_locked ), // trigged by PLL Lock
  .reset_stop (reset_stop ), // for DHCEN CE
  .reset_calib(reset_calib), // for IDES8 and CLKDIV reset
  .set_calib  (           ),
  .ready      (ready      )
);

//==================== clocks ================================
// fclk is high speed clock from PLL output, use HCLK resource
// pclk is divived from fclk, use GCLK resource
// DHCEN CE will be drived by oser_rst, which is reset sync for all IDES and CLKDIV
wire fclk;
wire sclk;
DHCEN dhcen_inst (
  .CLKIN (fclk_deg0 ),
  .CE    (reset_stop),
  .CLKOUT(fclk      )
);

CLKDIV clkdiv_inst (
  .HCLKIN(fclk        ),
  .RESETN(~reset_calib),
  .CALIB (1'b0        ),
  .CLKOUT(sclk        )
);
defparam clkdiv_inst.DIV_MODE="4";

//======================================================================================--
// RX Phy and DPLL                                                                    --
//======================================================================================--

  `getname(usb_rx_phy_hs,`module_name) i_rx_phy (
    .fclk_deg0  (fclk       ),//480mhz
    .clk        (clk60mhz   ),//60mhz
    .sclk       (sclk       ),//120mhz
    .rstn       (rstn       ),
    .reset_calib(reset_calib),
    // Transciever Interface
    .rxd        (rxd        ),
    .rxdp       (rxdp       ),
    .rxdn       (rxdn       ),
    // UTMI Interface
    .DataIn_o   (DataIn_o   ),
    .RxValid_o  (RxValid_o  ),
    .RxActive_o (RxActive_o ),
    .RxError_o  (RxError_o  ),
    .LineState  (LineState  ),
    .TxValid_i  (TxValid_i  ) //need tx valid to gate tx data coming back into the deserializer.
    ,.w_eop_o (w_eop_o)
    ,.q_offset_o (q_offset_o)
    ,.DRU_dbg_signals_o(DRU_dbg_signals_o)
  );

//======================================================================================--
// TX Phy                                                                             --
//======================================================================================--
  `getname(usb_tx_phy_hs,`module_name) i_tx_phy (
    .fclk       (fclk       ),
    .clk        (clk        ),
    .sclk       (sclk       ),
    .rstn       (rstn       ),
    .reset_calib(reset_calib),
    .phy_mode   (phy_tx_mode),
    // Transciever Interface
    .txdx       (txdx       ),
    .txoe       (txoe       ),
    // UTMI Interface
    .DataOut_i  (DataOut_i  ),
    .TxValid_i  (TxValid_i  ),
    .TxReady_o  (TxReady_o  ),
    .OpMode_i   (OpMode_i   )
  );

//======================================================================================--
// Generate an USB Reset if we see SE0 for at least 2.5uS                             --
//======================================================================================--

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        rst_cnt <= 0;
    end
    else begin
        if (LineState != 2'b00) begin
            rst_cnt <= 0;
        end
        //else if (!usb_rst_out && fs_ce)
        else if (!usb_rst_out) begin
            rst_cnt <= rst_cnt + 1'b1;
        end
    end
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        usb_rst_out  <= 1'b0;
    end
    else begin
        if (rst_cnt == 8'd150) begin
            usb_rst_out  <= 1'b1;
        end
        else begin
            usb_rst_out  <= 1'b0;
        end
    end
end


endmodule

module `getname(usb_rx_phy_hs,`module_name)
(
    input wire          fclk_deg0       ,
    input wire          clk             ,
    input wire          sclk            ,
    input wire          rstn            ,
    input wire          reset_calib     ,
    //-- Transciever Interface
    input  wire         rxd             ,
    input  wire         rxdp            ,
    input  wire         rxdn            ,
    //-- UTMI Interface
    output wire [7:0]   DataIn_o        ,
    output wire         RxValid_o       ,
    output wire         RxActive_o      ,
    output wire         RxError_o       ,
    output wire [1:0]   LineState       ,
    input wire          TxValid_i
    ,output w_eop_o
    ,output [3:0]  q_offset_o 
    ,output wire [15 : 0] DRU_dbg_signals_o
);

parameter SE0_DET_PERIOD = 3600; //@60MHz clock, measure 60uS
parameter IDLE_STATE = 0;
parameter SOP_STATE0 = 1;
parameter SOP_STATE1 = 2;
parameter SOP_STATE2 = 3;
parameter SOP_STATE3 = 4;
parameter DATA_STATE = 5;
parameter EOP_STATE1 = 6;
parameter EOP_STATE = 7;

//always @(*) begin
//
//    case(state) 
//        IDLE_STATE: q_offset_o
//
//
//    endcase
//
//end


reg  [7:0]  q_nrzi_data, q_nrzi_data2, q_smark;
wire [7:0]  w_nrzi_data;
wire [15:0] wq_nrzi_data;
wire [10:0] w_smark;
reg         q_prev_bit;
wire [7:0]  w_se0_x8, w_rxd, w_rxdp, w_rxdn;
wire        w_rxd_val;
wire [7:0]  w_align_data;
wire [7:0]  w_align_data_pre;
reg  [2:0]  q_shift;
reg  [7:0]  q_rxd;
wire        w_sop, w_eop;
reg  [2:0]  state;
reg  [3:0]  q_offset;

wire [18:0] w_smark_19;
wire [18:0] w_data_19, w_ofs_mask;
wire [7:0]  w_data_raw;
reg  [2:0]  q_data_dly;
wire [7:0]  w_rxdp_x8_sclk, w_rxdn_x8_sclk, w_cdrdata_120, w_cdrmask_120;
wire        w_sync;
reg  [7:0]  q_rxd_0, q_rxd_1, q_rxd_2;
reg         q_rxd_val_0;
reg         q_rxd_val_1;
wire [1:0]  w_linestate;
reg  [1:0]  q_linestate, q_linestate_1, q_linestate_2;
reg  [15:0] se0_det_counter;
wire        se0_detected;
assign      se0_detected = (se0_det_counter >= SE0_DET_PERIOD);

assign q_offset_o[0] = RxValid_o;
assign q_offset_o[1] = w_linestate[1];
assign q_offset_o[2] = w_linestate[0];
assign w_eop_o = w_sync;

always @ (posedge clk or negedge rstn) begin
    if (!rstn) begin
        se0_det_counter <= 16'h0;
    end
    else begin
        if ((|w_rxd)&&w_rxd_val) begin
            se0_det_counter <= 16'h0;
        end
        else begin
            //if ((!se0_detected) && (!(|w_rxd)))
            if ((!se0_detected) && ((!(|w_rxd))||(!w_rxd_val)))
                se0_det_counter <= se0_det_counter + 16'h1;
            else
                se0_det_counter <= se0_det_counter;
        end
    end
end

  ////deserializer////
wire serdes_rx_en;
wire [15:0] DRU_dbg_signals_serdes;
 `getname(cdr_serdes_x8,`module_name) u_cdr_serdes_x8_dx
  (
    .rstn        (rstn       ),
    .fclk_deg0   (fclk_deg0  ),//480mhz
    .clk60mhz    (clk        ),
    .rxd         (rxd        ),
    .rxdp        (rxdp       ),
    .rxdn        (rxdn       ),
    .sclk        (sclk       ),//120mhz
    .reset_calib (reset_calib),
    .eop         (w_eop      ),
    //data busses
    .data_out    (w_rxd      ),
    .dval        (w_rxd_val  ),
    .data_err    (RxError_o  ),//good bit when active high for each of the 4 output bits
    .LineState   (w_linestate),
    .TxValid_i   (TxValid_i  )
    ,.serdes_rx_en       (serdes_rx_en)
    ,.DRU_dbg_signals_o  (DRU_dbg_signals_o
   // DRU_dbg_signals_serdes
    )
  );

/////////////////////

always @(posedge clk or negedge rstn)
    if(!rstn) begin
         q_rxd_0 <= 0;
         q_rxd_1 <= 0;
         q_rxd_2 <= 0;
         q_shift <= 0;
         q_rxd_val_0 <= 1'b0;
         q_rxd_val_1 <= 1'b0;
    end
    else begin
         q_rxd_val_0 <= w_rxd_val;
         q_rxd_val_1 <= q_rxd_val_0;
        if (w_rxd_val) begin
            q_rxd_0 <= w_rxd;
            q_rxd_1 <= q_rxd_0;
            q_rxd_2 <= q_rxd_1;
            q_shift <= (state==EOP_STATE) ? 3'h0 :
                       ((state==IDLE_STATE) & w_sync) ? (({w_rxd[0:0],q_rxd_0[7:0],q_rxd_1[7:5]} == 12'b001010101010)||({w_rxd[0:0],q_rxd_0[7:0],q_rxd_1[7:5]} == 12'b110101010101)) ? 3'h1 :
                                                        (({w_rxd[1:0],q_rxd_0[7:0],q_rxd_1[7:6]} == 12'b001010101010)||({w_rxd[1:0],q_rxd_0[7:0],q_rxd_1[7:6]} == 12'b110101010101)) ? 3'h2 :
                                                        (({w_rxd[2:0],q_rxd_0[7:0],q_rxd_1[7:7]} == 12'b001010101010)||({w_rxd[2:0],q_rxd_0[7:0],q_rxd_1[7:7]} == 12'b110101010101)) ? 3'h3 :
                                                        (({w_rxd[3:0],q_rxd_0[7:0]}              == 12'b001010101010)||({w_rxd[3:0],q_rxd_0[7:0]}              == 12'b110101010101)) ? 3'h4 :
                                                        (({w_rxd[4:0],q_rxd_0[7:1]}              == 12'b001010101010)||({w_rxd[4:0],q_rxd_0[7:1]}              == 12'b110101010101)) ? 3'h5 :
                                                        (({w_rxd[5:0],q_rxd_0[7:2]}              == 12'b001010101010)||({w_rxd[5:0],q_rxd_0[7:2]}              == 12'b110101010101)) ? 3'h6 :
                                                        (({w_rxd[6:0],q_rxd_0[7:3]}              == 12'b001010101010)||({w_rxd[6:0],q_rxd_0[7:3]}              == 12'b110101010101)) ? 3'h7 : 3'h0 : se0_detected ? 3'h0 : q_shift;
        end
                 //((state==IDLE_STATE) & w_sync) ? (({w_rxd[0:0],q_rxd_0[7:5]} == 4'b0010)||({w_rxd[0:0],q_rxd_0[7:5]} == 4'b1101)) ? 3'h1 :
                 //                                 (({w_rxd[1:0],q_rxd_0[7:6]} == 4'b0010)||({w_rxd[1:0],q_rxd_0[7:6]} == 4'b1101)) ? 3'h2 :
                 //                                 (({w_rxd[2:0],q_rxd_0[7:7]} == 4'b0010)||({w_rxd[2:0],q_rxd_0[7:7]} == 4'b1101)) ? 3'h3 :
                 //                                 (({w_rxd[3:0]} == 4'b0010)||({w_rxd[3:0]} == 4'b1101)) ? 3'h4 :
                 //                                 (({w_rxd[4:1]} == 4'b0010)||({w_rxd[4:1]} == 4'b1101)) ? 3'h5 :
                 //                                 (({w_rxd[5:2]} == 4'b0010)||({w_rxd[5:2]} == 4'b1101)) ? 3'h6 :
                 //                                 (({w_rxd[6:3]} == 4'b0010)||({w_rxd[6:3]} == 4'b1101)) ? 3'h7 : 3'h0 : se0_detected ? 3'h0 : q_shift;  //8'h2A perfect alignment
                 //((state==IDLE_STATE) & w_sync) ? (({w_rxd[0:0],q_rxd_0[7:3]} == 6'b001010)||({w_rxd[0:0],q_rxd_0[7:3]} == 6'b110101)) ? 3'h1 :
                 //                                 (({w_rxd[1:0],q_rxd_0[7:4]} == 6'b001010)||({w_rxd[1:0],q_rxd_0[7:4]} == 6'b110101)) ? 3'h2 :
                 //                                 (({w_rxd[2:0],q_rxd_0[7:5]} == 6'b001010)||({w_rxd[2:0],q_rxd_0[7:5]} == 6'b110101)) ? 3'h3 :
                 //                                 (({w_rxd[3:0],q_rxd_0[7:6]} == 6'b001010)||({w_rxd[3:0],q_rxd_0[7:6]} == 6'b110101)) ? 3'h4 :
                 //                                 (({w_rxd[4:0],q_rxd_0[7:7]} == 6'b001010)||({w_rxd[4:0],q_rxd_0[7:7]} == 6'b110101)) ? 3'h5 :
                 //                                 (({w_rxd[5:0]} == 6'b001010)||({w_rxd[5:0]} == 6'b110101)) ? 3'h6 :
                 //                                 (({w_rxd[6:1]} == 6'b001010)||({w_rxd[6:1]} == 6'b110101)) ? 3'h7 : 3'h0 : se0_detected ? 3'h0 : q_shift;  //8'h2A perfect alignment
                 //((state==IDLE_STATE) & w_sync) ? (({w_rxd[0:0],q_rxd_0[7:1]} == 8'b00101010)||({w_rxd[0:0],q_rxd_0[7:1]} == 8'b11010101)) ? 3'h1 :
                 //                                 (({w_rxd[1:0],q_rxd_0[7:2]} == 8'b00101010)||({w_rxd[1:0],q_rxd_0[7:2]} == 8'b11010101)) ? 3'h2 :
                 //                                 (({w_rxd[2:0],q_rxd_0[7:3]} == 8'b00101010)||({w_rxd[2:0],q_rxd_0[7:3]} == 8'b11010101)) ? 3'h3 :
                 //                                 (({w_rxd[3:0],q_rxd_0[7:4]} == 8'b00101010)||({w_rxd[3:0],q_rxd_0[7:4]} == 8'b11010101)) ? 3'h4 :
                 //                                 (({w_rxd[4:0],q_rxd_0[7:5]} == 8'b00101010)||({w_rxd[4:0],q_rxd_0[7:5]} == 8'b11010101)) ? 3'h5 :
                 //                                 (({w_rxd[5:0],q_rxd_0[7:6]} == 8'b00101010)||({w_rxd[5:0],q_rxd_0[7:6]} == 8'b11010101)) ? 3'h6 :
                 //                                 (({w_rxd[6:0],q_rxd_0[7:7]} == 8'b00101010)||({w_rxd[6:0],q_rxd_0[7:7]} == 8'b11010101)) ? 3'h7 : 3'h0 : se0_detected ? 3'h0 : q_shift;
    end
assign w_sync = (state==IDLE_STATE) ?  (({w_rxd[0:0],q_rxd_0[7:0],q_rxd_1[7:5]} == 12'b001010101010)||({w_rxd[0:0],q_rxd_0[7:0],q_rxd_1[7:5]} == 12'b110101010101)) ? 1'h1 :
                                       (({w_rxd[1:0],q_rxd_0[7:0],q_rxd_1[7:6]} == 12'b001010101010)||({w_rxd[1:0],q_rxd_0[7:0],q_rxd_1[7:6]} == 12'b110101010101)) ? 1'h1 :
                                       (({w_rxd[2:0],q_rxd_0[7:0],q_rxd_1[7:7]} == 12'b001010101010)||({w_rxd[2:0],q_rxd_0[7:0],q_rxd_1[7:7]} == 12'b110101010101)) ? 1'h1 :
                                       (({w_rxd[3:0],q_rxd_0[7:0]}              == 12'b001010101010)||({w_rxd[3:0],q_rxd_0[7:0]}              == 12'b110101010101)) ? 1'h1 :
                                       (({w_rxd[4:0],q_rxd_0[7:1]}              == 12'b001010101010)||({w_rxd[4:0],q_rxd_0[7:1]}              == 12'b110101010101)) ? 1'h1 :
                                       (({w_rxd[5:0],q_rxd_0[7:2]}              == 12'b001010101010)||({w_rxd[5:0],q_rxd_0[7:2]}              == 12'b110101010101)) ? 1'h1 :
                                       (({w_rxd[6:0],q_rxd_0[7:3]}              == 12'b001010101010)||({w_rxd[6:0],q_rxd_0[7:3]}              == 12'b110101010101)) ? 1'h1 :
                                       (({w_rxd[7:0],q_rxd_0[7:4]}              == 12'b001010101010)||({w_rxd[7:0],q_rxd_0[7:4]}              == 12'b110101010101)) ? 1'h1 : 1'b0 : 1'b0;  //8'h2A perfect alignment
//assign w_sync = (state==IDLE_STATE) ?  (({w_rxd[0:0],q_rxd_0[7:1]} == 8'b00101010)||({w_rxd[0:0],q_rxd_0[7:1]} == 8'b11010101)) ? 1'h1 :
//                                       (({w_rxd[1:0],q_rxd_0[7:2]} == 8'b00101010)||({w_rxd[1:0],q_rxd_0[7:2]} == 8'b11010101)) ? 1'h1 :
//                                       (({w_rxd[2:0],q_rxd_0[7:3]} == 8'b00101010)||({w_rxd[2:0],q_rxd_0[7:3]} == 8'b11010101)) ? 1'h1 :
//                                       (({w_rxd[3:0],q_rxd_0[7:4]} == 8'b00101010)||({w_rxd[3:0],q_rxd_0[7:4]} == 8'b11010101)) ? 1'h1 :
//                                       (({w_rxd[4:0],q_rxd_0[7:5]} == 8'b00101010)||({w_rxd[4:0],q_rxd_0[7:5]} == 8'b11010101)) ? 1'h1 :
//                                       (({w_rxd[5:0],q_rxd_0[7:6]} == 8'b00101010)||({w_rxd[5:0],q_rxd_0[7:6]} == 8'b11010101)) ? 1'h1 :
//                                       (({w_rxd[6:0],q_rxd_0[7:7]} == 8'b00101010)||({w_rxd[6:0],q_rxd_0[7:7]} == 8'b11010101)) ? 1'h1 :
//                                       (({w_rxd[7:0]} == 8'b00101010)             ||({w_rxd[7:0]} == 8'b11010101)             ) ? 1'h1 : 1'b0 : 1'b0;  //8'h2A perfect alignment
//assign w_sync = ((state==IDLE_STATE)&&((q_rxd_1!=8'h00)&&(q_rxd_1!=8'hFF))) ?  (({w_rxd[0:0],q_rxd_0[7:3]} == 6'b001010)||({w_rxd[0:0],q_rxd_0[7:3]} == 6'b110101)) ? 1'h1 :
//                                       (({w_rxd[1:0],q_rxd_0[7:4]} == 6'b001010)||({w_rxd[1:0],q_rxd_0[7:4]} == 6'b110101)) ? 1'h1 :
//                                       (({w_rxd[2:0],q_rxd_0[7:5]} == 6'b001010)||({w_rxd[2:0],q_rxd_0[7:5]} == 6'b110101)) ? 1'h1 :
//                                       (({w_rxd[3:0],q_rxd_0[7:6]} == 6'b001010)||({w_rxd[3:0],q_rxd_0[7:6]} == 6'b110101)) ? 1'h1 :
//                                       (({w_rxd[4:0],q_rxd_0[7:7]} == 6'b001010)||({w_rxd[4:0],q_rxd_0[7:7]} == 6'b110101)) ? 1'h1 :
//                                       (({w_rxd[5:0]} == 6'b001010)||({w_rxd[5:0]} == 6'b110101)) ? 1'h1 :
//                                       (({w_rxd[6:1]} == 6'b001010)||({w_rxd[6:1]} == 6'b110101)) ? 1'h1 :
//                                       (({w_rxd[7:2]} == 6'b001010)||({w_rxd[7:2]} == 6'b110101)) ? 1'h1 : 1'b0 : 1'b0;  //8'h2A perfect alignment
//assign w_sync = ((state==IDLE_STATE)&&((q_rxd_0!=8'h00)&&(q_rxd_0!=8'hFF))) ?  (({w_rxd[0:0],q_rxd_0[7:5]} == 4'b0010)||({w_rxd[0:0],q_rxd_0[7:5]} == 4'b1101)) ? 1'h1 :
//                                       (({w_rxd[1:0],q_rxd_0[7:6]} == 4'b0010)||({w_rxd[1:0],q_rxd_0[7:6]} == 4'b1101)) ? 1'h1 :
//                                       (({w_rxd[2:0],q_rxd_0[7:7]} == 4'b0010)||({w_rxd[2:0],q_rxd_0[7:7]} == 4'b1101)) ? 1'h1 :
//                                       (({w_rxd[3:0]} == 4'b0010)||({w_rxd[3:0]} == 4'b1101)) ? 1'h1 :
//                                       (({w_rxd[4:1]} == 4'b0010)||({w_rxd[4:1]} == 4'b1101)) ? 1'h1 :
//                                       (({w_rxd[5:2]} == 4'b0010)||({w_rxd[5:2]} == 4'b1101)) ? 1'h1 :
//                                       (({w_rxd[6:3]} == 4'b0010)||({w_rxd[6:3]} == 4'b1101)) ? 1'h1 :
//                                       (({w_rxd[7:4]} == 4'b0010)||({w_rxd[7:4]} == 4'b1101)) ? 1'h1 : 1'b0 : 1'b0;  //8'h2A perfect alignment
//
assign wq_nrzi_data = {w_nrzi_data, q_nrzi_data};

reg [3:0] eop_enable_counter = 4'b0000;

assign w_eop =  //(eop_enable_counter >= 11) ? 
               (((q_offset == 7)&(w_data_19[14:8] == 7'b1111_111)) ||
                ((q_offset == 6)&(w_data_19[13:7] == 7'b1111_111)) ||
                ((q_offset == 5)&(w_data_19[12:6] == 7'b1111_111)) ||
                ((q_offset == 4)&(w_data_19[11:5] == 7'b1111_111)) ||
                ((q_offset == 3)&(w_data_19[10:4] == 7'b1111_111)) ||
                ((q_offset == 2)&(w_data_19[09:3] == 7'b1111_111)) ||
                ((q_offset == 1)&(w_data_19[08:2] == 7'b1111_111)) ||
                ((q_offset == 0)&(w_data_19[07:1] == 7'b1111_111))) 
                //: 1'b0
                ;

//always @(posedge clk or negedge rstn) begin
//    if (!rstn) begin
//       eop_enable_counter <= 4'b0; 
//    end else begin
//        if(serdes_rx_en) begin
//            if(eop_enable_counter >= 11) begin
//                eop_enable_counter <= eop_enable_counter;
//            end else begin
//                eop_enable_counter <= eop_enable_counter + 1'b1;
//            end
//        end else begin
//            eop_enable_counter <= 4'b0;
//        end 
//    end
//
//end







//reg byte_division_counter = 1'b0; 
//reg [3:0] byte_division_reg = 4'b0000;
//reg [7:0] q_nrzi_data_FastBuf0 = 8'b0;
//always @(posedge sclk) begin
//    if (!rstn) begin
//        byte_division_counter <= 1'b0; 
//        q_nrzi_data_FastBuf0 <= 8'b0;
//    end else begin
//        if (byte_division_counter == 1'b1) begin
//            byte_division_reg[3:0] <= q_nrzi_data_FastBuf0[3:0];   
//        end else begin
//            byte_division_reg[3:0] <= q_nrzi_data_FastBuf0[7:4];   
//        end
//        byte_division_counter <=byte_division_counter + 1'b1; 
//    end
//
//    q_nrzi_data_FastBuf0 <= q_nrzi_data;
//end
//
//
//assign DRU_dbg_signals_o[0] = DRU_dbg_signals_serdes[2];
//assign DRU_dbg_signals_o[1] = DRU_dbg_signals_serdes[4];
//assign DRU_dbg_signals_o[2] = DRU_dbg_signals_serdes[6];
//assign DRU_dbg_signals_o[3] = byte_division_reg[0];
//assign DRU_dbg_signals_o[4] = byte_division_reg[1];
//assign DRU_dbg_signals_o[5] = byte_division_reg[2];
//assign DRU_dbg_signals_o[6] = byte_division_reg[3];













//assign w_eop = (
//                //(w_data_19[18:11] == 8'b0111_1111) ||
//                //(w_data_19[17:10] == 8'b0111_1111) ||
//                //(w_data_19[16:9] == 8'b0111_1111) ||
//                (w_data_19[15:8] == 8'b1111_1110) ||
//                (w_data_19[14:7] == 8'b1111_1110) ||
//                (w_data_19[13:6] == 8'b1111_1110) ||
//                (w_data_19[12:5] == 8'b1111_1110) ||
//                (w_data_19[11:4] == 8'b1111_1110) ||
//                (w_data_19[10:3] == 8'b1111_1110) ||
//                (w_data_19[09:2] == 8'b1111_1110) ||
//                (w_data_19[08:1] == 8'b1111_1110) ||
//                (w_data_19[07:0] == 8'b1111_1110) 
//            );

















assign w_align_data = (q_shift==3'h1) ? {w_rxd[0:0],q_rxd_0[7:1]} :
                      (q_shift==3'h2) ? {w_rxd[1:0],q_rxd_0[7:2]} :
                      (q_shift==3'h3) ? {w_rxd[2:0],q_rxd_0[7:3]} :
                      (q_shift==3'h4) ? {w_rxd[3:0],q_rxd_0[7:4]} :
                      (q_shift==3'h5) ? {w_rxd[4:0],q_rxd_0[7:5]} :
                      (q_shift==3'h6) ? {w_rxd[5:0],q_rxd_0[7:6]} :
                      (q_shift==3'h7) ? {w_rxd[6:0],q_rxd_0[7:7]} : q_rxd_0[7:0];
assign w_align_data_pre = (q_shift==3'h1) ? {q_rxd_0[0:0],q_rxd_1[7:1]} :
                          (q_shift==3'h2) ? {q_rxd_0[1:0],q_rxd_1[7:2]} :
                          (q_shift==3'h3) ? {q_rxd_0[2:0],q_rxd_1[7:3]} :
                          (q_shift==3'h4) ? {q_rxd_0[3:0],q_rxd_1[7:4]} :
                          (q_shift==3'h5) ? {q_rxd_0[4:0],q_rxd_1[7:5]} :
                          (q_shift==3'h6) ? {q_rxd_0[5:0],q_rxd_1[7:6]} :
                          (q_shift==3'h7) ? {q_rxd_0[6:0],q_rxd_1[7:7]} : q_rxd_1[7:0];

reg w_sync_reg0;
reg reg2;
wire [6:0] sync_bitstuff_wires;
wire sync_bitstuff;
assign sync_bitstuff_wires = {w_align_data[4:0], w_align_data_pre[7:6]};
assign sync_bitstuff =  w_sync_reg0 && (&(sync_bitstuff_wires) || 
                        ~|(sync_bitstuff_wires));
always @(posedge clk) begin
    w_sync_reg0 <= w_sync;
    reg2 <= sync_bitstuff;
end

////nrzi removal////////////
assign w_nrzi_data = {!(w_align_data[7]^w_align_data[6]),
                      !(w_align_data[6]^w_align_data[5]),
                      !(w_align_data[5]^w_align_data[4]),
                      !(w_align_data[4]^w_align_data[3]),
                      !(w_align_data[3]^w_align_data[2]),
                      !(w_align_data[2]^w_align_data[1]),
                      !(w_align_data[1]^w_align_data[0]),
                      !(w_align_data[0]^w_align_data_pre[7])};


always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        q_prev_bit <= 0;
        q_nrzi_data<= 0;
    end
    else begin
        q_prev_bit <= ((state==IDLE_STATE) & w_sync) ? 1'b0 : w_align_data[7];
        if (q_rxd_val_0) begin
            q_nrzi_data<= w_nrzi_data;
        end
    end
end

////zeros removal////////////
assign w_data_19  = {w_nrzi_data[2:0], q_nrzi_data, q_nrzi_data2};
assign w_smark_19 = {w_smark, q_smark} & w_ofs_mask;
assign w_smark = reg2 ? 11'b00000100000 :  
                (state==IDLE_STATE) ? 'd0 : { &{~w_data_19[18],w_data_19[17:12]}
                                              ,&{~w_data_19[17],w_data_19[16:11]}
                                              ,&{~w_data_19[16],w_data_19[15:10]}
                                              ,&{~w_data_19[15],w_data_19[14:9]}
                                              ,&{~w_data_19[14],w_data_19[13:8]}
                                              ,&{~w_data_19[13],w_data_19[12:7]}
                                              ,&{~w_data_19[12],w_data_19[11:6]}
                                              ,&{~w_data_19[11],w_data_19[10:5]}
                                              ,&{~w_data_19[10],w_data_19[ 9:4]}
                                              ,&{~w_data_19[ 9],w_data_19[ 8:3]}
                                              ,&{~w_data_19[ 8],w_data_19[ 7:2]}};

wire [9:0] w_ofs_smark;
wire [3:0] w_offset_inc;

assign w_ofs_smark = w_smark_19[q_offset+:10];
assign w_offset_inc = (w_ofs_smark==10'b0010000001) |
                      (w_ofs_smark==10'b0100000010) |
                      (w_ofs_smark==10'b1000000100) |
                      (w_ofs_smark==10'b0100000001) |
                      (w_ofs_smark==10'b1000000010) |
                      (w_ofs_smark==10'b1000000001) ? 4'h2:
                      (w_ofs_smark==10'b1000000000) ? 4'h0:(|w_ofs_smark) ? 4'h1:4'h0;

always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        q_nrzi_data2 <= 0;
        q_offset     <= 0;
        q_smark      <= 0;
        q_data_dly[2:0] <= 0;
    end
    else begin
        if (state == DATA_STATE) begin
            //if (q_rxd_val_0) begin
            if (q_rxd_val_1) begin
                q_nrzi_data2 <= q_nrzi_data;
            end
        end
        else begin
            q_nrzi_data2 <= 0;
        end
        q_offset <= (state==DATA_STATE) ? (q_rxd_val_1 ? (q_offset>=4'h8 ? q_offset-4'h8:q_offset+w_offset_inc) : q_offset) : 0;
        q_smark <= w_smark[7:0];
        q_data_dly[2:0] <= {q_data_dly[1:0], state==DATA_STATE};
    end
end

assign w_ofs_mask = {1'h1 ,q_offset[3:0] <= 4'hf
                          ,q_offset[3:0] <= 4'he
                          ,q_offset[3:0] <= 4'hd
                          ,q_offset[3:0] <= 4'hc
                          ,q_offset[3:0] <= 4'hb
                          ,q_offset[3:0] <= 4'ha
                          ,q_offset[3:0] <= 4'h9
                          ,q_offset[3:0] <= 4'h8
                          ,q_offset[3:0] <= 4'h7
                          ,q_offset[3:0] <= 4'h6
                          ,q_offset[3:0] <= 4'h5
                          ,q_offset[3:0] <= 4'h4
                          ,q_offset[3:0] <= 4'h3
                          ,q_offset[3:0] <= 4'h2
                          ,q_offset[3:0] <= 4'h1
                          ,q_offset[3:0] == 4'h0};

assign w_data_raw = (w_ofs_smark==10'b0010000001) ? {w_data_19[q_offset+8+:2], w_data_19[q_offset+1+:6]}:
                    (w_ofs_smark==10'b0100000010) ? {w_data_19[q_offset+9], w_data_19[q_offset+2+:6], w_data_19[q_offset]}:
                    (w_ofs_smark==10'b1000000100) ? {w_data_19[q_offset+3+:6], w_data_19[q_offset+:2]}:
                    (w_ofs_smark==10'b0100000001) ? {w_data_19[q_offset+9], w_data_19[q_offset+1+:7]}:
                    (w_ofs_smark==10'b1000000010) ? {w_data_19[q_offset+2+:7], w_data_19[q_offset]}:
                    (w_ofs_smark==10'b1000000001) ? w_data_19[q_offset+1+:8]:
                    (w_ofs_smark==10'b0000000001) ? w_data_19[q_offset+1+:8]:
                    (w_ofs_smark==10'b0000000010) ? {w_data_19[q_offset+2+:7], w_data_19[q_offset]}:
                    (w_ofs_smark==10'b0000000100) ? {w_data_19[q_offset+3+:6], w_data_19[q_offset+:2]}:
                    (w_ofs_smark==10'b0000001000) ? {w_data_19[q_offset+4+:5], w_data_19[q_offset+:3]}:
                    (w_ofs_smark==10'b0000010000) ? {w_data_19[q_offset+5+:4], w_data_19[q_offset+:4]}:
                    (w_ofs_smark==10'b0000100000) ? {w_data_19[q_offset+6+:3], w_data_19[q_offset+:5]}:
                    (w_ofs_smark==10'b0001000000) ? {w_data_19[q_offset+7+:2], w_data_19[q_offset+:6]}:
                    (w_ofs_smark==10'b0010000000) ? {w_data_19[q_offset+8], w_data_19[q_offset+:7]}: w_data_19[q_offset+:8];

assign DataIn_o = w_data_raw;
assign RxActive_o = (state != IDLE_STATE) & (state!=EOP_STATE) & (!w_eop);
assign RxValid_o = RxActive_o & ((q_shift == 0) ? q_data_dly[2] : q_data_dly[1]) & (q_offset<8) & q_rxd_val_1;
assign LineState = q_linestate_2;

always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        q_linestate   <= 0;
        q_linestate_1 <= 0;
        q_linestate_2 <= 0;
    end
    else begin
        q_linestate   <= w_linestate;
        q_linestate_1 <= q_linestate;
        q_linestate_2 <= (q_linestate_1==q_linestate) ? q_linestate_1 : q_linestate_2;
    end
end

always @(posedge clk, negedge rstn)
  if(!rstn)
    state <= IDLE_STATE;
  else
    case (state)
      IDLE_STATE:
        if(w_sync)              //data bus exits se0 idle state
          state <= DATA_STATE;
      DATA_STATE:
        if (w_eop)
          state <= EOP_STATE;
      EOP_STATE:
          state <= IDLE_STATE;
    endcase

endmodule


module `getname(usb_tx_phy_hs,`module_name)
(
  input wire       fclk             , //480mhz
  input wire       clk              , //60mhz
  input wire       sclk             , //120mhz
  input wire       rstn             ,
  input wire       phy_mode         ,
  input wire       reset_calib      ,
  // Transciever Interface
  output wire      txdx             ,
  output wire      txoe             ,
  // UTMI Interface
  input  wire [7:0]DataOut_i        ,
  input  wire      TxValid_i        ,
  output wire      TxReady_o        ,
  input  wire [1:0]OpMode_i
);
parameter IDLE_STATE = 0;
parameter SOP_STATE0 = 1;
parameter SOP_STATE1 = 2;
parameter SOP_STATE2 = 3;
parameter SOP_STATE3 = 4;
parameter DATA_STATE = 5;
parameter CRC_STATE  = 6;
parameter EOP1_STATE = 7;
parameter EOP2_STATE = 8;
parameter EOP3_STATE = 9;
parameter EOP4_STATE = 10;
parameter EOP5_STATE = 11;

reg  [ 7:0] q_data_hold, q_data_hold_2, q_data_hold_3;
wire [15:0] w_data_16;
wire [ 4:0] w_zero_stuff_bit;
reg  [ 3:0] q_rem_ones;
reg  [ 7:0] q_data_packetized;
reg         q_skip_byte;
wire        skip_byte_flag;
wire [7:0]  w_data_stuff, w_data_shift;
reg  [7:0]  q_nrzi;
reg  [3:0]  state, next_state;
reg  [2:0]  q_offset;
wire [7:0]  w_nrzi;
wire        txoe_int;
reg         rst_fifo=0;
reg  [7:0]  cnt_rst_fifo=0;
wire        w_empty;
reg         q_empty, q2_empty;
reg         data_out_en, data_out_en_1, data_out_en_2;
wire [3:0]  w_data_out;
reg  [3:0]  data_out;
reg  [3:0]  tx_data;
reg  [3:0]  tx_data_d0;
reg         txoe_int_d0;
reg         txoe_int_d1;
wire        fifo_wr ;
wire [7:0]  fifo_wr_data ;

always @(posedge sclk, negedge rstn) begin
    if (~rstn) begin
        cnt_rst_fifo<=0;
        rst_fifo<=0;
    end
    else begin
        cnt_rst_fifo<=cnt_rst_fifo[7]?cnt_rst_fifo:cnt_rst_fifo+1'b1;
        rst_fifo<=cnt_rst_fifo[7]?1'b0:1'b1;
    end
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        q_data_hold     <= 0;
        q_data_hold_2   <= 0;
        q_data_hold_3   <= 0;
    end
    else begin
        q_data_hold     <= (((state==SOP_STATE3) | (state==DATA_STATE)) & TxReady_o) ? DataOut_i :
        q_skip_byte ? q_data_hold :
        ((state==DATA_STATE) & (!TxValid_i)) ? 8'hFE :
        (state==CRC_STATE  ) ? 8'hFE    :
        (state==IDLE_STATE ) ? 8'h00    :
        (state==EOP1_STATE ) ? 8'h00    : q_data_hold;
        q_data_hold_2   <=  q_skip_byte ? q_data_hold_2 : q_data_hold;
        q_data_hold_3   <=  q_skip_byte ? q_data_hold_3 : q_data_hold_2;
    end
end

assign w_data_16 = {q_data_hold, q_data_hold_2};

assign w_zero_stuff_bit = ((state==EOP1_STATE)&(q_offset==0)&(q_rem_ones!=6)) ? 0 :
                          ((state==CRC_STATE)|(state==EOP1_STATE)|(state==DATA_STATE)|(state==SOP_STATE3))?
                          (&w_data_shift[5:0] & q_rem_ones==6 )  ? 9 :
                          (q_rem_ones==6)                        ? 1 :
                          (&w_data_shift[0:0] & q_rem_ones==5)   ? 2 :
                          (&w_data_shift[1:0] & q_rem_ones==4)   ? 3 :
                          (&w_data_shift[2:0] & q_rem_ones==3)   ? 4 :
                          (&w_data_shift[3:0] & q_rem_ones==2)   ? 5 :
                          (&w_data_shift[4:0] & q_rem_ones==1)   ? 6 :
                          (&w_data_shift[5:0])                   ? 7 :
                          (&w_data_shift[6:1])                   ? 8 :0 :0;

always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        q_rem_ones  <= 0;
        q_offset    <= 0;
        q_skip_byte <= 0;
    end
    else begin
        q_rem_ones  <= (state==SOP_STATE3) ? 1:
                       (&w_data_stuff[7:2]) ? 6 :  //number of ones on most significant bits
                       (&w_data_stuff[7:3]) ? 5 :
                       (&w_data_stuff[7:4]) ? 4 :
                       (&w_data_stuff[7:5]) ? 3 :
                       (&w_data_stuff[7:6]) ? 2 :
                       (&w_data_stuff[7:7]) ? 1 : 0;
        q_offset    <= (state!=IDLE_STATE) ? (w_zero_stuff_bit!=0) ? q_offset + 1'b1 + (w_zero_stuff_bit == 5'd9) : q_offset : 0;
        q_skip_byte <= ((state==IDLE_STATE)|(state>=EOP2_STATE)) ? 0 : skip_byte_flag;
    end
end

assign skip_byte_flag = ((q_offset == 5'd6) & (w_zero_stuff_bit == 5'd9))|((q_offset == 5'd7) & (w_zero_stuff_bit != 0));

assign w_data_shift = (q_offset==0) ? (q_skip_byte) ? w_data_16[ 7:0] : w_data_16[15:8] :
                      (q_offset==1) ? (q_skip_byte) ? {w_data_16[6:0],q_data_hold_3[7]} :w_data_16[14:7] :
                      (q_offset==2) ? w_data_16[13:6] :
                      (q_offset==3) ? w_data_16[12:5] :
                      (q_offset==4) ? w_data_16[11:4] :
                      (q_offset==5) ? w_data_16[10:3] :
                      (q_offset==6) ? w_data_16[ 9:2] :
                      (q_offset==7) ? w_data_16[ 8:1] : w_data_16[15:8];

assign w_data_stuff = (w_zero_stuff_bit==9) ? {1'b0, w_data_shift[5:0], 1'b0} :
                      (w_zero_stuff_bit==8) ? {                   1'b0, w_data_shift[6:0]} :
                      (w_zero_stuff_bit==7) ? {w_data_shift[6:6], 1'b0, w_data_shift[5:0]} :
                      (w_zero_stuff_bit==6) ? {w_data_shift[6:5], 1'b0, w_data_shift[4:0]} :
                      (w_zero_stuff_bit==5) ? {w_data_shift[6:4], 1'b0, w_data_shift[3:0]} :
                      (w_zero_stuff_bit==4) ? {w_data_shift[6:3], 1'b0, w_data_shift[2:0]} :
                      (w_zero_stuff_bit==3) ? {w_data_shift[6:2], 1'b0, w_data_shift[1:0]} :
                      (w_zero_stuff_bit==2) ? {w_data_shift[6:1], 1'b0, w_data_shift[0:0]} :
                      (w_zero_stuff_bit==1) ? {w_data_shift[6:0], 1'b0                   } : w_data_shift[7:0];

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        q_data_packetized  <= 8'h00;
    end
    else begin
        q_data_packetized  <= (state==SOP_STATE0) ? 8'h00 :
                              (state==SOP_STATE1) ? 8'h00 :
                              (state==SOP_STATE2) ? 8'h00 :
                              (state==SOP_STATE3) ? 8'h80 : w_data_stuff;
    end
end


assign w_nrzi[0] = (OpMode_i==2'b10) ? DataOut_i[0] : q_data_packetized[0] ? q_nrzi[7] : ~q_nrzi[7];
assign w_nrzi[1] = (OpMode_i==2'b10) ? DataOut_i[1] : q_data_packetized[1] ? w_nrzi[0] : ~w_nrzi[0];
assign w_nrzi[2] = (OpMode_i==2'b10) ? DataOut_i[2] : q_data_packetized[2] ? w_nrzi[1] : ~w_nrzi[1];
assign w_nrzi[3] = (OpMode_i==2'b10) ? DataOut_i[3] : q_data_packetized[3] ? w_nrzi[2] : ~w_nrzi[2];
assign w_nrzi[4] = (OpMode_i==2'b10) ? DataOut_i[4] : q_data_packetized[4] ? w_nrzi[3] : ~w_nrzi[3];
assign w_nrzi[5] = (OpMode_i==2'b10) ? DataOut_i[5] : q_data_packetized[5] ? w_nrzi[4] : ~w_nrzi[4];
assign w_nrzi[6] = (OpMode_i==2'b10) ? DataOut_i[6] : q_data_packetized[6] ? w_nrzi[5] : ~w_nrzi[5];
assign w_nrzi[7] = (OpMode_i==2'b10) ? DataOut_i[7] : q_data_packetized[7] ? w_nrzi[6] : ~w_nrzi[6];

always @(posedge clk or negedge rstn) begin
    if(!rstn) begin
        q_nrzi  <= 8'hFF;
    end
    else begin
        if(!txoe_int)
            q_nrzi  <= 8'hAA;
        else
            q_nrzi  <= w_nrzi;
    end
end



always @(posedge sclk, negedge rstn) begin
  if (~rstn) begin
      q_empty <= 0;
      q2_empty <= 0;
      data_out_en <= 0;
      data_out_en_1 <= 0;
      data_out_en_2 <= 0;
      data_out <= 0;
  end
  else begin
      q_empty <= w_empty;
      q2_empty <= q_empty;
      data_out_en <= w_empty | q_empty | q2_empty;
      data_out_en_1 <= data_out_en;
      data_out_en_2 <= data_out_en_1;
      data_out <= w_data_out;
  end
end

always @(posedge sclk, negedge rstn) begin
    if (~rstn) begin
        tx_data <= 4'd0;
        tx_data_d0 <= 4'd0;
    end
    else begin
        tx_data <= data_out;
        tx_data_d0 <= tx_data;
    end
end

always @(posedge sclk, negedge rstn) begin
    if (~rstn) begin
        txoe_int_d0 <= 1'b0;
        txoe_int_d1 <= 1'b0;
    end
    else begin
        txoe_int_d0 <= txoe_int;
        txoe_int_d1 <= txoe_int_d0;
    end
end

  //Serializer - txd
  OSER8 oser8txd (
    .Q0     (txdx),
    .Q1     (txoe),
    `ifdef TX_DELAY_1_BIT
        .D0     (tx_data_d0[3]),//4 K
        .D1     (tx_data_d0[3]),//4 K
        .D2     (tx_data[0]   ),//4 K
        .D3     (tx_data[0]   ),//4 K
        .D4     (tx_data[1]   ),//4 K
        .D5     (tx_data[1]   ),//4 K
        .D6     (tx_data[2]   ),//4 K
        .D7     (tx_data[2]   ),//4 K
     `elsif TX_DELAY_2_BIT
        .D0     (tx_data_d0[2]),//4 K
        .D1     (tx_data_d0[2]),//4 K
        .D2     (tx_data_d0[3]),//4 K
        .D3     (tx_data_d0[3]),//4 K
        .D4     (tx_data[0]   ),//4 K
        .D5     (tx_data[0]   ),//4 K
        .D6     (tx_data[1]   ),//4 K
        .D7     (tx_data[1]   ),//4 K
    `else
        .D0     (tx_data[0]),//18 K
        .D1     (tx_data[0]),//18 K
        .D2     (tx_data[1]),//18 K
        .D3     (tx_data[1]),//18 K
        .D4     (tx_data[2]),//18 K
        .D5     (tx_data[2]),//18 K
        .D6     (tx_data[3]),//18 K
        .D7     (tx_data[3]),//18 K
    `endif
    .TX0    (tx_en|(data_out_en_2|q_empty)),//(w_empty | q_empty | q2_empty)&data_out_en_1data_out_en&!txoe_int),
    .TX1    (tx_en|(data_out_en_2|q_empty)),//(w_empty | q_empty | q2_empty)&data_out_en_1data_out_en&!txoe_int),
    .TX2    (tx_en|(data_out_en_2|q_empty)),//(w_empty | q_empty | q2_empty)&data_out_en_1data_out_en&!txoe_int),
    .TX3    (tx_en|(data_out_en_2|q_empty)),//(w_empty | q_empty | q2_empty)&data_out_en_1data_out_en&!txoe_int),
    .PCLK   (sclk),
    .FCLK   (fclk),
    .RESET  (reset_calib)
  );



assign fifo_wr = (OpMode_i==2'b10) ? TxValid_i : txoe_int;
assign fifo_wr_data = (OpMode_i==2'b10) ? DataOut_i : q_nrzi;
assign tx_en = (OpMode_i==2'b01);

`getname(fifo_8to4,`module_name) u_fifo_8to4
  (
     .Data (fifo_wr_data),//
     .WrClk(clk         ),//
     .RdClk(sclk        ),//
     .WrEn (fifo_wr     ),//
     .RdEn (!q2_empty   ),//
     .Reset(!rstn       ),//
     .Q    (w_data_out  ),//
     .Empty(w_empty     ),//
     .Full () //output Full
 );


assign txoe_int = (state!=IDLE_STATE)&(state!=SOP_STATE0)&(state!=SOP_STATE1) ? 1'b1 : 1'b0;
assign TxReady_o = (OpMode_i==2'b10) ? TxValid_i : ((state==SOP_STATE3) | (state==DATA_STATE && TxValid_i)) & !q_skip_byte;

  //======================================================================================--
  //-- Tx Statemashine                                                                    --
  //======================================================================================--

always @(posedge clk or negedge rstn) begin
    if (!rstn)
        state <= IDLE_STATE;
    else
        state <= next_state;
end


always @(rstn, state, TxValid_i, skip_byte_flag, q_skip_byte, OpMode_i) begin
    if (!rstn) begin
        next_state <= IDLE_STATE;
    end
    else begin
        case (state)
            IDLE_STATE : begin
                if (TxValid_i & (OpMode_i!=2'b01))
                    next_state <= SOP_STATE0;
                else
                    next_state <= IDLE_STATE;
            end
            SOP_STATE0 : next_state <= SOP_STATE1;
            SOP_STATE1 : next_state <= SOP_STATE2;
            SOP_STATE2 : next_state <= SOP_STATE3;
            SOP_STATE3 : next_state <= DATA_STATE;
            DATA_STATE : begin
                if ((!TxValid_i)&(q_skip_byte|skip_byte_flag))
                  next_state <= CRC_STATE;
                else if(!TxValid_i)
                  next_state <= EOP1_STATE;
                else
                  next_state <= DATA_STATE;
            end
            CRC_STATE  : next_state <= EOP1_STATE;
            EOP1_STATE : begin
                if (!skip_byte_flag)
                   next_state <= EOP2_STATE;
                else
                   next_state <= EOP1_STATE;
            end
            EOP2_STATE : next_state <= EOP3_STATE;
            EOP3_STATE : next_state <= EOP4_STATE;
            EOP4_STATE : next_state <= EOP5_STATE;
            EOP5_STATE : next_state <= IDLE_STATE;
            default    : next_state <= IDLE_STATE;
        endcase
    end
end

endmodule



module `getname(cdr_serdes_x8,`module_name)
(
    input wire rstn           ,
    input wire fclk_deg0      ,//480mhz
    input wire clk60mhz       ,
    input wire rxd            ,
    input wire rxdp           ,
    input wire rxdn           ,
    input wire sclk           ,//120mhz
    input wire reset_calib    ,
    input wire eop            ,
    //data busses
    output wire [7:0] data_out,
    output wire       dval,
    output wire       data_err,
    output reg [1:0] LineState,
    input  wire TxValid_i
    ,output wire serdes_rx_en
    ,output wire [15 : 0] DRU_dbg_signals_o
);

localparam IDLE = 0;
localparam TX = 1;
localparam RX = 2;
wire        w_empty;
wire [4:0]  s_data_out;
wire [1:0]  data_valid;
wire [15:0] ides8_out;
reg         dru_rstn;
reg  [7:0]  rx_data;
reg  [7:0]  fifo_wr_data ;
reg         fifo_wr ;
reg         fifo_rd;
reg  [3:0]  rx_bit_count;
wire [7:0]  fifo_rd_data;
wire [1:0]  ref_line_state;
wire [7:0]  dp;
wire [7:0]  dn;
reg  [7:0]  q_tx_cnt;
reg  [3:0]  rx0_cnt;
reg  [3:0]  rx1_cnt;
reg  [1:0]  s_state;
reg         rx_en;
reg         eop_buf0;

//assign serdes_rx_en = rx_en;

//assign DRU_dbg_signals_o[0] = |ides8_out;
//assign DRU_dbg_signals_o[1] = &ides8_out;
//assign DRU_dbg_signals_o[2] = rx_en;
//assign DRU_dbg_signals_o[3] = fifo_wr_FastBuf1;
//assign DRU_dbg_signals_o[4] = fifo_rd;
//assign DRU_dbg_signals_o[5] = w_empty;
//assign DRU_dbg_signals_o[6] = eop;
//assign DRU_dbg_signals_o[7] = 1'b0;
//assign DRU_dbg_signals_o[8] = 1'b0;
//assign DRU_dbg_signals_o[9] = 1'b0;
//assign DRU_dbg_signals_o[10] = 1'b0;
//assign DRU_dbg_signals_o[11] = 1'b0;
//assign DRU_dbg_signals_o[12] = 1'b0;
//assign DRU_dbg_signals_o[13] = 1'b0;
//assign DRU_dbg_signals_o[14] = 1'b0;
//assign DRU_dbg_signals_o[15] = 1'b0;

//==============================================================
//======Ref DP DN
IDES8 dp_ref_ides8(
  .Q0(dp[7]),
  .Q1(dp[6]),
  .Q2(dp[5]),
  .Q3(dp[4]),
  .Q4(dp[3]),
  .Q5(dp[2]),
  .Q6(dp[1]),
  .Q7(dp[0]),
  .D(rxdp),
  .FCLK(fclk_deg0),
  .PCLK(sclk),
  .CALIB(1'b0),
  .RESET(1'b0)
);
IDES8 dn_ref_ides8(
  .Q0(dn[7]),
  .Q1(dn[6]),
  .Q2(dn[5]),
  .Q3(dn[4]),
  .Q4(dn[3]),
  .Q5(dn[2]),
  .Q6(dn[1]),
  .Q7(dn[0]),
  .D (rxdn),
  .FCLK(fclk_deg0),
  .PCLK(sclk),
  .CALIB(1'b0),
  .RESET(1'b0)
);
assign  ref_line_state = {|dn,|dp};

//==============================================================
//======DRU
//DRU_500M_Top #(
`getname(DRU_500M_Top ,`module_name) #(
    .IODELAY_A(0),
    .IODELAY_B(25)
    )
DRU_500M_Top
(
     .serial_data(rxd          )// serial data at line rate
    ,.sys_clk    (clk60mhz     )// a system clock, such as 50MHz crystal, use as PLL reference clock and OSER_Reset sync FSM clock
    ,.rstn       (dru_rstn     )//rstn connect to external global reset
    ,.fclk       (fclk_deg0    )// plck is synced to data_valid and data_out
    ,.sclk       (sclk         )// plck is synced to data_valid and data_out
    ,.reset_calib(reset_calib  )
    ,.ides8_out  (ides8_out    )
    ,.data_out   (s_data_out   )// DRU data output
    ,.data_valid (data_valid   )// DRU data output flag. 2b'00: data_out[2:0] valid; 2b'01: data_out[3:0] valid; 2b'11: data_out[4:0] valid
//    ,.DRU_dbg_signals_o(DRU_dbg_signals_o)
);
//==============================================================
//======RX Data 4to8
always @(posedge sclk or negedge rstn) begin
    if (!rstn) begin
        rx_data <= 4'd0;
        rx_bit_count <= 4'd0;
        fifo_wr_data <= 8'd0;
        fifo_wr <= 1'd0;
    end
    else begin
        case (data_valid)
            2'b00 : begin
                rx_data <= {rx_data[4:0],s_data_out[2:0]};
                if (rx_bit_count == 4'd5) begin
                    rx_bit_count <= 4'd0;
                    fifo_wr_data <= {rx_data[4:0],s_data_out[2:0]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd6) begin
                    rx_bit_count <= 4'd1;
                    fifo_wr_data <= {rx_data[5:0],s_data_out[2:1]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd7) begin
                    rx_bit_count <= 4'd2;
                    fifo_wr_data <= {rx_data[6:0],s_data_out[2:2]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd8) begin
                    rx_bit_count <= 4'd3;
                    fifo_wr_data <= {rx_data[7:0]};
                    fifo_wr <= 1'b1;
                end
                else begin
                    rx_bit_count <= rx_bit_count + 4'd3;
                    fifo_wr <= 1'b0;
                end
            end
            2'b01 : begin
                rx_data <= {rx_data[3:0],s_data_out[3:0]};
                if (rx_bit_count == 4'd4) begin
                    rx_bit_count <= 4'd0;
                    fifo_wr_data <= {rx_data[3:0],s_data_out[3:0]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd5) begin
                    rx_bit_count <= 4'd1;
                    fifo_wr_data <= {rx_data[4:0],s_data_out[3:1]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd6) begin
                    rx_bit_count <= 4'd2;
                    fifo_wr_data <= {rx_data[5:0],s_data_out[3:2]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd7) begin
                    rx_bit_count <= 4'd3;
                    fifo_wr_data <= {rx_data[6:0],s_data_out[3:3]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd8) begin
                    rx_bit_count <= 4'd4;
                    fifo_wr_data <= {rx_data[7:0]};
                    fifo_wr <= 1'b1;
                end
                else begin
                    rx_bit_count <= rx_bit_count + 4'd4;
                    fifo_wr <= 1'b0;
                end
            end
            2'b11 : begin
                rx_data <= {rx_data[2:0],s_data_out[4:0]};
                if (rx_bit_count == 4'd3) begin
                    rx_bit_count <= 4'd0;
                    fifo_wr_data <= {rx_data[2:0],s_data_out[4:0]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd4) begin
                    rx_bit_count <= 4'd1;
                    fifo_wr_data <= {rx_data[3:0],s_data_out[4:1]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd5) begin
                    rx_bit_count <= 4'd2;
                    fifo_wr_data <= {rx_data[4:0],s_data_out[4:2]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd6) begin
                    rx_bit_count <= 4'd3;
                    fifo_wr_data <= {rx_data[5:0],s_data_out[4:3]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd7) begin
                    rx_bit_count <= 4'd4;
                    fifo_wr_data <= {rx_data[6:0],s_data_out[4:4]};
                    fifo_wr <= 1'b1;
                end
                else if (rx_bit_count == 4'd8) begin
                    rx_bit_count <= 4'd5;
                    fifo_wr_data <= {rx_data[7:0]};
                    fifo_wr <= 1'b1;
                end
                else begin
                    rx_bit_count <= rx_bit_count + 4'd5;
                    fifo_wr <= 1'b0;
                end
            end
            default: begin
                fifo_wr <= 1'd0;
            end
        endcase
    end
end


//reg [7:0] fifo_wr_data_SlowBuf0, fifo_wr_data_SlowBuf1, fifo_wr_data_FastBuf0, fifo_wr_data_FastBuf1;
//reg fifo_wr_SlowBuf0, fifo_wr_SlowBuf1, fifo_wr_FastBuf0, fifo_wr_FastBuf1;
//reg eop_SlowBuf0, eop_SlowBuf1, eop_FastBuf0, eop_FastBuf1;
//reg rx_en_SlowBuf0, rx_en_SlowBuf1, rx_en_FastBuf0, rx_en_FastBuf1;
//
//always @(posedge clk60mhz) begin
//    fifo_wr_data_SlowBuf0 <= fifo_wr_data;
//    fifo_wr_data_SlowBuf1 <= fifo_wr_data_SlowBuf0;
//
//    fifo_wr_SlowBuf0 <= fifo_wr;
//    fifo_wr_SlowBuf1 <= fifo_wr_SlowBuf0;
//
//    eop_SlowBuf0 <= eop;
//    eop_SlowBuf1 <= eop_SlowBuf0;
//
//    rx_en_SlowBuf0 <= rx_en;
//    rx_en_SlowBuf1 <= rx_en_SlowBuf0;
//end
//
//always @(posedge sclk) begin
//    fifo_wr_data_FastBuf0 <= fifo_wr_data_SlowBuf1;
//    fifo_wr_data_FastBuf1 <= fifo_wr_data_FastBuf0;
//
//    fifo_wr_FastBuf0 <= fifo_wr_SlowBuf1;
//    fifo_wr_FastBuf1 <= fifo_wr_FastBuf0;
//
//    eop_FastBuf0 <= eop_SlowBuf1;
//    eop_FastBuf1 <= eop_FastBuf0;
//
//    rx_en_FastBuf0 <= rx_en_SlowBuf1;
//    rx_en_FastBuf1 <= rx_en_FastBuf0;
//end




//==============================================================
//======RX Enable
always @(posedge sclk or negedge rstn) begin
    if (!rstn) begin
        s_state <= 0;
        q_tx_cnt <= 0;
        rx_en <= 0;
        dru_rstn <= 0;
        rx0_cnt <= 'd0;
        rx1_cnt <= 'd0;
    end
    else begin
        case(s_state)
            IDLE: begin
                rx0_cnt <= 'd0;
                rx1_cnt <= 'd0;
                q_tx_cnt <= 0;
                dru_rstn <= 1;
                if (TxValid_i) begin
                    s_state <= TX;
                    rx_en <= 0;
                end
                else if (ref_line_state != 2'b00) begin
                    if ((|ides8_out!=1'b0)&&(&ides8_out!=1'b1)) begin
                        s_state <= RX;
                        rx_en <= 1;
                    end
                end
                else begin
                    rx_en <= 'd0;
                end
            end
            TX: begin //hold the receive bus until transmission is complete
                dru_rstn <= 1'b0;
                if(TxValid_i)
                    q_tx_cnt <= 0;
                else if(q_tx_cnt < 45)
                    q_tx_cnt <= q_tx_cnt + 1'b1;
                else
                    s_state <= IDLE;
            end
            RX: begin
                if (eop /*eop_FastBuf1*/||TxValid_i) begin
                    q_tx_cnt <= 'd0;
                    s_state <= IDLE;
                    dru_rstn <= 0;
                end
            end
        endcase
    end
end

 `getname(fifo_8to8,`module_name) fifo_rx
 (
      .Data     ({  fifo_wr_data[0],//fifo_wr_data_FastBuf1[0],
                    fifo_wr_data[1],//fifo_wr_data_FastBuf1[1],
                    fifo_wr_data[2],//fifo_wr_data_FastBuf1[2],
                    fifo_wr_data[3],//fifo_wr_data_FastBuf1[3],
                    fifo_wr_data[4],//fifo_wr_data_FastBuf1[4],
                    fifo_wr_data[5],//fifo_wr_data_FastBuf1[5],
                    fifo_wr_data[6],//fifo_wr_data_FastBuf1[6],
                    fifo_wr_data[7] //fifo_wr_data_FastBuf1[7]
                    })
     ,.Reset    (!rstn        )
     ,.WrClk    (sclk         )
     ,.RdClk    (clk60mhz     )
     ,.WrEn     (fifo_wr & rx_en)//fifo_wr_FastBuf1 & rx_en_FastBuf1)
     ,.RdEn     (fifo_rd      )
     ,.Q        (fifo_rd_data )
     ,.Empty    (w_empty      )
     ,.Full     (             )
);


//async_fifo_ft #(
//    .DSIZE      ( 8 ),
//    .ASIZE      ( 6 )
//) usb_phy_rx_fifo_inst ( 
//
//    // External side (write)
//     .wclk         ( sclk ) 
//    ,.wrst_n       ( rstn ) 
//    ,.winc         ( fifo_wr & rx_en) 
//    ,.wdata        ( {fifo_wr_data[0],
//                      fifo_wr_data[1],
//                      fifo_wr_data[2],
//                      fifo_wr_data[3],
//                      fifo_wr_data[4],
//                      fifo_wr_data[5],
//                      fifo_wr_data[6],
//                      fifo_wr_data[7]}
//                   )
//    ,.wfull        ( ) 
//    ,.awfull       ( ) 
//    ,.wnum         ( ) 
//    ,.wavail_th    ( ) 
//    ,.wemptyenough ( ) 
//    ,.wavail       ( )  
//    
//    // USB side (read)
//    ,.rclk         ( clk60mhz    ) 
//    ,.rrst_n       ( rstn        ) 
//    ,.rinc         ( fifo_rd     ) 
//    ,.rdata        ( fifo_rd_data) 
//    ,.rempty       ( w_empty     ) 
//    ,.rnum         ( ) 
//    ,.arempty      ( ) 
//    ,.rnum_th      ( ) 
//    ,.rfullenough  ( )
//);


//==============================================================
//======FIFO Read
//assign  data_out = (!fifo_rd) ? 8'd0 : fifo_rd_data;
assign  data_out = fifo_rd_data;
assign  dval = fifo_rd&(!w_empty);
always @(posedge clk60mhz or negedge rstn) begin
    if (!rstn) begin
        fifo_rd <= 1'b0;
    end
    else begin
        if (w_empty) begin
            fifo_rd <= 1'b0;
        end
        else begin
            fifo_rd <= 1'b1;
        end
    end
end
assign data_err = 1'b0;


//==============================================================
//======Line State
always @(posedge sclk or negedge rstn) begin
    if (!rstn) begin
        LineState <= 2'b00;
    end
    else begin
        if (ref_line_state != 2'b00) begin
            if (|ides8_out) begin
                LineState <= 2'b01;
            end
            else begin
                LineState <= 2'b10;
            end
        end
        else begin
            LineState <= 2'b00;
            //    LineState <= 2'b00;
            //if ((|ides8_out==1'b0)||(&ides8_out==1'b1)) begin
            //    LineState <= 2'b00;
            //end
            //else begin
            //    LineState <= 2'b01;
            //end
        end
    end
end


endmodule



 // `define module_name  fifo_8to4
//  `define getname(oriName,tmodule_name) \~oriName.tmodule_name
   `define     EBR_BASED
   `define     En_Reset
   `define     Reset_Synchronization

`timescale 1ns/1ps
 module `getname(fifo_8to4,`module_name) (
          Data                ,
  `ifdef  En_Reset
     `ifdef Reset_Synchronization
          Reset               ,
     `else
          WrReset             ,
          RdReset             ,
     `endif
  `endif
          WrClk               ,
          RdClk               ,
          WrEn                ,
          RdEn                ,
`ifdef    Al_Empty_Flag
   `ifdef Empty_D_Dual_Th
          AlmostEmptySetTh    ,
          AlmostEmptyClrTh    ,
   `endif
   `ifdef Empty_D_Single_Th
          AlmostEmptyTh       ,
   `endif
`endif

`ifdef    Al_Full_Flag
   `ifdef Full_D_Dual_Th
          AlmostFullSetTh     ,
          AlmostFullClrTh     ,
  `endif
  `ifdef  Full_D_Single_Th
          AlmostFullTh        ,
  `endif
`endif

`ifdef    Count_W
          Wnum                ,
`endif
`ifdef    Count_R
          Rnum                ,
`endif
`ifdef    Al_Empty_Flag
          Almost_Empty        ,
`endif
`ifdef    Al_Full_Flag
          Almost_Full         ,
`endif
`ifdef En_ECC
          ERROR               ,
`endif
          Q                   ,
          Empty               ,
          Full
);

parameter   WDEPTH           = 16;        //when synplify,use these three parameters
parameter   WDSIZE           = 8;
parameter   RDEPTH           = 32;
parameter   ASIZE            =  $clog2(WDEPTH);
parameter   RASIZE           =  $clog2(RDEPTH);
parameter   RDSIZE           =  WDEPTH*WDSIZE/RDEPTH;  // must be divisible, the remainder is 0;
       input           [WDSIZE-1:0]  Data  ;    // Data writed into the FIFO
       input                         WrClk ;    // Write Clock
       input                         RdClk ;    // Read  Clock
       input                         WrEn  ;    // Write Enable
       input                         RdEn  ;    // Read  Enable
`ifdef En_Reset
    `ifdef Reset_Synchronization
       input                         Reset ;    //Reset Synchronization : only Reset
    `else
       input                         WrReset ;  // Two Reset :Write Reset, Read Reset
       input                         RdReset ;
     `endif
`endif

`ifdef Al_Empty_Flag
    `ifdef Empty_D_Dual_Th
       input          [RASIZE-1:0]   AlmostEmptySetTh ;   // Dynamic input threshold of almost empty set to 1
       input          [RASIZE-1:0]   AlmostEmptyClrTh ;   // Dynamic input threshold of almost empty set to 0
    `endif
    `ifdef Empty_D_Single_Th
       input          [RASIZE-1:0]   AlmostEmptyTh    ;   //Dynamic input threshold of  almost empty set to 1
    `endif
`endif

`ifdef Al_Full_Flag
   `ifdef Full_D_Dual_Th
       input          [ASIZE-1:0]    AlmostFullSetTh ;   // Dynamic input threshold of almost full set to 1
       input          [ASIZE-1:0]    AlmostFullClrTh ;   // Dynamic input threshold of almost full set to 0

   `endif
   `ifdef Full_D_Single_Th
       input          [ASIZE-1:0]    AlmostFullTh    ;   //Dynamic input threshold of  almost full set to 1
   `endif
`endif

`ifdef Count_W
       output         [ASIZE:0]      Wnum ;              // Write data count: synchronized to WrClk;
`endif
`ifdef Count_R
       output         [RASIZE:0]     Rnum ;              // Read data count: synchronized to RdCLK;
`endif

`ifdef   Al_Empty_Flag
       output                        Almost_Empty ;      // Flag of Almost empty
`endif
`ifdef   Al_Full_Flag
       output                        Almost_Full  ;      // Flag of Almost full
`endif
`ifdef En_ECC
       output         [1:0]          ERROR        ;
`endif

       output         [RDSIZE-1:0]   Q     ;             // Data read from the fifo
       output                        Empty ;             // Empty flag
       output                        Full  ;             // Full flag

`getname(fifo_8to4_sub,`module_name)  fifo_8to4_sub_inst (
          .Data                  (Data)                ,
  `ifdef  En_Reset
     `ifdef Reset_Synchronization
          .Reset                 (Reset)               ,
     `else
          .WrReset               (WrReset)             ,
          .RdReset               (RdReset)             ,
     `endif
  `endif
          .WrClk                 (WrClk)               ,
          .RdClk                 (RdClk)               ,
          .WrEn                  (WrEn)                ,
          .RdEn                  (RdEn)                ,
`ifdef    Al_Empty_Flag
   `ifdef Empty_D_Dual_Th
          .AlmostEmptySetTh      (AlmostEmptySetTh)    ,
          .AlmostEmptyClrTh      (AlmostEmptyClrTh)    ,
   `endif
   `ifdef Empty_D_Single_Th
          .AlmostEmptyTh         (AlmostEmptyTh)       ,
   `endif
`endif

`ifdef    Al_Full_Flag
   `ifdef Full_D_Dual_Th
          .AlmostFullSetTh       (AlmostFullSetTh)     ,
          .AlmostFullClrTh       (AlmostFullClrTh)     ,
  `endif
  `ifdef  Full_D_Single_Th
          .AlmostFullTh          (AlmostFullTh)        ,
  `endif
`endif

`ifdef    Count_W
          .Wnum                  (Wnum)                ,
`endif
`ifdef    Count_R
          .Rnum                  (Rnum)                ,
`endif
`ifdef    Al_Empty_Flag
          .Almost_Empty          (Almost_Empty)        ,
`endif
`ifdef    Al_Full_Flag
          .Almost_Full           (Almost_Full)         ,
`endif
`ifdef En_ECC
          .ERROR                 (ERROR)               ,
`endif
          .Q                     (Q)                   ,
          .Empty                 ()               ,
          .rempty_val            (Empty)               ,
          .Full                  (Full)
    );

endmodule

//`include "fifo_define.v"
`timescale 1ns/1ps
module `getname(fifo_8to4_sub,`module_name) (
          Data                ,
  `ifdef  En_Reset
     `ifdef Reset_Synchronization
          Reset               ,
     `else
          WrReset             ,
          RdReset             ,
     `endif
  `endif
          WrClk               ,
          RdClk               ,
          WrEn                ,
          RdEn                ,
`ifdef    Al_Empty_Flag
   `ifdef Empty_D_Dual_Th
          AlmostEmptySetTh    ,
          AlmostEmptyClrTh    ,
   `endif
   `ifdef Empty_D_Single_Th
          AlmostEmptyTh       ,
   `endif
`endif

`ifdef    Al_Full_Flag
   `ifdef Full_D_Dual_Th
          AlmostFullSetTh     ,
          AlmostFullClrTh     ,
  `endif
  `ifdef  Full_D_Single_Th
          AlmostFullTh        ,
  `endif
`endif

`ifdef    Count_W
          Wnum                ,
`endif
`ifdef    Count_R
          Rnum                ,
`endif
`ifdef    Al_Empty_Flag
          Almost_Empty        ,
`endif
`ifdef    Al_Full_Flag
          Almost_Full         ,
`endif
`ifdef En_ECC
          ERROR               ,
`endif
          Q                   ,
          Empty               ,
          rempty_val          ,
          Full
);
    //  `include "fifo_parameter.v"
parameter   WDEPTH           = 16;        //when synplify,use these three parameters
parameter   WDSIZE           = 8;
parameter   RDEPTH           = 32;
parameter   ASIZE            =  $clog2(WDEPTH);
parameter   RASIZE           =  $clog2(RDEPTH);
parameter   RDSIZE           =  WDEPTH*WDSIZE/RDEPTH;  // must be divisible, the remainder is 0;
       input           [WDSIZE-1:0]  Data  ;    // Data writed into the FIFO
       input                         WrClk ;    // Write Clock
       input                         RdClk ;    // Read  Clock
       input                         WrEn  ;    // Write Enable
       input                         RdEn  ;    // Read  Enable
`ifdef En_Reset
    `ifdef Reset_Synchronization
       input                         Reset;      //Reset Synchronization : only Reset
    `else
       input                         WrReset ;   // Two Reset :Write Reset, Read Reset
       input                         RdReset ;
     `endif
`endif

`ifdef Al_Empty_Flag
    `ifdef Empty_D_Dual_Th
       input          [RASIZE-1:0]     AlmostEmptySetTh ;   // Dynamic input threshold of almost empty set to 1
       input          [RASIZE-1:0]     AlmostEmptyClrTh ;   // Dynamic input threshold of almost empty set to 0
    `endif
    `ifdef Empty_D_Single_Th
       input          [RASIZE-1:0]     AlmostEmptyTh    ;   //Dynamic input threshold of  almost empty set to 1
    `endif
`endif

`ifdef Al_Full_Flag
   `ifdef Full_D_Dual_Th
       input          [ASIZE-1:0]      AlmostFullSetTh ;   // Dynamic input threshold of almost full set to 1
       input          [ASIZE-1:0]      AlmostFullClrTh ;   // Dynamic input threshold of almost full set to 0

  `endif
  `ifdef Full_D_Single_Th
       input          [ASIZE-1:0]      AlmostFullTh    ;   //Dynamic input threshold of  almost full set to 1
  `endif
`endif

`ifdef Count_W
       output  reg    [ASIZE:0]      Wnum ;              // Write data count: synchronized to WrClk;
`endif
`ifdef Count_R
       output  reg    [RASIZE:0]     Rnum ;              // Read data count: synchronized to RdCLK;
`endif

`ifdef   Al_Empty_Flag
       output  reg                   Almost_Empty ;      // Flag of Almost empty
`endif
`ifdef   Al_Full_Flag
       output  reg                   Almost_Full  ;      // Flag of Almost full
`endif
`ifdef En_ECC
       output         [1:0]          ERROR        ;
`endif

       output         [RDSIZE-1:0]   Q     ;             // Data read from the fifo
       output  reg                   Empty ;             // Empty flag
       output                        rempty_val;
       output  reg                   Full  ;             // Full flag

       reg            [RASIZE:0]     rbin_num      ;     // Read pointer : binary,1-bit bigger than raddr_num inoder to obtain the right flag singal
       wire           [RASIZE-1:0]   raddr_num     ;     // Read address
       wire           [RASIZE:0]     rbin_num_next ;
       wire           [RASIZE:0]     rcnt_sub      ;     // Read data count
       wire           [ASIZE-1:0]    waddr         ;     // Write address
       wire                          rempty_val    ;     // Empty value
       wire                          wfull_val     ;     // Full  value
       wire                          arempty_val   ;     // Almost empty value
       wire                          awfull_val    ;     // Almost full  value
       wire           [ASIZE:0]      wcnt_sub      ;     // Write data count
wire WRst,RRst;
 `ifdef En_ECC
  localparam   PWIDTH =  (WDSIZE == 1 ) ? 3 :
                             (WDSIZE >= 2  && WDSIZE <= 4 ) ? 4 :
                             (WDSIZE >= 5  && WDSIZE <= 11) ? 5 :
                             (WDSIZE >= 12 && WDSIZE <= 26) ? 6 :
                             (WDSIZE >= 27 && WDSIZE <= 57) ? 7 :
                             (WDSIZE >= 58 && WDSIZE <= 64) ? 8 :0;

  `else
       localparam PWIDTH = 0;
 `endif
/******************************************************/
// The read and write logic for differnt depth
/******************************************************/

`ifdef En_Reset
    `ifdef Reset_Synchronization
     reg  [1:0] reset_r;
     reg  [1:0] reset_w;
     always @(negedge RdClk or posedge Reset)
         if(Reset)
           reset_r <= 2'b11;
         else
           reset_r <= {reset_r[0],1'b0};
     assign RRst = reset_r[1];
     always @(negedge WrClk or posedge Reset)
         if(Reset)
           reset_w <= 2'b11;
         else
           reset_w <= {reset_w[0],1'b0};
     assign WRst = reset_w[1];
   `else
     assign RRst = RdReset;
     assign WRst = WrReset;
  `endif
`else
     assign RRst = 0;
     assign WRst = 0;
`endif

     always @(posedge RdClk or posedge RRst)
           if (RRst)
              rbin_num <= 0;
           else
              rbin_num <= rbin_num_next;

generate
  if(WDEPTH < RDEPTH) begin:  Small // WDEPTH < RDEPTH
     localparam                  a = RDEPTH/WDEPTH ;
   `ifdef EBR_BASED
     reg       [WDSIZE-1:0]      mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
   `endif

   `ifdef DSR_BASED
     reg       [WDSIZE-1:0]      mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "distributed_ram" */;
   `endif

   `ifdef LUT_BASED
     reg       [WDSIZE-1:0]      mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "registers" */;
   `endif

  `ifdef En_Output_Reg
     reg       [RDSIZE-1:0]      wdata_q_r ;
  `endif
     reg       [WDSIZE-1:0]      wdata;         // Data from the fifo
     reg       [ASIZE:0]         wptr ;         // Write pointer
     reg       [ASIZE:0]         rptr ;         // Read  pointer
     reg       [ASIZE:0]         wq2_rptr ;     // Read  pointer synchronized to WrClk
     reg       [ASIZE:0]         rq2_wptr ;     // Write pointer synchronized to RdCkl
     reg       [ASIZE:0]         wq1_rptr ;
     reg       [ASIZE:0]         rq1_wptr ;
     reg       [ASIZE:0]         wbin     ;     // Write pointer:binary, 1-bit bigger than waddr in order to obta
                                                // in the right flag signal
     wire      [RASIZE:0]        wcount_r_1 ;
     wire      [ASIZE:0]         rgraynext  ;   // Read pointer: gray code
     wire      [ASIZE:0]         rbinnext   ;
     wire      [ASIZE:0]         rbinnext_1 ;
     wire      [ASIZE:0]         wgraynext  ;
     wire      [ASIZE:0]         wcount_r   ;
     wire      [ASIZE:0]         rcount_w   ;
     wire      [ASIZE:0]         wbinnext   ;
     wire      [RDSIZE-1:0]      wdata_q    ;

   always @(posedge WrClk)    // Write data into fifo
       if(WrEn && !Full)
          mem[waddr] <= Data;
`ifdef FWFT
// Read data from fifo
      always @(posedge RdClk or posedge RRst )
          if(RRst)
             wdata <= 0;
          else  if(RdEn ? ~rempty_val : (Empty & !rempty_val))
             wdata <= mem[raddr_num/a];

  assign wdata_q = RdEn ? wdata[(((raddr_num-1)%a+1)*RDSIZE-1)-:RDSIZE] :  wdata[(((raddr_num)%a+1)*RDSIZE-1)-:RDSIZE];
`else
// Read data from fifo
      always @(posedge RdClk or posedge RRst )
          if(RRst)
             wdata <= 0;
          else  if(RdEn && !Empty)
             wdata <= mem[raddr_num/a];

  assign wdata_q = wdata[(((raddr_num-1)%a+1)*RDSIZE-1)-:RDSIZE];
`endif
// if define the Enable output registers,the data will be output one clock latar
`ifdef FWFT
`ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
             always @(posedge RdClk or posedge RRst )
                 if(RRst)
                    wdata_q_r  <= 0;
                 else  if(RdEn)
                         wdata_q_r <= wdata_q;
   `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                    wdata_q_r <= 0;
                 else
                    wdata_q_r <= wdata_q;
    `endif
`endif
`else
`ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
             always @(posedge RdClk or posedge RRst )
                 if(RRst)
                    wdata_q_r  <= 0;
                 else  if(RdEn & ~Empty)
                         wdata_q_r <= wdata_q;
   `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                    wdata_q_r <= 0;
                 else
                    wdata_q_r <= wdata_q;
    `endif
`endif
`endif

 `ifdef En_Output_Reg
    assign Q              =  wdata_q_r;
   `else
    assign Q              =  wdata_q;
 `endif
`ifdef FWFT
    assign raddr_num      =  rbin_num_next[RASIZE-1:0];        // Read address
`else
    assign raddr_num      =  rbin_num[RASIZE-1:0];        // Read address
`endif
    assign rbin_num_next  =  rbin_num + (RdEn & ~Empty);  // Obtain the next read pointer
    assign rbinnext       =  rbin_num_next[RASIZE:0]/a;   // Read address transform because the different depth
    assign rbinnext_1     =  rbin_num/a;
    assign rgraynext      =  (rbinnext>>1) ^ rbinnext;    // Gray code transform
    assign rempty_val     =  (rgraynext == rq2_wptr);     // Jude empty value
    assign wcount_r       =  gry2bin(rq2_wptr);
    assign wcount_r_1     =  gry2bin(rq2_wptr)*a;         // Write address transform
    assign rcnt_sub       =  {(wcount_r[ASIZE]^rbinnext_1[ASIZE]),wcount_r_1[RASIZE-1:0]}
                             -{1'b0,rbin_num[RASIZE-1:0]}; // Caculate the read data count

    assign waddr          =  wbin[ASIZE-1:0];             // Write address
    assign wbinnext       =  wbin + (WrEn & ~Full);
    assign wgraynext      =  (wbinnext>>1) ^ wbinnext;    // Gray code transform

    if(ASIZE ==1) begin :ac                               // Cacultate the full value
       assign wfull_val   =  (wgraynext == ~wq2_rptr[ASIZE:ASIZE-1]);
       end
    else if(ASIZE >1) begin :ad
       assign wfull_val   =  (wgraynext == {~wq2_rptr[ASIZE:ASIZE-1],wq2_rptr[ASIZE-2:0]});
        end

    assign rcount_w       =  gry2bin(wq2_rptr);         // Transform to binary
    assign wcnt_sub       =  {(rcount_w[ASIZE] ^ wbin[ASIZE]), wbin[ASIZE-1:0]}
                            -{1'b0, rcount_w[ASIZE-1:0]};
// pointer synchronization
      always @(posedge WrClk or posedge WRst)
          if(WRst)
            {wq2_rptr,wq1_rptr} <= 0;
          else
            {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};

      always @(posedge RdClk or posedge RRst)
         if(RRst)
           {rq2_wptr,rq1_wptr} <= 0;
         else
           {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

      always @(posedge RdClk or posedge RRst)
          if(RRst)
            rptr <= 0;
          else
            rptr <= rgraynext;

      always @(posedge WrClk or posedge WRst)
          if(WRst)
            {wbin, wptr} <= 0;
          else
            {wbin, wptr} <= {wbinnext, wgraynext};
end

else if(WDEPTH > RDEPTH) begin: Big  // WDEPTH > RDEPTH,variableas are similar to that when WDEPTH < RDEPTH
     integer                     j ;
     localparam                  b = WDEPTH/RDEPTH ;

  `ifdef EBR_BASED
     reg        [WDSIZE-1:0]     mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
  `endif

  `ifdef DSR_BASED
     reg        [WDSIZE-1:0]     mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "distributed_ram" */;
  `endif
  `ifdef LUT_BASED
     reg        [WDSIZE-1:0]     mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "registers" */;
  `endif

  `ifdef En_Output_Reg
      reg       [RDSIZE-1:0]     wdata_q_r ;
  `endif
      reg       [RDSIZE-1:0]     wdata_q ;
      reg       [RASIZE:0]       wptr ;
      reg       [RASIZE:0]       rptr ;
      reg       [RASIZE:0]       wq2_rptr ;
      reg       [RASIZE:0]       rq2_wptr ;
      reg       [RASIZE:0]       wq1_rptr ;
      reg       [RASIZE:0]       rq1_wptr ;
      reg       [ASIZE:0]        wbin ;
      wire      [RASIZE:0]       rgraynext ;
      wire      [RASIZE:0]       wgraynext ;
      wire      [RASIZE:0]       wcount_r  ;
      wire      [RASIZE:0]       rcount_w  ;
      wire      [ASIZE:0]        rcount_w_1;
      wire      [ASIZE:0]        wbin_num_next ;
      wire      [RASIZE:0]       wbinnext ;
      wire      [RASIZE:0]       wbinnext_1 ;

   always @(posedge WrClk)
       if(WrEn && !Full)
          mem[waddr] <= Data;
`ifdef FWFT
`ifdef En_Output_Reg
    `ifdef Ctrl_By_RdEn
             always @(posedge RdClk  or posedge RRst) begin
                 if(RRst) begin
                    wdata_q_r <= 0;
                   // wdata_q   <= 0;
                    end
                 else if(RdEn ? ~rempty_val : (Empty & !rempty_val)) begin
                    for(j = 0;j < b;j = j + 1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                       //wdata_q <= wdata_q_r;
                    end
             end
             always @(posedge RdClk  or posedge RRst) begin
                 if(RRst)
                    wdata_q   <= 0;
                 else if(RdEn)
                    wdata_q <= wdata_q_r;
    `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                   wdata_q_r <= 0;
                 else  if(RdEn ? ~rempty_val : (Empty & !rempty_val)) begin
                    for(j = 0;j < b;j = j+1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                   end
    `endif
 `else
          always @(posedge RdClk or posedge RRst)
              if(RRst)
                wdata_q <=0;
              else if(RdEn ? ~rempty_val : (Empty & !rempty_val))
                for(j = 0;j < b;j = j+1)
                   wdata_q[((j+1)*WDSIZE-1)-:WDSIZE]<= mem[raddr_num*b+j];
 `endif

 `ifdef En_Output_Reg
     `ifndef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)
                  if(RRst)
                     wdata_q <= 0;
                  else
                     wdata_q <= wdata_q_r;
     `endif
  `endif
`else
`ifdef En_Output_Reg
    `ifdef Ctrl_By_RdEn
             always @(posedge RdClk  or posedge RRst) begin
                 if(RRst) begin
                    wdata_q_r <= 0;
                    wdata_q   <= 0;
                    end
                 else if(RdEn&~Empty) begin
                    for(j = 0;j < b;j = j + 1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                       wdata_q <= wdata_q_r;
                    end
             end
    `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                   wdata_q_r <= 0;
                 else  if(RdEn&~Empty) begin
                    for(j = 0;j < b;j = j+1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                   end
    `endif
 `else
          always @(posedge RdClk or posedge RRst)
              if(RRst)
                wdata_q <=0;
              else if(RdEn&~Empty)
                for(j = 0;j < b;j = j+1)
                   wdata_q[((j+1)*WDSIZE-1)-:WDSIZE]<= mem[raddr_num*b+j];
 `endif

 `ifdef En_Output_Reg
     `ifndef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)
                  if(RRst)
                     wdata_q <= 0;
                  else
                     wdata_q <= wdata_q_r;
     `endif
  `endif
`endif
   assign Q               =  wdata_q;
`ifdef FWFT
   assign raddr_num       =  rbin_num_next[RASIZE-1:0];
`else
   assign raddr_num       =  rbin_num[RASIZE-1:0];
`endif
   assign rbin_num_next   =  rbin_num + (RdEn & ~Empty);
   assign rgraynext       =  (rbin_num_next>>1) ^ rbin_num_next;
   assign rempty_val      =  (rgraynext == rq2_wptr);
   assign wcount_r        =  gry2bin(rq2_wptr);
   assign rcnt_sub        =  {(wcount_r[RASIZE]^rbin_num[RASIZE]),wcount_r[RASIZE-1:0]}
                             -{1'b0,rbin_num[RASIZE-1:0]};

   assign waddr           =  wbin[ASIZE-1:0];
   assign wbin_num_next   =  wbin + (WrEn & ~Full);
   assign wbinnext        =  wbin_num_next/b;      //write to read
   assign wbinnext_1      =  wbin/b;
   assign wgraynext       =  (wbinnext>>1) ^ wbinnext;
   if (RASIZE == 1) begin :ae
      assign wfull_val    =  (wgraynext == ~wq2_rptr[RASIZE:RASIZE-1]);
      end
   else if (RASIZE >1) begin :af
      assign wfull_val    =  (wgraynext == {~wq2_rptr[RASIZE:RASIZE-1],wq2_rptr[RASIZE-2:0]});
      end
   assign rcount_w        =  gry2bin(wq2_rptr);
   assign rcount_w_1      =  gry2bin(wq2_rptr)*b;
   assign wcnt_sub        =  {(rcount_w[RASIZE] ^ wbinnext_1[RASIZE]), wbin[ASIZE-1:0]}
                             -{1'b0, rcount_w_1[ASIZE-1:0]};

       always @(posedge WrClk or posedge WRst)
           if(WRst)
             {wq2_rptr,wq1_rptr} <= 0;
           else
             {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};

      always @(posedge RdClk or posedge RRst)
          if(RRst)
            {rq2_wptr,rq1_wptr} <= 0;
          else
          {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

      always @(posedge RdClk or posedge RRst)
          if(RRst)
            rptr<= 0;
          else
            rptr <= rgraynext;

      always @(posedge WrClk or posedge WRst)
          if(WRst)
             {wbin, wptr} <= 0;
           else
             {wbin, wptr} <= {wbin_num_next, wgraynext};

end
else if (WDEPTH == RDEPTH) begin :  Equal
  `ifdef En_ECC
         wire      [WDSIZE+PWIDTH-1:0]        Qreg   ;
         wire      [WDSIZE+PWIDTH-1:0]        data   ;
         wire      [WDSIZE-1:0]               Data_p ;
         wire      [PWIDTH-1:0]               P_out  ;
         wire      [WDSIZE-1:0]               Din    ;
         wire      [PWIDTH-1:0]               P_in   ;

      `getname(edc,`module_name) #( .DSIZE(WDSIZE)  )
         u_edc (
                   .WrClk                    ( WrClk )   ,
                   .RdClk                    ( RdClk )   ,
               `ifdef En_Reset
                   `ifdef Reset_Synchronization
                   .RST                      ( Reset )   ,
                   `else
                   .Reset                    ( WRst ) ,
                   .RPReset                  ( RRst ) ,
                   `endif
                `endif
                   .Ein                      ( Data )    ,
                   .Eout                     ( Data_p )  ,
                   .P_out                    ( P_out )   ,
                   .Din                      ( Din )     ,
                   .Dout                     ( Q )       ,
                   .P_in                     ( P_in )    ,
                  `ifdef Enable_force_error
                   .force_error              ( 2'b00 )   ,
                  `endif
                   .error                    ( ERROR )
              );
       assign  data  =   {P_out, Data_p};
       assign  Din   =   Qreg[WDSIZE-1:0];
       assign  P_in  =   Qreg[WDSIZE+PWIDTH-1:WDSIZE];
   `endif

   `ifdef EBR_BASED
      `ifdef En_ECC
          reg       [WDSIZE+PWIDTH-1:0]    mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
      `else
          reg       [WDSIZE-1:0]           mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
      `endif
   `endif
   `ifdef DSR_BASED
          reg      [WDSIZE-1:0]            mem[0:(WDEPTH-1)]  /* synthesis syn_ramstyle= "distributed_ram" */;
   `endif
   `ifdef LUT_BASED
          reg      [WDSIZE-1:0]            mem[0:(WDEPTH-1)]  /* synthesis syn_ramstyle= "registers" */;
   `endif

   `ifdef  En_Output_Reg
      `ifdef En_ECC
          reg       [WDSIZE+PWIDTH-1:0]    wdata_q_r ;
      `else
          reg       [WDSIZE-1:0]           wdata_q_r ;
      `endif
   `endif
   `ifdef En_ECC
          reg       [WDSIZE+PWIDTH-1:0]    wdata_q ;
    `else
          reg       [WDSIZE-1:0]           wdata_q ;
   `endif
          reg       [ASIZE:0]              wptr ;
          reg       [ASIZE:0]              rptr ;
          reg       [ASIZE:0]              wq2_rptr ;
          reg       [ASIZE:0]              rq2_wptr ;
          reg       [ASIZE:0]              wq1_rptr ;
          reg       [ASIZE:0]              rq1_wptr;

          reg       [ASIZE:0]              wbin ;
          wire      [ASIZE:0]              rgraynext ;
          wire      [ASIZE:0]              wgraynext ;
          wire      [ASIZE:0]              wcount_r ;
          wire      [ASIZE:0]              rcount_w ;
          wire      [ASIZE:0]              wbinnext ;

  always @(posedge WrClk)
      if(WrEn && !Full)
        `ifdef En_ECC
           mem[waddr] <= data;
        `else
           mem[waddr] <= Data;
        `endif
`ifdef FWFT
`ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)begin
                  if(RRst) begin
                     wdata_q_r <= 0;
                     //wdata_q   <= 0;
                     end
                  else  if(RdEn ? ~rempty_val : (Empty & !rempty_val)) begin
                     wdata_q_r <= mem[raddr_num];
                    // wdata_q   <= wdata_q_r;
                  end
              end
             always @(posedge RdClk or posedge RRst)begin
                  if(RRst)
                      wdata_q   <= 0;
                  else if(RdEn)
                      wdata_q   <= wdata_q_r;
   `else
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                   wdata_q_r <= 0;
                else  if(RdEn ? ~rempty_val : (Empty & !rempty_val))
                   wdata_q_r <= mem[raddr_num];
   `endif
`else
         always @(posedge RdClk or posedge RRst)
                if(RRst)
                   wdata_q <= 0;
               // else if(~Empty)
                else if (RdEn ? ~rempty_val : (Empty & !rempty_val))
                   wdata_q <= mem[raddr_num];
`endif

`ifdef En_Output_Reg
  `ifndef Ctrl_By_RdEn
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                  wdata_q <=0;
                else
                  wdata_q <= wdata_q_r;
  `endif
`endif
`else
 `ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)begin
                  if(RRst) begin
                     wdata_q_r <= 0;
                     wdata_q   <= 0;
                     end
                  else  if(RdEn&~Empty) begin
                     wdata_q_r <= mem[raddr_num];
                     wdata_q   <= wdata_q_r;
                  end
              end
   `else
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                   wdata_q_r <= 0;
                else  if(RdEn&~Empty)
                   wdata_q_r <= mem[raddr_num];
   `endif
`else
         always @(posedge RdClk or posedge RRst)
                if(RRst)
                   wdata_q <= 0;
                else if(RdEn&~Empty)
                   wdata_q <= mem[raddr_num];
`endif

`ifdef En_Output_Reg
  `ifndef Ctrl_By_RdEn
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                  wdata_q <=0;
                else
                  wdata_q <= wdata_q_r;
  `endif
`endif
`endif
`ifdef En_ECC
   assign Qreg            =  wdata_q;
`else
   assign Q               =  wdata_q;
`endif
`ifdef FWFT
   assign raddr_num       =  rbin_num_next[ASIZE-1:0];
`else
   assign raddr_num       =  rbin_num[ASIZE-1:0];
`endif
   assign rbin_num_next   =  rbin_num + (RdEn & ~Empty);
   assign rgraynext       =  (rbin_num_next>>1) ^ rbin_num_next;
   assign rempty_val      =  (rgraynext == rq2_wptr);
   assign wcount_r        =  gry2bin(rq2_wptr);
   assign rcnt_sub        =  {(wcount_r[ASIZE] ^ rbin_num[RASIZE]),wcount_r[ASIZE-1:0]}-{1'b0, rbin_num[RASIZE-1:0]};
   assign waddr           =  wbin[ASIZE-1:0];
   assign wbinnext        =  wbin + (WrEn & ~Full);
   assign wgraynext       =  (wbinnext>>1) ^ wbinnext;

   if (ASIZE == 1) begin :ag
      assign wfull_val    =  (wgraynext == ~wq2_rptr[ASIZE:ASIZE-1]);
      end
   else if (ASIZE > 1) begin :ah
        assign wfull_val  =  (wgraynext == {~wq2_rptr[ASIZE:ASIZE-1],wq2_rptr[ASIZE-2:0]});
   end

   assign rcount_w        =  gry2bin(wq2_rptr);
   assign wcnt_sub        =  {(rcount_w[ASIZE] ^ wbin[ASIZE]), wbin[ASIZE-1:0]} - {1'b0, rcount_w[ASIZE-1:0]};

       always @(posedge WrClk or posedge WRst)
           if(WRst)
             {wq2_rptr,wq1_rptr}<=0;
           else
             {wq2_rptr,wq1_rptr}<={wq1_rptr,rptr};

       always @(posedge RdClk or posedge RRst)
           if(RRst)
             {rq2_wptr,rq1_wptr} <= 0;
           else
             {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

       always @(posedge RdClk or posedge RRst)
           if(RRst)
              rptr<= 0;
           else
              rptr <= rgraynext;

      always @(posedge WrClk or posedge WRst)
           if(WRst)
             {wbin, wptr} <= 0;
           else
             {wbin, wptr} <= {wbinnext, wgraynext};
end
endgenerate

       always @(posedge RdClk or posedge RRst)
           if(RRst)
              Empty <= 1'b1;
           else
              Empty <= rempty_val;

       always @(posedge WrClk or posedge WRst)
           if(WRst)
             Full <= 1'b0;
           else
             Full <= wfull_val;

`ifdef Al_Empty_Flag
  `ifdef Empty_S_Dual_Th
        assign arempty_val    = (rcnt_sub <= DeassertEmptyTh)|(( rcnt_sub == DeassertEmptyTh + 1'b1) & RdEn);
  `endif
  `ifdef  Empty_S_Single_Th
        assign arempty_val    = (rcnt_sub <= AEMPT)|((rcnt_sub == AEMPT + 1'b1) & RdEn);
  `endif
  `ifdef  Empty_D_Dual_Th
        assign arempty_val    = (rcnt_sub <= AlmostEmptyClrTh)|((rcnt_sub ==  AlmostEmptyClrTh + 1'b1) & RdEn);
  `endif
  `ifdef Empty_D_Single_Th
        assign arempty_val    = (rcnt_sub <= AlmostEmptyTh)|((rcnt_sub == AlmostEmptyTh + 1'b1) & RdEn);
  `endif
`endif

`ifdef Al_Full_Flag
  `ifdef Full_S_Dual_Th
        assign awfull_val    = (wcnt_sub >= DeassertFullTh)|((wcnt_sub == DeassertFullTh - 1'b1) & WrEn);
  `endif
  `ifdef  Full_S_Single_Th
        assign awfull_val    = (wcnt_sub >= AFULL)|((wcnt_sub == AFULL - 1'b1) & WrEn);
  `endif
  `ifdef  Full_D_Dual_Th
        assign awfull_val    = (wcnt_sub >= AlmostFullClrTh)|((wcnt_sub == AlmostFullClrTh - 1'b1) & WrEn);;
  `endif
  `ifdef  Full_D_Single_Th
        assign awfull_val    = (wcnt_sub >= AlmostFullTh)|((wcnt_sub == AlmostFullTh - 1'b1) & WrEn);
  `endif
`endif

`ifdef Al_Empty_Flag
       always @(posedge RdClk or posedge RRst)begin
           if(RRst)
              Almost_Empty <= 1'b1;
           else
        `ifdef Empty_S_Dual_Th
           if((arempty_val == 1)&&((rcnt_sub <= AssertEmptyTh)|(( rcnt_sub == AssertEmptyTh + 1'b1) & RdEn)))
               Almost_Empty <= 1'b1;
           else if (arempty_val == 0)
                   Almost_Empty <= 1'b0;
        `endif
        `ifdef Empty_S_Single_Th
              Almost_Empty <= arempty_val;
        `endif
        `ifdef Empty_D_Dual_Th
           if((arempty_val == 1)&&((rcnt_sub <= AlmostEmptySetTh)|(( rcnt_sub == AlmostEmptySetTh + 1'b1)&RdEn)))
               Almost_Empty <= 1'b1;
           else if (arempty_val == 0)
                   Almost_Empty <= 1'b0;
        `endif
        `ifdef Empty_D_Single_Th
              Almost_Empty <= arempty_val;
        `endif
  end
`endif

 `ifdef Al_Full_Flag
       always @(posedge WrClk or posedge WRst)begin
           if (WRst)
              Almost_Full <= 1'b0;
           else
         `ifdef Full_S_Dual_Th
            if((awfull_val == 1)&&((wcnt_sub >= AssertFullTh)|((wcnt_sub == AssertFullTh - 1'b1) & WrEn)))
                Almost_Full <= 1'b1;
            else if(awfull_val == 0)
                   Almost_Full <= 1'b0;
         `endif
         `ifdef  Full_S_Single_Th
                Almost_Full <= awfull_val;
         `endif
         `ifdef Full_D_Dual_Th
            if((awfull_val == 1)&&((wcnt_sub >= AlmostFullSetTh)|((wcnt_sub == AlmostFullSetTh - 1'b1) & WrEn)))
                Almost_Full <= 1'b1;
            else if(awfull_val == 0)
                   Almost_Full <= 1'b0;
         `endif
         `ifdef Full_D_Single_Th
                Almost_Full <= awfull_val;
         `endif
   end
`endif

`ifdef Count_W
         always @(posedge WrClk or posedge WRst)
             if(WRst)
                Wnum <= 0;
             else
                Wnum <= wcnt_sub;
`endif

`ifdef Count_R
          always @(posedge RdClk or posedge RRst)
             if(RRst)
               Rnum <= 0;
             else
               Rnum <= rcnt_sub;
`endif

function [ASIZE:0]gry2bin;
     input [ASIZE:0] gry_code;
     integer         i;
      begin
        gry2bin[ASIZE]=gry_code[ASIZE];
     for(i=ASIZE-1;i>=0;i=i-1)
        gry2bin[i]=gry2bin[i+1]^gry_code[i];
     end
  endfunction
endmodule


 // `define module_name  fifo_8to4
//  `define getname(oriName,tmodule_name) \~oriName.tmodule_name
   `define     EBR_BASED
   `define     En_Reset
   `define     Reset_Synchronization

`timescale 1ns/1ps
 module `getname(fifo_8to8,`module_name) (
          Data                ,
  `ifdef  En_Reset
     `ifdef Reset_Synchronization
          Reset               ,
     `else
          WrReset             ,
          RdReset             ,
     `endif
  `endif
          WrClk               ,
          RdClk               ,
          WrEn                ,
          RdEn                ,
`ifdef    Al_Empty_Flag
   `ifdef Empty_D_Dual_Th
          AlmostEmptySetTh    ,
          AlmostEmptyClrTh    ,
   `endif
   `ifdef Empty_D_Single_Th
          AlmostEmptyTh       ,
   `endif
`endif

`ifdef    Al_Full_Flag
   `ifdef Full_D_Dual_Th
          AlmostFullSetTh     ,
          AlmostFullClrTh     ,
  `endif
  `ifdef  Full_D_Single_Th
          AlmostFullTh        ,
  `endif
`endif

`ifdef    Count_W
          Wnum                ,
`endif
`ifdef    Count_R
          Rnum                ,
`endif
`ifdef    Al_Empty_Flag
          Almost_Empty        ,
`endif
`ifdef    Al_Full_Flag
          Almost_Full         ,
`endif
`ifdef En_ECC
          ERROR               ,
`endif
          Q                   ,
          Empty               ,
          Full
);

parameter   WDEPTH           = 32;        //when synplify,use these three parameters
parameter   WDSIZE           = 8;
parameter   RDEPTH           = 32;
parameter   ASIZE            =  $clog2(WDEPTH);
parameter   RASIZE           =  $clog2(RDEPTH);
parameter   RDSIZE           =  WDEPTH*WDSIZE/RDEPTH;  // must be divisible, the remainder is 0;
       input           [WDSIZE-1:0]  Data  ;    // Data writed into the FIFO
       input                         WrClk ;    // Write Clock
       input                         RdClk ;    // Read  Clock
       input                         WrEn  ;    // Write Enable
       input                         RdEn  ;    // Read  Enable
`ifdef En_Reset
    `ifdef Reset_Synchronization
       input                         Reset ;    //Reset Synchronization : only Reset
    `else
       input                         WrReset ;  // Two Reset :Write Reset, Read Reset
       input                         RdReset ;
     `endif
`endif

`ifdef Al_Empty_Flag
    `ifdef Empty_D_Dual_Th
       input          [RASIZE-1:0]   AlmostEmptySetTh ;   // Dynamic input threshold of almost empty set to 1
       input          [RASIZE-1:0]   AlmostEmptyClrTh ;   // Dynamic input threshold of almost empty set to 0
    `endif
    `ifdef Empty_D_Single_Th
       input          [RASIZE-1:0]   AlmostEmptyTh    ;   //Dynamic input threshold of  almost empty set to 1
    `endif
`endif

`ifdef Al_Full_Flag
   `ifdef Full_D_Dual_Th
       input          [ASIZE-1:0]    AlmostFullSetTh ;   // Dynamic input threshold of almost full set to 1
       input          [ASIZE-1:0]    AlmostFullClrTh ;   // Dynamic input threshold of almost full set to 0

   `endif
   `ifdef Full_D_Single_Th
       input          [ASIZE-1:0]    AlmostFullTh    ;   //Dynamic input threshold of  almost full set to 1
   `endif
`endif

`ifdef Count_W
       output         [ASIZE:0]      Wnum ;              // Write data count: synchronized to WrClk;
`endif
`ifdef Count_R
       output         [RASIZE:0]     Rnum ;              // Read data count: synchronized to RdCLK;
`endif

`ifdef   Al_Empty_Flag
       output                        Almost_Empty ;      // Flag of Almost empty
`endif
`ifdef   Al_Full_Flag
       output                        Almost_Full  ;      // Flag of Almost full
`endif
`ifdef En_ECC
       output         [1:0]          ERROR        ;
`endif

       output         [RDSIZE-1:0]   Q     ;             // Data read from the fifo
       output                        Empty ;             // Empty flag
       output                        Full  ;             // Full flag

`getname(fifo_4to8_sub,`module_name)  fifo_4to8_sub_inst (
          .Data                  (Data)                ,
  `ifdef  En_Reset
     `ifdef Reset_Synchronization
          .Reset                 (Reset)               ,
     `else
          .WrReset               (WrReset)             ,
          .RdReset               (RdReset)             ,
     `endif
  `endif
          .WrClk                 (WrClk)               ,
          .RdClk                 (RdClk)               ,
          .WrEn                  (WrEn)                ,
          .RdEn                  (RdEn)                ,
`ifdef    Al_Empty_Flag
   `ifdef Empty_D_Dual_Th
          .AlmostEmptySetTh      (AlmostEmptySetTh)    ,
          .AlmostEmptyClrTh      (AlmostEmptyClrTh)    ,
   `endif
   `ifdef Empty_D_Single_Th
          .AlmostEmptyTh         (AlmostEmptyTh)       ,
   `endif
`endif

`ifdef    Al_Full_Flag
   `ifdef Full_D_Dual_Th
          .AlmostFullSetTh       (AlmostFullSetTh)     ,
          .AlmostFullClrTh       (AlmostFullClrTh)     ,
  `endif
  `ifdef  Full_D_Single_Th
          .AlmostFullTh          (AlmostFullTh)        ,
  `endif
`endif

`ifdef    Count_W
          .Wnum                  (Wnum)                ,
`endif
`ifdef    Count_R
          .Rnum                  (Rnum)                ,
`endif
`ifdef    Al_Empty_Flag
          .Almost_Empty          (Almost_Empty)        ,
`endif
`ifdef    Al_Full_Flag
          .Almost_Full           (Almost_Full)         ,
`endif
`ifdef En_ECC
          .ERROR                 (ERROR)               ,
`endif
          .Q                     (Q)                   ,
          .Empty                 (Empty)               ,
          .Full                  (Full)
    );

endmodule

//`include "fifo_define.v"
`timescale 1ns/1ps
module `getname(fifo_4to8_sub,`module_name) (
          Data                ,
  `ifdef  En_Reset
     `ifdef Reset_Synchronization
          Reset               ,
     `else
          WrReset             ,
          RdReset             ,
     `endif
  `endif
          WrClk               ,
          RdClk               ,
          WrEn                ,
          RdEn                ,
`ifdef    Al_Empty_Flag
   `ifdef Empty_D_Dual_Th
          AlmostEmptySetTh    ,
          AlmostEmptyClrTh    ,
   `endif
   `ifdef Empty_D_Single_Th
          AlmostEmptyTh       ,
   `endif
`endif

`ifdef    Al_Full_Flag
   `ifdef Full_D_Dual_Th
          AlmostFullSetTh     ,
          AlmostFullClrTh     ,
  `endif
  `ifdef  Full_D_Single_Th
          AlmostFullTh        ,
  `endif
`endif

`ifdef    Count_W
          Wnum                ,
`endif
`ifdef    Count_R
          Rnum                ,
`endif
`ifdef    Al_Empty_Flag
          Almost_Empty        ,
`endif
`ifdef    Al_Full_Flag
          Almost_Full         ,
`endif
`ifdef En_ECC
          ERROR               ,
`endif
          Q                   ,
          Empty               ,
          Full
);
    //  `include "fifo_parameter.v"
parameter   WDEPTH           = 32;        //when synplify,use these three parameters
parameter   WDSIZE           = 8;
parameter   RDEPTH           = 32;
parameter   ASIZE            =  $clog2(WDEPTH);
parameter   RASIZE           =  $clog2(RDEPTH);
parameter   RDSIZE           =  WDEPTH*WDSIZE/RDEPTH;  // must be divisible, the remainder is 0;
       input           [WDSIZE-1:0]  Data  ;    // Data writed into the FIFO
       input                         WrClk ;    // Write Clock
       input                         RdClk ;    // Read  Clock
       input                         WrEn  ;    // Write Enable
       input                         RdEn  ;    // Read  Enable
`ifdef En_Reset
    `ifdef Reset_Synchronization
       input                         Reset;      //Reset Synchronization : only Reset
    `else
       input                         WrReset ;   // Two Reset :Write Reset, Read Reset
       input                         RdReset ;
     `endif
`endif

`ifdef Al_Empty_Flag
    `ifdef Empty_D_Dual_Th
       input          [RASIZE-1:0]     AlmostEmptySetTh ;   // Dynamic input threshold of almost empty set to 1
       input          [RASIZE-1:0]     AlmostEmptyClrTh ;   // Dynamic input threshold of almost empty set to 0
    `endif
    `ifdef Empty_D_Single_Th
       input          [RASIZE-1:0]     AlmostEmptyTh    ;   //Dynamic input threshold of  almost empty set to 1
    `endif
`endif

`ifdef Al_Full_Flag
   `ifdef Full_D_Dual_Th
       input          [ASIZE-1:0]      AlmostFullSetTh ;   // Dynamic input threshold of almost full set to 1
       input          [ASIZE-1:0]      AlmostFullClrTh ;   // Dynamic input threshold of almost full set to 0

  `endif
  `ifdef Full_D_Single_Th
       input          [ASIZE-1:0]      AlmostFullTh    ;   //Dynamic input threshold of  almost full set to 1
  `endif
`endif

`ifdef Count_W
       output  reg    [ASIZE:0]      Wnum ;              // Write data count: synchronized to WrClk;
`endif
`ifdef Count_R
       output  reg    [RASIZE:0]     Rnum ;              // Read data count: synchronized to RdCLK;
`endif

`ifdef   Al_Empty_Flag
       output  reg                   Almost_Empty ;      // Flag of Almost empty
`endif
`ifdef   Al_Full_Flag
       output  reg                   Almost_Full  ;      // Flag of Almost full
`endif
`ifdef En_ECC
       output         [1:0]          ERROR        ;
`endif

       output         [RDSIZE-1:0]   Q     ;             // Data read from the fifo
       output  reg                   Empty ;             // Empty flag
       output  reg                   Full  ;             // Full flag

       reg            [RASIZE:0]     rbin_num      ;     // Read pointer : binary,1-bit bigger than raddr_num inoder to obtain the right flag singal
       wire           [RASIZE-1:0]   raddr_num     ;     // Read address
       wire           [RASIZE:0]     rbin_num_next ;
       wire           [RASIZE:0]     rcnt_sub      ;     // Read data count
       wire           [ASIZE-1:0]    waddr         ;     // Write address
       wire                          rempty_val    ;     // Empty value
       wire                          wfull_val     ;     // Full  value
       wire                          arempty_val   ;     // Almost empty value
       wire                          awfull_val    ;     // Almost full  value
       wire           [ASIZE:0]      wcnt_sub      ;     // Write data count
wire WRst,RRst;
 `ifdef En_ECC
  localparam   PWIDTH =  (WDSIZE == 1 ) ? 3 :
                             (WDSIZE >= 2  && WDSIZE <= 4 ) ? 4 :
                             (WDSIZE >= 5  && WDSIZE <= 11) ? 5 :
                             (WDSIZE >= 12 && WDSIZE <= 26) ? 6 :
                             (WDSIZE >= 27 && WDSIZE <= 57) ? 7 :
                             (WDSIZE >= 58 && WDSIZE <= 64) ? 8 :0;

  `else
       localparam PWIDTH = 0;
 `endif
/******************************************************/
// The read and write logic for differnt depth
/******************************************************/

`ifdef En_Reset
    `ifdef Reset_Synchronization
     reg  [1:0] reset_r;
     reg  [1:0] reset_w;
     always @(negedge RdClk or posedge Reset)
         if(Reset)
           reset_r <= 2'b11;
         else
           reset_r <= {reset_r[0],1'b0};
     assign RRst = reset_r[1];
     always @(negedge WrClk or posedge Reset)
         if(Reset)
           reset_w <= 2'b11;
         else
           reset_w <= {reset_w[0],1'b0};
     assign WRst = reset_w[1];
   `else
     assign RRst = RdReset;
     assign WRst = WrReset;
  `endif
`else
     assign RRst = 0;
     assign WRst = 0;
`endif

     always @(posedge RdClk or posedge RRst)
           if (RRst)
              rbin_num <= 0;
           else
              rbin_num <= rbin_num_next;

generate
  if(WDEPTH < RDEPTH) begin:  Small // WDEPTH < RDEPTH
     localparam                  a = RDEPTH/WDEPTH ;
   `ifdef EBR_BASED
     reg       [WDSIZE-1:0]      mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
   `endif

   `ifdef DSR_BASED
     reg       [WDSIZE-1:0]      mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "distributed_ram" */;
   `endif

   `ifdef LUT_BASED
     reg       [WDSIZE-1:0]      mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "registers" */;
   `endif

  `ifdef En_Output_Reg
     reg       [RDSIZE-1:0]      wdata_q_r ;
  `endif
     reg       [WDSIZE-1:0]      wdata;         // Data from the fifo
     reg       [ASIZE:0]         wptr ;         // Write pointer
     reg       [ASIZE:0]         rptr ;         // Read  pointer
     reg       [ASIZE:0]         wq2_rptr ;     // Read  pointer synchronized to WrClk
     reg       [ASIZE:0]         rq2_wptr ;     // Write pointer synchronized to RdCkl
     reg       [ASIZE:0]         wq1_rptr ;
     reg       [ASIZE:0]         rq1_wptr ;
     reg       [ASIZE:0]         wbin     ;     // Write pointer:binary, 1-bit bigger than waddr in order to obta
                                                // in the right flag signal
     wire      [RASIZE:0]        wcount_r_1 ;
     wire      [ASIZE:0]         rgraynext  ;   // Read pointer: gray code
     wire      [ASIZE:0]         rbinnext   ;
     wire      [ASIZE:0]         rbinnext_1 ;
     wire      [ASIZE:0]         wgraynext  ;
     wire      [ASIZE:0]         wcount_r   ;
     wire      [ASIZE:0]         rcount_w   ;
     wire      [ASIZE:0]         wbinnext   ;
     wire      [RDSIZE-1:0]      wdata_q    ;

   always @(posedge WrClk)    // Write data into fifo
       if(WrEn && !Full)
          mem[waddr] <= Data;
`ifdef FWFT
// Read data from fifo
      always @(posedge RdClk or posedge RRst )
          if(RRst)
             wdata <= 0;
          else  if(RdEn ? ~rempty_val : (Empty & !rempty_val))
             wdata <= mem[raddr_num/a];

  assign wdata_q = RdEn ? wdata[(((raddr_num-1)%a+1)*RDSIZE-1)-:RDSIZE] :  wdata[(((raddr_num)%a+1)*RDSIZE-1)-:RDSIZE];
`else
// Read data from fifo
      always @(posedge RdClk or posedge RRst )
          if(RRst)
             wdata <= 0;
          else  if(RdEn && !Empty)
             wdata <= mem[raddr_num/a];

  assign wdata_q = wdata[(((raddr_num-1)%a+1)*RDSIZE-1)-:RDSIZE];
`endif
// if define the Enable output registers,the data will be output one clock latar
`ifdef FWFT
`ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
             always @(posedge RdClk or posedge RRst )
                 if(RRst)
                    wdata_q_r  <= 0;
                 else  if(RdEn)
                         wdata_q_r <= wdata_q;
   `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                    wdata_q_r <= 0;
                 else
                    wdata_q_r <= wdata_q;
    `endif
`endif
`else
`ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
             always @(posedge RdClk or posedge RRst )
                 if(RRst)
                    wdata_q_r  <= 0;
                 else  if(RdEn & ~Empty)
                         wdata_q_r <= wdata_q;
   `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                    wdata_q_r <= 0;
                 else
                    wdata_q_r <= wdata_q;
    `endif
`endif
`endif

 `ifdef En_Output_Reg
    assign Q              =  wdata_q_r;
   `else
    assign Q              =  wdata_q;
 `endif
`ifdef FWFT
    assign raddr_num      =  rbin_num_next[RASIZE-1:0];        // Read address
`else
    assign raddr_num      =  rbin_num[RASIZE-1:0];        // Read address
`endif
    assign rbin_num_next  =  rbin_num + (RdEn & ~Empty);  // Obtain the next read pointer
    assign rbinnext       =  rbin_num_next[RASIZE:0]/a;   // Read address transform because the different depth
    assign rbinnext_1     =  rbin_num/a;
    assign rgraynext      =  (rbinnext>>1) ^ rbinnext;    // Gray code transform
    assign rempty_val     =  (rgraynext == rq2_wptr);     // Jude empty value
    assign wcount_r       =  gry2bin(rq2_wptr);
    assign wcount_r_1     =  gry2bin(rq2_wptr)*a;         // Write address transform
    assign rcnt_sub       =  {(wcount_r[ASIZE]^rbinnext_1[ASIZE]),wcount_r_1[RASIZE-1:0]}
                             -{1'b0,rbin_num[RASIZE-1:0]}; // Caculate the read data count

    assign waddr          =  wbin[ASIZE-1:0];             // Write address
    assign wbinnext       =  wbin + (WrEn & ~Full);
    assign wgraynext      =  (wbinnext>>1) ^ wbinnext;    // Gray code transform

    if(ASIZE ==1) begin :ac                               // Cacultate the full value
       assign wfull_val   =  (wgraynext == ~wq2_rptr[ASIZE:ASIZE-1]);
       end
    else if(ASIZE >1) begin :ad
       assign wfull_val   =  (wgraynext == {~wq2_rptr[ASIZE:ASIZE-1],wq2_rptr[ASIZE-2:0]});
        end

    assign rcount_w       =  gry2bin(wq2_rptr);         // Transform to binary
    assign wcnt_sub       =  {(rcount_w[ASIZE] ^ wbin[ASIZE]), wbin[ASIZE-1:0]}
                            -{1'b0, rcount_w[ASIZE-1:0]};
// pointer synchronization
      always @(posedge WrClk or posedge WRst)
          if(WRst)
            {wq2_rptr,wq1_rptr} <= 0;
          else
            {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};

      always @(posedge RdClk or posedge RRst)
         if(RRst)
           {rq2_wptr,rq1_wptr} <= 0;
         else
           {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

      always @(posedge RdClk or posedge RRst)
          if(RRst)
            rptr <= 0;
          else
            rptr <= rgraynext;

      always @(posedge WrClk or posedge WRst)
          if(WRst)
            {wbin, wptr} <= 0;
          else
            {wbin, wptr} <= {wbinnext, wgraynext};
end

else if(WDEPTH > RDEPTH) begin: Big  // WDEPTH > RDEPTH,variableas are similar to that when WDEPTH < RDEPTH
     integer                     j ;
     localparam                  b = WDEPTH/RDEPTH ;

  `ifdef EBR_BASED
     reg        [WDSIZE-1:0]     mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
  `endif

  `ifdef DSR_BASED
     reg        [WDSIZE-1:0]     mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "distributed_ram" */;
  `endif
  `ifdef LUT_BASED
     reg        [WDSIZE-1:0]     mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "registers" */;
  `endif

  `ifdef En_Output_Reg
      reg       [RDSIZE-1:0]     wdata_q_r ;
  `endif
      reg       [RDSIZE-1:0]     wdata_q ;
      reg       [RASIZE:0]       wptr ;
      reg       [RASIZE:0]       rptr ;
      reg       [RASIZE:0]       wq2_rptr ;
      reg       [RASIZE:0]       rq2_wptr ;
      reg       [RASIZE:0]       wq1_rptr ;
      reg       [RASIZE:0]       rq1_wptr ;
      reg       [ASIZE:0]        wbin ;
      wire      [RASIZE:0]       rgraynext ;
      wire      [RASIZE:0]       wgraynext ;
      wire      [RASIZE:0]       wcount_r  ;
      wire      [RASIZE:0]       rcount_w  ;
      wire      [ASIZE:0]        rcount_w_1;
      wire      [ASIZE:0]        wbin_num_next ;
      wire      [RASIZE:0]       wbinnext ;
      wire      [RASIZE:0]       wbinnext_1 ;

   always @(posedge WrClk)
       if(WrEn && !Full)
          mem[waddr] <= Data;
`ifdef FWFT
`ifdef En_Output_Reg
    `ifdef Ctrl_By_RdEn
             always @(posedge RdClk  or posedge RRst) begin
                 if(RRst) begin
                    wdata_q_r <= 0;
                   // wdata_q   <= 0;
                    end
                 else if(RdEn ? ~rempty_val : (Empty & !rempty_val)) begin
                    for(j = 0;j < b;j = j + 1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                       //wdata_q <= wdata_q_r;
                    end
             end
             always @(posedge RdClk  or posedge RRst) begin
                 if(RRst)
                    wdata_q   <= 0;
                 else if(RdEn)
                    wdata_q <= wdata_q_r;
    `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                   wdata_q_r <= 0;
                 else  if(RdEn ? ~rempty_val : (Empty & !rempty_val)) begin
                    for(j = 0;j < b;j = j+1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                   end
    `endif
 `else
          always @(posedge RdClk or posedge RRst)
              if(RRst)
                wdata_q <=0;
              else if(RdEn ? ~rempty_val : (Empty & !rempty_val))
                for(j = 0;j < b;j = j+1)
                   wdata_q[((j+1)*WDSIZE-1)-:WDSIZE]<= mem[raddr_num*b+j];
 `endif

 `ifdef En_Output_Reg
     `ifndef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)
                  if(RRst)
                     wdata_q <= 0;
                  else
                     wdata_q <= wdata_q_r;
     `endif
  `endif
`else
`ifdef En_Output_Reg
    `ifdef Ctrl_By_RdEn
             always @(posedge RdClk  or posedge RRst) begin
                 if(RRst) begin
                    wdata_q_r <= 0;
                    wdata_q   <= 0;
                    end
                 else if(RdEn&~Empty) begin
                    for(j = 0;j < b;j = j + 1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                       wdata_q <= wdata_q_r;
                    end
             end
    `else
             always @(posedge RdClk or posedge RRst)
                 if(RRst)
                   wdata_q_r <= 0;
                 else  if(RdEn&~Empty) begin
                    for(j = 0;j < b;j = j+1)
                       wdata_q_r[((j+1)*WDSIZE-1)-:WDSIZE] <= mem[raddr_num*b+j];
                   end
    `endif
 `else
          always @(posedge RdClk or posedge RRst)
              if(RRst)
                wdata_q <=0;
              else if(RdEn&~Empty)
                for(j = 0;j < b;j = j+1)
                   wdata_q[((j+1)*WDSIZE-1)-:WDSIZE]<= mem[raddr_num*b+j];
 `endif

 `ifdef En_Output_Reg
     `ifndef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)
                  if(RRst)
                     wdata_q <= 0;
                  else
                     wdata_q <= wdata_q_r;
     `endif
  `endif
`endif
   assign Q               =  wdata_q;
`ifdef FWFT
   assign raddr_num       =  rbin_num_next[RASIZE-1:0];
`else
   assign raddr_num       =  rbin_num[RASIZE-1:0];
`endif
   assign rbin_num_next   =  rbin_num + (RdEn & ~Empty);
   assign rgraynext       =  (rbin_num_next>>1) ^ rbin_num_next;
   assign rempty_val      =  (rgraynext == rq2_wptr);
   assign wcount_r        =  gry2bin(rq2_wptr);
   assign rcnt_sub        =  {(wcount_r[RASIZE]^rbin_num[RASIZE]),wcount_r[RASIZE-1:0]}
                             -{1'b0,rbin_num[RASIZE-1:0]};

   assign waddr           =  wbin[ASIZE-1:0];
   assign wbin_num_next   =  wbin + (WrEn & ~Full);
   assign wbinnext        =  wbin_num_next/b;      //write to read
   assign wbinnext_1      =  wbin/b;
   assign wgraynext       =  (wbinnext>>1) ^ wbinnext;
   if (RASIZE == 1) begin :ae
      assign wfull_val    =  (wgraynext == ~wq2_rptr[RASIZE:RASIZE-1]);
      end
   else if (RASIZE >1) begin :af
      assign wfull_val    =  (wgraynext == {~wq2_rptr[RASIZE:RASIZE-1],wq2_rptr[RASIZE-2:0]});
      end
   assign rcount_w        =  gry2bin(wq2_rptr);
   assign rcount_w_1      =  gry2bin(wq2_rptr)*b;
   assign wcnt_sub        =  {(rcount_w[RASIZE] ^ wbinnext_1[RASIZE]), wbin[ASIZE-1:0]}
                             -{1'b0, rcount_w_1[ASIZE-1:0]};

       always @(posedge WrClk or posedge WRst)
           if(WRst)
             {wq2_rptr,wq1_rptr} <= 0;
           else
             {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};

      always @(posedge RdClk or posedge RRst)
          if(RRst)
            {rq2_wptr,rq1_wptr} <= 0;
          else
          {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

      always @(posedge RdClk or posedge RRst)
          if(RRst)
            rptr<= 0;
          else
            rptr <= rgraynext;

      always @(posedge WrClk or posedge WRst)
          if(WRst)
             {wbin, wptr} <= 0;
           else
             {wbin, wptr} <= {wbin_num_next, wgraynext};

end
else if (WDEPTH == RDEPTH) begin :  Equal
  `ifdef En_ECC
         wire      [WDSIZE+PWIDTH-1:0]        Qreg   ;
         wire      [WDSIZE+PWIDTH-1:0]        data   ;
         wire      [WDSIZE-1:0]               Data_p ;
         wire      [PWIDTH-1:0]               P_out  ;
         wire      [WDSIZE-1:0]               Din    ;
         wire      [PWIDTH-1:0]               P_in   ;

      `getname(edc,`module_name) #( .DSIZE(WDSIZE)  )
         u_edc (
                   .WrClk                    ( WrClk )   ,
                   .RdClk                    ( RdClk )   ,
               `ifdef En_Reset
                   `ifdef Reset_Synchronization
                   .RST                      ( Reset )   ,
                   `else
                   .Reset                    ( WRst ) ,
                   .RPReset                  ( RRst ) ,
                   `endif
                `endif
                   .Ein                      ( Data )    ,
                   .Eout                     ( Data_p )  ,
                   .P_out                    ( P_out )   ,
                   .Din                      ( Din )     ,
                   .Dout                     ( Q )       ,
                   .P_in                     ( P_in )    ,
                  `ifdef Enable_force_error
                   .force_error              ( 2'b00 )   ,
                  `endif
                   .error                    ( ERROR )
              );
       assign  data  =   {P_out, Data_p};
       assign  Din   =   Qreg[WDSIZE-1:0];
       assign  P_in  =   Qreg[WDSIZE+PWIDTH-1:WDSIZE];
   `endif

   `ifdef EBR_BASED
      `ifdef En_ECC
          reg       [WDSIZE+PWIDTH-1:0]    mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
      `else
          reg       [WDSIZE-1:0]           mem[0:(WDEPTH-1)] /* synthesis syn_ramstyle= "block_ram" */;
      `endif
   `endif
   `ifdef DSR_BASED
          reg      [WDSIZE-1:0]            mem[0:(WDEPTH-1)]  /* synthesis syn_ramstyle= "distributed_ram" */;
   `endif
   `ifdef LUT_BASED
          reg      [WDSIZE-1:0]            mem[0:(WDEPTH-1)]  /* synthesis syn_ramstyle= "registers" */;
   `endif

   `ifdef  En_Output_Reg
      `ifdef En_ECC
          reg       [WDSIZE+PWIDTH-1:0]    wdata_q_r ;
      `else
          reg       [WDSIZE-1:0]           wdata_q_r ;
      `endif
   `endif
   `ifdef En_ECC
          reg       [WDSIZE+PWIDTH-1:0]    wdata_q ;
    `else
          //reg       [WDSIZE-1:0]           wdata_q ;
          wire      [WDSIZE-1:0]           wdata_q ;
   `endif
          reg       [ASIZE:0]              wptr ;
          reg       [ASIZE:0]              rptr ;
          reg       [ASIZE:0]              wq2_rptr ;
          reg       [ASIZE:0]              rq2_wptr ;
          reg       [ASIZE:0]              wq1_rptr ;
          reg       [ASIZE:0]              rq1_wptr;

          reg       [ASIZE:0]              wbin ;
          wire      [ASIZE:0]              rgraynext ;
          wire      [ASIZE:0]              wgraynext ;
          wire      [ASIZE:0]              wcount_r ;
          wire      [ASIZE:0]              rcount_w ;
          wire      [ASIZE:0]              wbinnext ;

  always @(posedge WrClk)
      if(WrEn && !Full)
        `ifdef En_ECC
           mem[waddr] <= data;
        `else
           mem[waddr] <= Data;
        `endif
`ifdef FWFT
`ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)begin
                  if(RRst) begin
                     wdata_q_r <= 0;
                     //wdata_q   <= 0;
                     end
                  else  if(RdEn ? ~rempty_val : (Empty & !rempty_val)) begin
                     wdata_q_r <= mem[raddr_num];
                    // wdata_q   <= wdata_q_r;
                  end
              end
             always @(posedge RdClk or posedge RRst)begin
                  if(RRst)
                      wdata_q   <= 0;
                  else if(RdEn)
                      wdata_q   <= wdata_q_r;
   `else
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                   wdata_q_r <= 0;
                else  if(RdEn ? ~rempty_val : (Empty & !rempty_val))
                   wdata_q_r <= mem[raddr_num];
   `endif
`else
         always @(posedge RdClk or posedge RRst)
                if(RRst)
                   wdata_q <= 0;
               // else if(~Empty)
                else if (RdEn ? ~rempty_val : (Empty & !rempty_val))
                   wdata_q <= mem[raddr_num];
`endif

`ifdef En_Output_Reg
  `ifndef Ctrl_By_RdEn
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                  wdata_q <=0;
                else
                  wdata_q <= wdata_q_r;
  `endif
`endif
`else
 `ifdef En_Output_Reg
   `ifdef Ctrl_By_RdEn
              always @(posedge RdClk or posedge RRst)begin
                  if(RRst) begin
                     wdata_q_r <= 0;
                     wdata_q   <= 0;
                     end
                  else  if(RdEn&~Empty) begin
                     wdata_q_r <= mem[raddr_num];
                     wdata_q   <= wdata_q_r;
                  end
              end
   `else
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                   wdata_q_r <= 0;
                else  if(RdEn&~Empty)
                   wdata_q_r <= mem[raddr_num];
   `endif
`else
         //always @(posedge RdClk or posedge RRst)
         //       if(RRst)
         //          wdata_q <= 0;
         //       else if(RdEn&~Empty)
         //          wdata_q <= mem[raddr_num];
         assign wdata_q = mem[raddr_num];
`endif

`ifdef En_Output_Reg
  `ifndef Ctrl_By_RdEn
            always @(posedge RdClk or posedge RRst)
                if(RRst)
                  wdata_q <=0;
                else
                  wdata_q <= wdata_q_r;
  `endif
`endif
`endif
`ifdef En_ECC
   assign Qreg            =  wdata_q;
`else
   assign Q               =  wdata_q;
`endif
`ifdef FWFT
   assign raddr_num       =  rbin_num_next[ASIZE-1:0];
`else
   assign raddr_num       =  rbin_num[ASIZE-1:0];
`endif
   assign rbin_num_next   =  rbin_num + (RdEn & ~Empty);
   assign rgraynext       =  (rbin_num_next>>1) ^ rbin_num_next;
   assign rempty_val      =  (rgraynext == rq2_wptr);
   assign wcount_r        =  gry2bin(rq2_wptr);
   assign rcnt_sub        =  {(wcount_r[ASIZE] ^ rbin_num[RASIZE]),wcount_r[ASIZE-1:0]}-{1'b0, rbin_num[RASIZE-1:0]};
   assign waddr           =  wbin[ASIZE-1:0];
   assign wbinnext        =  wbin + (WrEn & ~Full);
   assign wgraynext       =  (wbinnext>>1) ^ wbinnext;

   if (ASIZE == 1) begin :ag
      assign wfull_val    =  (wgraynext == ~wq2_rptr[ASIZE:ASIZE-1]);
      end
   else if (ASIZE > 1) begin :ah
        assign wfull_val  =  (wgraynext == {~wq2_rptr[ASIZE:ASIZE-1],wq2_rptr[ASIZE-2:0]});
   end

   assign rcount_w        =  gry2bin(wq2_rptr);
   assign wcnt_sub        =  {(rcount_w[ASIZE] ^ wbin[ASIZE]), wbin[ASIZE-1:0]} - {1'b0, rcount_w[ASIZE-1:0]};

       always @(posedge WrClk or posedge WRst)
           if(WRst)
             {wq2_rptr,wq1_rptr}<=0;
           else
             {wq2_rptr,wq1_rptr}<={wq1_rptr,rptr};

       always @(posedge RdClk or posedge RRst)
           if(RRst)
             {rq2_wptr,rq1_wptr} <= 0;
           else
             {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};

       always @(posedge RdClk or posedge RRst)
           if(RRst)
              rptr<= 0;
           else
              rptr <= rgraynext;

      always @(posedge WrClk or posedge WRst)
           if(WRst)
             {wbin, wptr} <= 0;
           else
             {wbin, wptr} <= {wbinnext, wgraynext};
end
endgenerate

       always @(posedge RdClk or posedge RRst)
           if(RRst)
              Empty <= 1'b1;
           else
              Empty <= rempty_val;

       always @(posedge WrClk or posedge WRst)
           if(WRst)
             Full <= 1'b0;
           else
             Full <= wfull_val;

`ifdef Al_Empty_Flag
  `ifdef Empty_S_Dual_Th
        assign arempty_val    = (rcnt_sub <= DeassertEmptyTh)|(( rcnt_sub == DeassertEmptyTh + 1'b1) & RdEn);
  `endif
  `ifdef  Empty_S_Single_Th
        assign arempty_val    = (rcnt_sub <= AEMPT)|((rcnt_sub == AEMPT + 1'b1) & RdEn);
  `endif
  `ifdef  Empty_D_Dual_Th
        assign arempty_val    = (rcnt_sub <= AlmostEmptyClrTh)|((rcnt_sub ==  AlmostEmptyClrTh + 1'b1) & RdEn);
  `endif
  `ifdef Empty_D_Single_Th
        assign arempty_val    = (rcnt_sub <= AlmostEmptyTh)|((rcnt_sub == AlmostEmptyTh + 1'b1) & RdEn);
  `endif
`endif

`ifdef Al_Full_Flag
  `ifdef Full_S_Dual_Th
        assign awfull_val    = (wcnt_sub >= DeassertFullTh)|((wcnt_sub == DeassertFullTh - 1'b1) & WrEn);
  `endif
  `ifdef  Full_S_Single_Th
        assign awfull_val    = (wcnt_sub >= AFULL)|((wcnt_sub == AFULL - 1'b1) & WrEn);
  `endif
  `ifdef  Full_D_Dual_Th
        assign awfull_val    = (wcnt_sub >= AlmostFullClrTh)|((wcnt_sub == AlmostFullClrTh - 1'b1) & WrEn);;
  `endif
  `ifdef  Full_D_Single_Th
        assign awfull_val    = (wcnt_sub >= AlmostFullTh)|((wcnt_sub == AlmostFullTh - 1'b1) & WrEn);
  `endif
`endif

`ifdef Al_Empty_Flag
       always @(posedge RdClk or posedge RRst)begin
           if(RRst)
              Almost_Empty <= 1'b1;
           else
        `ifdef Empty_S_Dual_Th
           if((arempty_val == 1)&&((rcnt_sub <= AssertEmptyTh)|(( rcnt_sub == AssertEmptyTh + 1'b1) & RdEn)))
               Almost_Empty <= 1'b1;
           else if (arempty_val == 0)
                   Almost_Empty <= 1'b0;
        `endif
        `ifdef Empty_S_Single_Th
              Almost_Empty <= arempty_val;
        `endif
        `ifdef Empty_D_Dual_Th
           if((arempty_val == 1)&&((rcnt_sub <= AlmostEmptySetTh)|(( rcnt_sub == AlmostEmptySetTh + 1'b1)&RdEn)))
               Almost_Empty <= 1'b1;
           else if (arempty_val == 0)
                   Almost_Empty <= 1'b0;
        `endif
        `ifdef Empty_D_Single_Th
              Almost_Empty <= arempty_val;
        `endif
  end
`endif

 `ifdef Al_Full_Flag
       always @(posedge WrClk or posedge WRst)begin
           if (WRst)
              Almost_Full <= 1'b0;
           else
         `ifdef Full_S_Dual_Th
            if((awfull_val == 1)&&((wcnt_sub >= AssertFullTh)|((wcnt_sub == AssertFullTh - 1'b1) & WrEn)))
                Almost_Full <= 1'b1;
            else if(awfull_val == 0)
                   Almost_Full <= 1'b0;
         `endif
         `ifdef  Full_S_Single_Th
                Almost_Full <= awfull_val;
         `endif
         `ifdef Full_D_Dual_Th
            if((awfull_val == 1)&&((wcnt_sub >= AlmostFullSetTh)|((wcnt_sub == AlmostFullSetTh - 1'b1) & WrEn)))
                Almost_Full <= 1'b1;
            else if(awfull_val == 0)
                   Almost_Full <= 1'b0;
         `endif
         `ifdef Full_D_Single_Th
                Almost_Full <= awfull_val;
         `endif
   end
`endif

`ifdef Count_W
         always @(posedge WrClk or posedge WRst)
             if(WRst)
                Wnum <= 0;
             else
                Wnum <= wcnt_sub;
`endif

`ifdef Count_R
          always @(posedge RdClk or posedge RRst)
             if(RRst)
               Rnum <= 0;
             else
               Rnum <= rcnt_sub;
`endif

function [ASIZE:0]gry2bin;
     input [ASIZE:0] gry_code;
     integer         i;
      begin
        gry2bin[ASIZE]=gry_code[ASIZE];
     for(i=ASIZE-1;i>=0;i=i-1)
        gry2bin[i]=gry2bin[i+1]^gry_code[i];
     end
  endfunction
endmodule



//==============================================================
//======FS Module
`define CLK_FREQ60
module `getname(usb_fs_phy,`module_name)
(
     input           clk_i
    ,input           rst_i
    ,output          usb_reset_detect_o
    ,input  [  7:0]  utmi_data_out_i
    ,input           utmi_txvalid_i
    ,input  [  1:0]  utmi_op_mode_i
    ,input           utmi_xcvrselect_i
    ,input           utmi_termselect_i
    ,output [  7:0]  utmi_data_in_o
    ,output          utmi_txready_o
    ,output          utmi_rxvalid_o
    ,output          utmi_rxactive_o
    ,output          utmi_rxerror_o
    ,output [  1:0]  utmi_linestate_o
    // USB D+ / D-
    ,inout           usb_dp_io
    ,inout           usb_dn_io
);

wire           usb_pads_rx_rcv_w;
wire           usb_pads_rx_dn_w;
wire           usb_pads_rx_dp_w;
wire           usb_pads_tx_dn_w;
wire           usb_pads_tx_dp_w;
wire           usb_pads_tx_oen_w;


`getname(usb_fs_transceiver,`module_name) u_usb_transceiver
(
    // Inputs
     .usb_phy_tx_dp_i  (usb_pads_tx_dp_w )
    ,.usb_phy_tx_dn_i  (usb_pads_tx_dn_w )
    ,.usb_phy_tx_oen_i (usb_pads_tx_oen_w)
    ,.mode_i           (1'b1             )
    ,.utmi_termselect_i(utmi_termselect_i)
    // Outputs
    ,.usb_phy_rx_rcv_o (usb_pads_rx_rcv_w)
    ,.usb_phy_rx_dp_o  (usb_pads_rx_dp_w )
    ,.usb_phy_rx_dn_o  (usb_pads_rx_dn_w )
    ,.usb_dp_io        (usb_dp_io        )
    ,.usb_dn_io        (usb_dn_io        )
);


`getname(usb_fs_phy_rt,`module_name) u_usb_phy_rt
(
    // Inputs
     .clk_i                   (clk_i            )
    ,.rst_i                   (rst_i            )
    ,.utmi_data_out_i         (utmi_data_out_i  )
    ,.utmi_txvalid_i          (utmi_txvalid_i   )
    ,.utmi_op_mode_i          (utmi_op_mode_i   )
    ,.utmi_xcvrselect_i       (utmi_xcvrselect_i)
    ,.utmi_termselect_i       (utmi_termselect_i)
    ,.usb_rx_rcv_i            (usb_pads_rx_rcv_w)
    ,.usb_rx_dp_i             (usb_pads_rx_dp_w )
    ,.usb_rx_dn_i             (usb_pads_rx_dn_w )
    ,.usb_reset_assert_i      (1'b0             )
    ,.utmi_data_in_o          (utmi_data_in_o   )
    ,.utmi_txready_o          (utmi_txready_o   )
    ,.utmi_rxvalid_o          (utmi_rxvalid_o   )
    ,.utmi_rxactive_o         (utmi_rxactive_o  )
    ,.utmi_rxerror_o          (utmi_rxerror_o   )
    ,.utmi_linestate_o        (utmi_linestate_o )
    ,.usb_tx_dp_o             (usb_pads_tx_dp_w )
    ,.usb_tx_dn_o             (usb_pads_tx_dn_w )
    ,.usb_tx_oen_o            (usb_pads_tx_oen_w)
    ,.usb_reset_detect_o      (usb_reset_detect_o)
    ,.usb_en_o                (                 )
);





endmodule






module `getname(usb_fs_phy_rt,`module_name)
(
    // Inputs
     input           clk_i
    ,input           rst_i
    ,input  [  7:0]  utmi_data_out_i
    ,input           utmi_txvalid_i
    ,input  [  1:0]  utmi_op_mode_i
    ,input           utmi_xcvrselect_i
    ,input           utmi_termselect_i

    ,input           usb_rx_rcv_i
    ,input           usb_rx_dp_i
    ,input           usb_rx_dn_i
    ,input           usb_reset_assert_i

    ,output [  7:0]  utmi_data_in_o
    ,output          utmi_txready_o
    ,output          utmi_rxvalid_o
    ,output          utmi_rxactive_o
    ,output          utmi_rxerror_o
    ,output [  1:0]  utmi_linestate_o
    ,output          usb_tx_dp_o
    ,output          usb_tx_dn_o
    ,output          usb_tx_oen_o
    ,output          usb_reset_detect_o
    ,output          usb_en_o
);


//`ifdef FS_MODE
//    localparam    SPEED_MODE = 1;
//`elsif LS_MODE
//    localparam    SPEED_MODE = 0;
//`else
//    localparam    SPEED_MODE = 1;
//`endif

  wire SPEED_MODE;
  assign  SPEED_MODE = 1'b1;//Full Spedd or High Speed, only support full speed

  wire [7:0] CLK_FREQ;
  assign  CLK_FREQ = `ifdef CLK_FREQ24
                          SPEED_MODE ? 8'd2 : 8'd16;
                     `elsif CLK_FREQ48
                          SPEED_MODE ? 8'd4 : 8'd32;
                     `elsif CLK_FREQ60
                          SPEED_MODE ? 8'd5 : 8'd40;
                     `else
                          SPEED_MODE ? 8'd5 : 8'd40;
                     `endif

//-----------------------------------------------------------------
// Wires / Registers
//-----------------------------------------------------------------
reg         rx_en_q;

// Xilinx placement pragmas:
//synthesis attribute IOB of out_dp_q is "TRUE"
//synthesis attribute IOB of out_dn_q is "TRUE"
reg         out_dp_q;
reg         out_dn_q;

wire        in_dp_w;
wire        in_dn_w;
wire        in_rx_w;

wire        in_j_w;
wire        in_k_w;
wire        in_se0_w;
wire        in_invalid_w;

wire        sample_w;


wire        bit_edge_w;
wire        bit_transition_w;

reg [2:0]   bit_count_q;
reg [2:0]   ones_count_q;
reg [7:0]   data_q;
reg         send_eop_q;

reg         sync_j_detected_q;

wire        bit_stuff_bit_w;
wire        next_is_bit_stuff_w;
wire        utmi_dppulldown_i=1'b0;
wire        utmi_dmpulldown_i=1'b0;

wire        usb_reset_assert_w = usb_reset_assert_i |
                                (utmi_xcvrselect_i == 1'b0 &&
                                 utmi_termselect_i == 1'b0  &&
                                 utmi_op_mode_i    == 2'b10 &&
                                 utmi_dppulldown_i &&
                                 utmi_dmpulldown_i);

//-----------------------------------------------------------------
// Resample async signals
//-----------------------------------------------------------------
reg         rx_dp_ms;
reg         rx_dn_ms;
reg         rxd_ms;
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    rx_dp_ms <= 1'b0;
    rx_dn_ms <= 1'b0;
    rxd_ms   <= 1'b0;
end
else
begin
    rx_dp_ms <= in_dp_w;
    rx_dn_ms <= in_dn_w;
    rxd_ms   <= ((in_dp_w == 1'b1)&&(in_dn_w == 1'b0)) ? 1'b1 : (((in_dp_w == 1'b0)&&(in_dn_w == 1'b1)) ? 1'b0 : rxd_ms);//in_rx_w;
end

//-----------------------------------------------------------------
// Edge Detection
//-----------------------------------------------------------------
reg         rx_dp0_q;
reg         rx_dn0_q;
reg         rx_dp1_q;
reg         rx_dn1_q;
reg         rx_dp_q;
reg         rx_dn_q;
reg         rxd0_q;
reg         rxd1_q;
reg         rxd_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    rx_dp0_q    <= 1'b0;
    rx_dn0_q    <= 1'b0;
    rx_dp1_q    <= 1'b0;
    rx_dn1_q    <= 1'b0;
    rx_dp_q     <= 1'b0;
    rx_dn_q     <= 1'b0;
    rxd0_q      <= 1'b0;
    rxd1_q      <= 1'b0;
    rxd_q       <= 1'b0;
end
else
begin
    // Glitch free versions
    if (rx_dp0_q & rx_dp1_q)
        rx_dp_q     <= 1'b1;
    else if (!rx_dp0_q & !rx_dp1_q)
        rx_dp_q     <= 1'b0;

    if (rx_dn0_q & rx_dn1_q)
        rx_dn_q     <= 1'b1;
    else if (!rx_dn0_q & !rx_dn1_q)
        rx_dn_q     <= 1'b0;

    if (rxd0_q & rxd1_q)
        rxd_q     <= 1'b1;
    else if (!rxd0_q & !rxd1_q)
        rxd_q     <= 1'b0;

    // Resyncs
    rx_dp1_q    <= rx_dp0_q;
    rx_dp0_q    <= rx_dp_ms;

    rx_dn1_q    <= rx_dn0_q;
    rx_dn0_q    <= rx_dn_ms;

    rxd1_q      <= rxd0_q;
    rxd0_q      <= rxd_ms;
end

// For Full Speed USB:
// SE0 = D+ = 0 && D- = 0
// J   = D+ = 1 && D- = 0
// K   = D+ = 0 && D- = 1


//For Low Speed USB:
// SE0 = D+ = 0 && D- = 0
// J   = D+ = 0 && D- = 1
// K   = D+ = 1 && D- = 0

assign in_k_w       = in_se0_w ? 1'b0 : (SPEED_MODE ? ~rxd_q:rxd_q);
assign in_j_w       = in_se0_w ? 1'b0 : (SPEED_MODE ? rxd_q:~rxd_q);
assign in_se0_w     = (!rx_dp_q & !rx_dn_q);
assign in_invalid_w = (rx_dp_q & rx_dn_q);

// Line state matches tx outputs if drivers enabled
assign utmi_linestate_o = usb_tx_oen_o ? {rx_dn_q, rx_dp_q} : {usb_tx_dn_o, usb_tx_dp_o};

//-----------------------------------------------------------------
// State Machine
//-----------------------------------------------------------------
localparam STATE_W              = 4;
localparam STATE_IDLE           = 4'd0;
localparam STATE_RX_DETECT      = 4'd1;
localparam STATE_RX_SYNC_J      = 4'd2;
localparam STATE_RX_SYNC_K      = 4'd3;
localparam STATE_RX_ACTIVE      = 4'd4;
localparam STATE_RX_EOP0        = 4'd5;
localparam STATE_RX_EOP1        = 4'd6;
localparam STATE_TX_SYNC        = 4'd7;
localparam STATE_TX_ACTIVE      = 4'd8;
localparam STATE_TX_EOP_STUFF   = 4'd9;
localparam STATE_TX_EOP0        = 4'd10;
localparam STATE_TX_EOP1        = 4'd11;
localparam STATE_TX_EOP2        = 4'd12;
localparam STATE_TX_RST         = 4'd13;

// Current state
reg [STATE_W-1:0] state_q;

reg [STATE_W-1:0] next_state_r;
always @ *
begin
    next_state_r = state_q;

    case (state_q)
    //-----------------------------------------
    // STATE_IDLE
    //-----------------------------------------
    STATE_IDLE :
    begin
        if (in_k_w)
            next_state_r    = STATE_RX_DETECT;
        else if (utmi_txvalid_i)
            next_state_r    = STATE_TX_SYNC;
        else if (usb_reset_assert_w)
            next_state_r    = STATE_TX_RST;
    end
    //-----------------------------------------
    // STATE_RX_DETECT
    //-----------------------------------------
    STATE_RX_DETECT :
    begin
        if (in_k_w && sample_w)
            next_state_r    = STATE_RX_SYNC_K;
        else if (sample_w)
            next_state_r    = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_RX_SYNC_J
    //-----------------------------------------
    STATE_RX_SYNC_J :
    begin
        if (in_k_w && sample_w)
            next_state_r    = STATE_RX_SYNC_K;
        // K glitch followed by multiple J's - return to idle
        else if ((bit_count_q == 3'd1) && sample_w)
            next_state_r    = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_RX_SYNC_K
    //-----------------------------------------
    STATE_RX_SYNC_K :
    begin
        // End of SYNC field ends with 2 K's
        // Must have seen at least 1 J state first!
        if (sync_j_detected_q && in_k_w && sample_w)
            next_state_r    = STATE_RX_ACTIVE;
        // No J detected since IDLE, must be an error!
        else if (!sync_j_detected_q && in_k_w && sample_w)
            next_state_r    = STATE_IDLE;
        else if (in_j_w && sample_w)
            next_state_r    = STATE_RX_SYNC_J;
    end
    //-----------------------------------------
    // STATE_RX_ACTIVE
    //-----------------------------------------
    STATE_RX_ACTIVE :
    begin
        if (in_se0_w && sample_w)
            next_state_r    = STATE_RX_EOP0;
        // Error!
        else if (in_invalid_w && sample_w)
            next_state_r    = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_RX_EOP0
    //-----------------------------------------
    STATE_RX_EOP0 :
    begin
        if (in_se0_w && sample_w)
            next_state_r    = STATE_RX_EOP1;
        // Error!
        else if (sample_w)
            next_state_r    = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_RX_EOP1
    //-----------------------------------------
    STATE_RX_EOP1 :
    begin
        // Return to idle
        if (in_j_w && sample_w)
            next_state_r    = STATE_IDLE;
        // Error!
        else if (sample_w)
            next_state_r    = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_TX_SYNC
    //-----------------------------------------
    STATE_TX_SYNC :
    begin
        if (bit_count_q == 3'd7 && sample_w)
            next_state_r    = STATE_TX_ACTIVE;
    end
    //-----------------------------------------
    // STATE_TX_ACTIVE
    //-----------------------------------------
    STATE_TX_ACTIVE :
    begin
        if (bit_count_q == 3'd7 && sample_w && (!utmi_txvalid_i || send_eop_q) && !bit_stuff_bit_w)
        begin
            // Bit stuff required at end of packet?
            if (next_is_bit_stuff_w)
                next_state_r    = STATE_TX_EOP_STUFF;
            else
                next_state_r    = STATE_TX_EOP0;
        end
    end
    //-----------------------------------------
    // STATE_TX_EOP_STUFF
    //-----------------------------------------
    STATE_TX_EOP_STUFF :
    begin
        if (sample_w)
            next_state_r    = STATE_TX_EOP0;
    end
    //-----------------------------------------
    // STATE_TX_EOP0
    //-----------------------------------------
    STATE_TX_EOP0 :
    begin
        if (sample_w)
            next_state_r    = STATE_TX_EOP1;
    end
    //-----------------------------------------
    // STATE_TX_EOP1
    //-----------------------------------------
    STATE_TX_EOP1 :
    begin
        if (sample_w)
            next_state_r    = STATE_TX_EOP2;
    end
    //-----------------------------------------
    // STATE_TX_EOP2
    //-----------------------------------------
    STATE_TX_EOP2 :
    begin
        if (sample_w)
            next_state_r    = STATE_IDLE;
    end
    //-----------------------------------------
    // STATE_TX_RST
    //-----------------------------------------
    STATE_TX_RST :
    begin
        if (!usb_reset_assert_w)
            next_state_r    = STATE_IDLE;
    end
    default:
        ;
   endcase
end

// Update state
always @ (posedge rst_i or posedge clk_i)
if (rst_i)
    state_q   <= STATE_IDLE;
else
    state_q   <= next_state_r;

//-----------------------------------------------------------------
// SYNC detect
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    sync_j_detected_q  <= 1'b0;
// Reset sync detect state in IDLE
else if (state_q == STATE_IDLE)
    sync_j_detected_q  <= 1'b0;
// At least one J detected
else if (state_q == STATE_RX_SYNC_J)
    sync_j_detected_q  <= 1'b1;

//-----------------------------------------------------------------
// Rx Error Detection
//-----------------------------------------------------------------
reg rx_error_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    rx_error_q  <= 1'b0;
// Rx bit stuffing error
else if (ones_count_q == 3'd7)
    rx_error_q  <= 1'b1;
// Invalid line state detection
else if (in_invalid_w && sample_w)
    rx_error_q  <= 1'b1;
// Detect invalid SYNC sequence
else if ((state_q == STATE_RX_SYNC_K) && !sync_j_detected_q && in_k_w && sample_w)
    rx_error_q  <= 1'b1;
else
    rx_error_q  <= 1'b0;

assign utmi_rxerror_o = rx_error_q;

//-----------------------------------------------------------------
// Edge Detector
//-----------------------------------------------------------------
reg rxd_last_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    rxd_last_q  <= 1'b0;
else
    rxd_last_q  <= in_j_w;

assign bit_edge_w = rxd_last_q ^ in_j_w;



reg [5:0] sample_cnt_q;
reg  [1:0]adjust_delayed_q;
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    sample_cnt_q        <= 6'd0;
    adjust_delayed_q    <= 2'd0;
end
// Delayed adjustment
else if (adjust_delayed_q>=2'd1)
    adjust_delayed_q    <= adjust_delayed_q-2'd1;
else if ((bit_edge_w && (sample_cnt_q != 6'd0) && (state_q < STATE_TX_SYNC))||(sample_cnt_q==CLK_FREQ - 1'b1))
    sample_cnt_q        <= 6'd0;
// Can't adjust sampling point now?
else if (bit_edge_w && (sample_cnt_q == 6'd0) && (state_q < STATE_TX_SYNC))
begin
    // Want to reset sampling point but need to delay adjustment by 1 cycle!
    adjust_delayed_q    <= 2'd1;
    sample_cnt_q        <= sample_cnt_q + 6'd1;
end
else
    sample_cnt_q        <= sample_cnt_q + 6'd1;

assign sample_w = (sample_cnt_q == CLK_FREQ[7:1]);
//-----------------------------------------------------------------
// NRZI Receiver
//-----------------------------------------------------------------
reg rxd_last_j_q;

// NRZI:
// 0 = transition between J & K
// 1 = same state
// After 6 consequitive 1's, a 0 is inserted to maintain the transitions

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    rxd_last_j_q  <= 1'b0;
else if ((state_q == STATE_IDLE) || sample_w)
    rxd_last_j_q  <= in_j_w;

assign bit_transition_w = sample_w ? rxd_last_j_q ^ in_j_w : 1'b0;

//-----------------------------------------------------------------
// Bit Counters
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    ones_count_q <= 3'd1;
// The packet starts with a double K (no transition)
else if (state_q == STATE_IDLE)
    ones_count_q <= 3'd1;
// Rx
else if ((state_q == STATE_RX_ACTIVE) && sample_w)
begin
    if (bit_transition_w)
        ones_count_q <= 3'b0;
    else
        ones_count_q <= ones_count_q + 3'd1;
end
// Tx
else if ((state_q == STATE_TX_ACTIVE) && sample_w)
begin
    // Toggle output data
    if (!data_q[0] || bit_stuff_bit_w)
        ones_count_q <= 3'b0;
    else
        ones_count_q <= ones_count_q + 3'd1;
end

assign bit_stuff_bit_w     = (ones_count_q == 3'd6);
assign next_is_bit_stuff_w = (ones_count_q == 3'd5) && !bit_transition_w;


always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    bit_count_q <= 3'd0;
else if ((state_q == STATE_IDLE) || (state_q == STATE_RX_SYNC_K))
    bit_count_q <= 3'd0;
else if ((state_q == STATE_RX_ACTIVE || state_q == STATE_TX_ACTIVE) && sample_w && !bit_stuff_bit_w)
    bit_count_q <= bit_count_q + 3'd1;
else if (((state_q == STATE_TX_SYNC) || (state_q == STATE_RX_SYNC_J)) && sample_w)
    bit_count_q <= bit_count_q + 3'd1;

//-----------------------------------------------------------------
// Shift register
//-----------------------------------------------------------------
always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    data_q  <= 8'b0;
// Pre-load shift register with SYNC word
else if (state_q == STATE_IDLE)
    data_q  <= 8'b00101010;
else if ((state_q == STATE_RX_ACTIVE) && sample_w && !bit_stuff_bit_w)
    data_q  <= {~bit_transition_w, data_q[7:1]};
else if ((state_q == STATE_TX_SYNC) && sample_w)
begin
    if (bit_count_q == 3'd7)
        data_q  <= utmi_data_out_i;
    else
        data_q  <= {~bit_transition_w, data_q[7:1]};
end
else if ((state_q == STATE_TX_ACTIVE) && sample_w && !bit_stuff_bit_w)
begin
    if (bit_count_q == 3'd7)
        data_q  <= utmi_data_out_i;
    else
        data_q  <= {~bit_transition_w, data_q[7:1]};
end

// Receive active (SYNC recieved)
assign utmi_rxactive_o = (state_q == STATE_RX_ACTIVE);

assign utmi_data_in_o  = data_q;

//-----------------------------------------------------------------
// Rx Ready
//-----------------------------------------------------------------
reg rx_ready_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    rx_ready_q <= 1'b0;
else if ((state_q == STATE_RX_ACTIVE) && sample_w && (bit_count_q == 3'd7) && !bit_stuff_bit_w)
    rx_ready_q <= 1'b1;
else
    rx_ready_q <= 1'b0;

assign utmi_rxvalid_o  = rx_ready_q;

//-----------------------------------------------------------------
// Tx Ready
//-----------------------------------------------------------------
reg tx_ready_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    tx_ready_q <= 1'b0;
else if ((state_q == STATE_TX_SYNC) && sample_w && (bit_count_q == 3'd7))
    tx_ready_q <= 1'b1;
else if ((state_q == STATE_TX_ACTIVE) && sample_w && !bit_stuff_bit_w && (bit_count_q == 3'd7) && !send_eop_q)
    tx_ready_q <= 1'b1;
else
    tx_ready_q <= 1'b0;

assign utmi_txready_o  = tx_ready_q;

//-----------------------------------------------------------------
// EOP pending
//-----------------------------------------------------------------
always @ (posedge rst_i or posedge clk_i)
if (rst_i)
    send_eop_q  <= 1'b0;
else if ((state_q == STATE_TX_ACTIVE) && !utmi_txvalid_i)
    send_eop_q  <= 1'b1;
else if (state_q == STATE_TX_EOP0)
    send_eop_q  <= 1'b0;

//-----------------------------------------------------------------
// Tx
//-----------------------------------------------------------------
wire out_bit_w = sample_w ? data_q[0] : 1'bz;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
begin
    out_dp_q <= 1'b0;
    out_dn_q <= 1'b0;
    rx_en_q  <= 1'b1;
end
else if (state_q == STATE_IDLE)
begin
    // IDLE
    out_dp_q <= (SPEED_MODE ? 1'b1 : 1'b0);
    out_dn_q <= (SPEED_MODE ? 1'b0 : 1'b1);

    if (utmi_txvalid_i || usb_reset_assert_w)
        rx_en_q <= 1'b0;
    else
        rx_en_q <= 1'b1;
end
else if ((state_q == STATE_TX_SYNC) && sample_w)
begin
    out_dp_q <= (SPEED_MODE ? data_q[0]  : ~data_q[0]);
    out_dn_q <= (SPEED_MODE ? ~data_q[0] : data_q[0]);
end
else if ((state_q == STATE_TX_ACTIVE || state_q == STATE_TX_EOP_STUFF) && sample_w)
begin
    // 0 = toggle, 1 = hold
    if (!data_q[0] || bit_stuff_bit_w)
    begin
        out_dp_q <= ~out_dp_q;
        out_dn_q <= ~out_dn_q;
    end
end
else if ((state_q == STATE_TX_EOP0 || state_q == STATE_TX_EOP1) && sample_w)
begin
    // SE0
    out_dp_q <= 1'b0;
    out_dn_q <= 1'b0;
end
else if ((state_q == STATE_TX_EOP2) && sample_w)
begin
    // IDLE
    out_dp_q <= (SPEED_MODE ? 1'b1 : 1'b0);
    out_dn_q <= (SPEED_MODE ? 1'b0 : 1'b1);

    // Set bus to input
    rx_en_q <= 1'b1;
end
else if (state_q == STATE_TX_RST)
begin
    // SE0
    out_dp_q <= 1'b0;
    out_dn_q <= 1'b0;
end

//-----------------------------------------------------------------
// Reset detection
//-----------------------------------------------------------------
reg [6:0] se0_cnt_q;

always @ (posedge clk_i or posedge rst_i)
if (rst_i)
    se0_cnt_q <= 7'b0;
else if (in_se0_w)
begin
    if (se0_cnt_q != 7'd127)
        se0_cnt_q <= se0_cnt_q + 7'd1;
end
else
    se0_cnt_q <= 7'b0;

assign usb_reset_detect_o = (se0_cnt_q == 7'd127);

//-----------------------------------------------------------------
// Transceiver Interface
//-----------------------------------------------------------------
// Tx output enable (active low)
assign usb_tx_oen_o = rx_en_q;

// Tx +/-
assign usb_tx_dp_o  = out_dp_q;
assign usb_tx_dn_o  = out_dn_q;

// Receive D+/D-
assign in_dp_w = usb_rx_dp_i;
assign in_dn_w = usb_rx_dn_i;

// Receive data
assign in_rx_w = usb_rx_rcv_i;

// USB device pull-up enable
assign usb_en_o = utmi_termselect_i;


endmodule

//-----------------------------------------------------------------
//                          Generated File
//-----------------------------------------------------------------

module `getname(usb_fs_transceiver,`module_name)
(
    // Inputs
     input           usb_phy_tx_dp_i
    ,input           usb_phy_tx_dn_i
    ,input           usb_phy_tx_oen_i
    ,input           mode_i
    ,input           utmi_termselect_i

    // Outputs
    ,inout           usb_dp_io
    ,inout           usb_dn_io
    ,output          usb_phy_rx_rcv_o
    ,output          usb_phy_rx_dp_o
    ,output          usb_phy_rx_dn_o
);



//-----------------------------------------------------------------
// Module: usb_transceiver
// Emulate standard USB PHY interface and produce a D+/D- outputs.
// Allows direct connection of USB port to FPGA.
// Limitations:
// As no differential amplifier present, no common mode noise
// rejection occurs.
// Unlikely to work well with longer connections!
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// Wires
//-----------------------------------------------------------------
reg out_dp;
reg out_dn;

//-----------------------------------------------------------------
// Assignments
//-----------------------------------------------------------------

// D+/D- Tristate buffers
assign usb_dp_io = utmi_termselect_i ? ((usb_phy_tx_oen_i == 1'b0) ? out_dp : 1'bz) : 1'b0;
assign usb_dn_io = utmi_termselect_i ? ((usb_phy_tx_oen_i == 1'b0) ? out_dn : 1'bz) : 1'b0;

// Receive D+/D-
assign usb_phy_rx_dp_o = usb_dp_io;
assign usb_phy_rx_dn_o = usb_dn_io;

// Receive output
assign usb_phy_rx_rcv_o = (usb_dp_io == 1'b1 && usb_dn_io == 1'b0) ? 1'b1 : 1'b1;

// PHY Transmit Mode:
// When phy_tx_mode_i is '0' the outputs are encoded as:
//     vmo_i, vpo_i
//      0    0    Differential Logic '0'
//      0    1    Differential Logic '1'
//      1    0    Single Ended '0'
//      1    1    Single Ended '0'
// When phy_tx_mode_i is '1' the outputs are encoded as:
//     vmo_i, vpo_i
//      0    0    Single Ended '0'
//      0    1    Differential Logic '1'
//      1    0    Differential Logic '0'
//      1    1    Illegal State
always @ (mode_i or usb_phy_tx_dp_i or usb_phy_tx_dn_i)
begin : MUX
 if (mode_i == 1'b0)
    begin
        if (usb_phy_tx_dp_i == 1'b0 && usb_phy_tx_dn_i == 1'b0)
        begin
            // Logic "0"
            out_dp = 1'b0;
            out_dn = 1'b1;
        end
        else if (usb_phy_tx_dp_i == 1'b0 && usb_phy_tx_dn_i == 1'b1)
        begin
            // SE0 (both low)
            out_dp = 1'b0;
            out_dn = 1'b0;
        end
        else if (usb_phy_tx_dp_i == 1'b1 && usb_phy_tx_dn_i == 1'b0)
        begin
            // Logic "1"
            out_dp = 1'b1;
            out_dn = 1'b0;
        end
        //else if (usb_phy_tx_dp_i == 1'b1 && usb_phy_tx_dn_i == 1'b1)
        else
        begin
            // SE0 (both low)
            out_dp = 1'b0;
            out_dn = 1'b0;
        end
    end
    else
    begin
        if (usb_phy_tx_dp_i == 1'b0 && usb_phy_tx_dn_i == 1'b0)
        begin
            // SE0 (both low)
            out_dp = 1'b0;
            out_dn = 1'b0;
        end
        else if (usb_phy_tx_dp_i == 1'b0 && usb_phy_tx_dn_i == 1'b1)
        begin
            // Logic "0"
            out_dp = 1'b0;
            out_dn = 1'b1;
        end
        else if (usb_phy_tx_dp_i == 1'b1 && usb_phy_tx_dn_i == 1'b0)
        begin
            // Logic "1"
            out_dp = 1'b1;
            out_dn = 1'b0;
        end
        //else if (usb_phy_tx_dp_i == 1'b1 && usb_phy_tx_dn_i == 1'b1)
        else
        begin
            // Illegal
            out_dp = 1'b1;
            out_dn = 1'b1;
        end
    end
 //endcase
end


endmodule



//==============================================================
//======

//module oser_rst
module `getname(oser_rst ,`module_name)
(input	clk_in
,input	rst_n
,input	pll_lock
,output	reg	reset_stop
,output	reg	reset_calib
,output	reg	set_calib
,output	reg	ready
);

/*******************************************************************************
 	* OSER FSM States defined
 	*******************************************************************************/
	parameter STATE_IDLE           	= 5'b0000;
	parameter STATE_SERDES_STOP_S  	= 5'b0001;
	parameter STATE_SERDES_RST_S    = 5'b0010;
	parameter STATE_SERDES_RST_E    = 5'b0011;
	parameter STATE_SERDES_STOP_E  	= 5'b0100;
	parameter STATE_SERDES_WAIT  	= 5'b0101;
	parameter STATE_SERDES_CALIB  	= 5'b0110;
	parameter STATE_SERDES_READY  	= 5'b0111;
	/*******************************************************************************
	//reg and wire definition
	 *******************************************************************************/
	reg [4:0] Fsm_serdes_rst_ctrl;
	reg [9:0] Cnt_serdes_rst_ctrl;

 /*******************************************************************************/
 /*******************************************************************************/
	always @(posedge clk_in or negedge rst_n)
	begin
		if (~rst_n) begin
			Fsm_serdes_rst_ctrl <= STATE_IDLE;
			Cnt_serdes_rst_ctrl <= 'd0;
			reset_stop <= 1'b0;
			reset_calib <= 1'b0;
			set_calib <= 1'b0;
			ready <= 1'b0;
		end
		else begin
			case(Fsm_serdes_rst_ctrl)
				STATE_IDLE:begin//idle
					if(pll_lock)
					Fsm_serdes_rst_ctrl <= STATE_SERDES_STOP_S;
					Cnt_serdes_rst_ctrl <= 'd0;
					reset_stop <= 1'b0;
					reset_calib <= 1'b0;
					set_calib <= 1'b0;
					ready <= 1'b0;
				end
				STATE_SERDES_STOP_S:begin
					Cnt_serdes_rst_ctrl<=Cnt_serdes_rst_ctrl+1'b1;
					if(Cnt_serdes_rst_ctrl[3])begin
						// reset_stop <= 1'b1;//
						reset_calib <= 1'b1;//
						Fsm_serdes_rst_ctrl <= STATE_SERDES_RST_S;
					end
					else begin
						reset_stop <= 1'b0;
					end
				end
				STATE_SERDES_RST_S:begin
					Cnt_serdes_rst_ctrl <= Cnt_serdes_rst_ctrl+1'b1;
					if(Cnt_serdes_rst_ctrl[5])begin
						// reset_calib <= 1'b1;
						reset_stop <= 1'b1;
						Fsm_serdes_rst_ctrl <= STATE_SERDES_RST_E;
					end
				end
				STATE_SERDES_RST_E:begin
					Cnt_serdes_rst_ctrl <= Cnt_serdes_rst_ctrl+1'b1;
					if(Cnt_serdes_rst_ctrl[6])begin
						reset_calib <= 1'b0;
						Fsm_serdes_rst_ctrl <= STATE_SERDES_STOP_E;
					end
				end
				STATE_SERDES_STOP_E:begin
					Cnt_serdes_rst_ctrl <= Cnt_serdes_rst_ctrl+1'b1;
					if(Cnt_serdes_rst_ctrl[7])begin
						reset_stop <= 1'b0;
						Fsm_serdes_rst_ctrl <= STATE_SERDES_CALIB;
					end
				end
				STATE_SERDES_CALIB:begin
					Cnt_serdes_rst_ctrl <= Cnt_serdes_rst_ctrl+1'b1;
					if(Cnt_serdes_rst_ctrl[3:0] == 4'h04 )begin
						set_calib <= 1'b1;
					end
					// if(Cnt_serdes_rst_ctrl[3:0] == 4'h08)begin
						// set_calib <= 1'b0;
					// end
					if(Cnt_serdes_rst_ctrl[6])begin
						set_calib <= 1'b0;
						Fsm_serdes_rst_ctrl <= STATE_SERDES_READY;
					end
				end

				STATE_SERDES_READY:begin
					Cnt_serdes_rst_ctrl <= Cnt_serdes_rst_ctrl+1'b1;
					if(Cnt_serdes_rst_ctrl[8])begin
						ready <= 1'b1;
						// set_calib <= 1'b0;
						Fsm_serdes_rst_ctrl <= STATE_SERDES_WAIT;
					end
				end
				STATE_SERDES_WAIT:begin
					if(~pll_lock)begin
						Fsm_serdes_rst_ctrl<=STATE_IDLE;
					end
				end
				default:begin
					Fsm_serdes_rst_ctrl<=STATE_IDLE;
				end
			endcase
		end
	end
endmodule
//module DRU_500M_Top(
module `getname(DRU_500M_Top ,`module_name) (
    input wire serial_data, // serial data at line rate
    input wire sys_clk,     // a system clock, such as 50MHz crystal, use as PLL reference clock and OSER_Reset sync FSM clock
    input wire rstn,       // connect to external global reset
    input wire fclk,
    input wire sclk,           // plck is synced to data_valid and data_out
    input wire reset_calib,
    output wire [15:0] ides8_out,
    output wire [4:0]  data_out,  // DRU data output
    output wire [1:0]  data_valid // DRU data output flag. 2b'00: data_out[2:0] valid; 2b'01: data_out[3:0] valid; 2b'11: data_out[4:0] valid
    ,output wire [15:0] DRU_dbg_signals_o
);
parameter IODELAY_A = 0;  // preferred to be 0
parameter IODELAY_B = 25; // UI/4 delay, 1N is 20ps/tap, 2A is TBD ps/tap ,max 0~127 range.
/*
As we use interleave method, please make sure the delay is correctly caculated
Take 1N-2 and 500Mbps as example:
 1. data rate is 500Mbps, so the UI = 1/500M = 2ns, so the UI/4 = 500ps
 2. 1N-2 the IODELAY is 20ps per tap, so that the delay_tap = 500ps/20ps = 25
For 2A device, please check the correct delay per tap.
*/


wire reset_stop,ready;
wire fclk_pll, pll_locked;
wire serial_data_a,serial_data_b;
wire [15:0] data_in;
assign ides8_out = data_in;
//////////////////////////////////////////////////////////////////////////////

//======= interleave input signal by IODELAY ==============
// the signal will be sampled by 2 IDES8 with interleave, which is equal to an IDES16
//

IODELAY iodelay_inst_a(
  .DO(serial_data_a),
  .DF(),
  .DI(serial_data),
  .SDTAP(1'b0),
  .SETN(1'b0),
  .VALUE(1'b0)
);
defparam iodelay_inst_a.C_STATIC_DLY=IODELAY_A;

IODELAY iodelay_inst_b(
  .DO(serial_data_b),
  .DF(),
  .DI(serial_data),
  .SDTAP(1'b0),
  .SETN(1'b0),
  .VALUE(1'b0)
);
defparam iodelay_inst_b.C_STATIC_DLY=IODELAY_B; //delay UI/4. 500Mbps. 500ps/20->25taps. 1N-2 is 20ps/tap

IDES8 uut_ides8_a(
  .Q0(data_in[14]),
  .Q1(data_in[12]),
  .Q2(data_in[10]),
  .Q3(data_in[8 ]),
  .Q4(data_in[6 ]),
  .Q5(data_in[4 ]),
  .Q6(data_in[2 ]),
  .Q7(data_in[0 ]),
  .D(serial_data_a),
  .FCLK(fclk),
  .PCLK(sclk),
  .CALIB(1'b0),
  .RESET(reset_calib)
  //.RESET(1'b0)
);

IDES8 uut_ides8_b(
  .Q0(data_in[15]),
  .Q1(data_in[13]),
  .Q2(data_in[11]),
  .Q3(data_in[9 ]),
  .Q4(data_in[7 ]),
  .Q5(data_in[5 ]),
  .Q6(data_in[3 ]),
  .Q7(data_in[1 ]),
  .D(serial_data_b),
  .FCLK(fclk),
  .PCLK(sclk),
  .CALIB(1'b0),
  //.RESET(1'b0)
  .RESET(reset_calib)
);

// DRU soft macro, there is another document to describe the theory for oversampling method.
//DRU_500M_Logic u_dru_500m_logic(
`getname(DRU_500M_Logic ,`module_name)
u_dru_500m_logic(
  .clk_in(sclk),
  .rst_n(rstn),
  .data_in(data_in),
  .data_out(data_out),
  .data_valid(data_valid)
  ,.DRU_dbg_signals_o(DRU_dbg_signals_o)
);
// ===================================================

endmodule


//module DRU_500M_Logic  (
module `getname(DRU_500M_Logic ,`module_name) (
     input  wire                        clk_in
    ,input  wire                        rst_n
    ,input  wire [15 : 0]               data_in
    ,output reg  [4 : 0]                data_out
    ,output wire [1 : 0]                data_valid
    ,output wire [15 : 0]               DRU_dbg_signals_o
);

localparam S00 = 2'b00;
localparam S01 = 2'b01;
localparam S10 = 2'b10;
localparam S11 = 2'b11;
//localparam S00 = 3'b000;
//localparam S01 = 3'b001;
//localparam S10 = 3'b010;
//localparam S11 = 3'b100;

reg [1:0] status = 2'b0;
reg [7:0] E4=4'b0;
reg [15:0] II=16'b0, ID=16'b0, IDD=16'b0;
reg I7DD=1'b0;

reg [1:0] DVE=2'b0, DV=2'b0;
wire [1:0] cDVE, cDV;


always @(posedge clk_in) begin
  II <= data_in;
  ID <= II;
  IDD <= ID;
  I7DD <= ID[15];
  E4[0] <= (II[0] ^ II[1]) | (II[4] ^ II[5]) | (II[8] ^ II[9])   | (II[12] ^ II[13]);
  E4[1] <= (II[1] ^ II[2]) | (II[5] ^ II[6]) | (II[9] ^ II[10])  | (II[13] ^ II[14]);
  E4[2] <= (II[2] ^ II[3]) | (II[6] ^ II[7]) | (II[10] ^ II[11]) | (II[14] ^ II[15]);
  E4[3] <= (II[3] ^ II[4]) | (II[7] ^ II[8]) | (II[11] ^ II[12]) | (II[15] ^ ID[0]);
end

wire [3:0] sync_bit;
reg dru_locked;

//assign sync_bit[3] = ({II[15],II[11],II[7],II[4],data_in[15],data_in[11],data_in[7],data_in[3]} == 8'b01010101)||
assign sync_bit[3] = ({II[15],II[11],II[7],II[3],data_in[15],data_in[11],data_in[7],data_in[3]} == 8'b01010101)||
                     ({ID[4],II[15],II[11],II[7],II[4],data_in[15],data_in[11],data_in[7]} == 8'b01010101)||
                     ({ID[7],ID[4],II[15],II[11],II[7],II[4],data_in[15],data_in[11]} == 8'b01010101)||
                     ({ID[11],ID[7],ID[4],II[15],II[11],II[7],II[4],data_in[15]} == 8'b01010101);
assign sync_bit[2] = ({II[14],II[10],II[6],II[2],data_in[14],data_in[10],data_in[6],data_in[2]} == 8'b010101010)||
                     ({ID[2],II[14],II[10],II[6],II[2],data_in[14],data_in[10],data_in[6]} == 8'b01010101)||
                     ({ID[6],ID[2],II[14],II[10],II[6],II[2],data_in[14],data_in[10]} == 8'b01010101)||
                     ({ID[10],ID[6],ID[2],II[14],II[10],II[6],II[2],data_in[14]} == 8'b01010101);
assign sync_bit[1] = ({II[13],II[ 9],II[5],II[1],data_in[13],data_in[ 9],data_in[5],data_in[1]} == 8'b01010101)||
                     ({ID[1],II[13],II[ 9],II[5],II[1],data_in[13],data_in[ 9],data_in[5]} == 8'b01010101)||
                     ({ID[5],ID[1],II[13],II[ 9],II[5],II[1],data_in[13],data_in[ 9]} == 8'b01010101)||
                     ({ID[ 9],ID[5],ID[1],II[13],II[ 9],II[5],II[1],data_in[13]} == 8'b01010101);
assign sync_bit[0] = ({II[12],II[ 8],II[4],II[0],data_in[12],data_in[ 8],data_in[4],data_in[0]} == 8'b01010101)||
                     ({ID[0],II[12],II[ 8],II[4],II[0],data_in[12],data_in[ 8],data_in[4]} == 8'b01010101)||
                     ({ID[4],ID[0],II[12],II[ 8],II[4],II[0],data_in[12],data_in[ 8]} == 8'b01010101)||
                     ({ID[ 8],ID[4],ID[0],II[12],II[ 8],II[4],II[0],data_in[12]} == 8'b01010101);

//assign DRU_dbg_signals_o[0] = E4[3];
//assign DRU_dbg_signals_o[1] = E4[2];
//assign DRU_dbg_signals_o[2] = E4[1];
//assign DRU_dbg_signals_o[3] = E4[0];
//
//assign DRU_dbg_signals_o[4] = dru_locked;
//
//assign DRU_dbg_signals_o[5] = data_valid[1];
//assign DRU_dbg_signals_o[6] = data_valid[0];
//
//assign DRU_dbg_signals_o[15:7] = 9'b0;

always @(posedge clk_in or negedge rst_n) begin
    if (!rst_n) begin
        dru_locked <= 1'b0;
    end
    else if (!dru_locked) begin
        if (&sync_bit[1:0]) begin
            dru_locked <= 1'b1;
        end
        else if (&sync_bit[2:1]) begin
            dru_locked <= 1'b1;
        end
        else if (&sync_bit[3:2]) begin
            dru_locked <= 1'b1;
        end
        else if (sync_bit[3]&sync_bit[0]) begin
            dru_locked <= 1'b1;
        end
    end
end
// combine logic
assign cDVE[0] = ((status==S10) && (E4[3]==1'b0) && (E4[2]==1'b1))? 1'b1:1'b0;

assign cDVE[1] = ((status==S00) && (E4[3]==1'b0) && (E4[0]==1'b1))? 1'b1:1'b0;
assign cDV[0] = ~DVE[1];//~
assign cDV[1] = DVE[0]; //(~DVE[0])^DVE[1];
// end of combine logic

assign data_valid = DV;

always @(posedge clk_in) begin
  DVE <= cDVE;
  DV <= cDV;
end

// state machine of edge detect
always @(posedge clk_in) begin
    if (!dru_locked) begin
        case(sync_bit)
            4'b0000 : status <= S00;//0 Failed
            4'b1010 : status <= S01;//A Failed
            4'b0101 : status <= S00;//5 Failed

            4'b0001 : status <= S00;//1
            4'b0011 : status <= S00;//3
            4'b1011 : status <= S00;//B

            4'b0010 : status <= S01;//2
            4'b0110 : status <= S01;//6
            4'b0111 : status <= S01;//7

            4'b0100 : status <= S10;//4
            4'b1100 : status <= S10;//E
            4'b1110 : status <= S10;//E
            4'b1111 : begin
                if (E4[0]) begin
                    status <= S10;//F
                end
                else if (E4[1]) begin
                    status <= S00;//F
                end
                else if (E4[2]) begin
                    status <= S01;//F
                end
                else if (E4[3]) begin
                    status <= S11;//F
                end
            end

            4'b1000 : status <= S11;//8
            4'b1001 : status <= S11;//9
            4'b1101 : status <= S11;//D
        endcase
    end
    else begin
        case(status)
          S00: begin
            if (E4[0] == 1'b1)
              status<= S10;
            else if(E4[3] == 1'b1)
              status<= S01;
          end
          S01: begin
            if (E4[1] == 1'b1)
              status<= S00;
            else if(E4[0] == 1'b1)
              status<= S11;
          end
          S11: begin
            if (E4[2] == 1'b1)
              status<= S01;
            else if(E4[1] == 1'b1)
              status<= S10;
          end
          S10: begin
            if (E4[3] == 1'b1)
              status<= S11;
            else if(E4[2] == 1'b1)
              status<= S00;
          end
          default: status <= status;
        endcase
    end
end // end of state machine for edge detect

always @(posedge clk_in) begin
  case(status)
    S00: data_out <= {I7DD,IDD[12],IDD[8],IDD[4],IDD[0]};
    S01: data_out <= {I7DD,IDD[13],IDD[9],IDD[5],IDD[1]};
    S11: data_out <= {I7DD,IDD[14],IDD[10],IDD[6],IDD[2]};
    S10: data_out <= {I7DD,IDD[15],IDD[11],IDD[7],IDD[3]};
    default: data_out <= data_out;
  endcase
end


endmodule  //module_name

`resetall
