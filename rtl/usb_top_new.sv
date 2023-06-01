
`default_nettype none

module usb_top_new (
    // Wishbone
    input  wire            wb_clk_i          ,
    input  wire            wb_rst_i          ,

    wb_if.slv              wb_sfr            ,
    // MSD wishbone
    wb_if.slv              wb_msd_sfr        ,
    wb_if.slv              wb_msd_sdc        ,
    wb_if.mst              wb_msd_dma        ,
    output wire            msd_tx_dma_irq_o  ,
    output wire            msd_rx_dma_irq_o  ,
    // CDC Wishbone
`ifdef USB_CDC_ENA
    wb_if.slv              wb_cdc            ,
    output wire            cdc_rx_irq_o      ,
`endif
    // USB Clocks
    input  wire            usb_clk480_i     ,
    input  wire            usb_clk60_i      ,
    input  wire            usb_pll_lock_i   ,

    // USB PHY
    inout  wire            usb_dxp_io       ,
    inout  wire            usb_dxn_io       ,
    input  wire            usb_rxdp_i       ,
    input  wire            usb_rxdn_i       ,
    output wire            usb_pullup_en_o  ,
    inout  wire            usb_term_dp_io   ,
    inout  wire            usb_term_dn_io   ,
    output  wire           usb_ack_tout_stretched_o     , 
    output  wire           usb_ack_received_stretched_o , 
    output  wire           usb_ack_bad_packet_stretched_o,
    output wire            utmi_txvalid_o
    ,output wire [3:0]     transact_state_o
    ,output wire w_eop_o
    ,output wire [3:0]  q_offset_o 
    ,output wire [3:0] crc_dbg_dataout_o
    ,output wire stuck_in_2nd_or_3rd_state_o
    ,output wire [15 : 0] DRU_dbg_signals_o
    ,output wire [19:0] dev_controller_dbg_o
    ,output wire [15:0] dbg
);

    usb_gowin_if usb();
    wire usb_rst;
    assign transact_state_o = usb.transact_state_o;
    // assign usb_dbg_if.endpt     = usb.endpt     ;
    // assign usb_dbg_if.setup     = usb.setup     ;
    // assign usb_dbg_if.txdat     = usb.txdat     ;
    // assign usb_dbg_if.txcork    = usb.txcork    ;
    // assign usb_dbg_if.txval     = usb.txval     ;
    // assign usb_dbg_if.txdat_len = usb.txdat_len ;
    // assign usb_dbg_if.txpop     = usb.txpop     ;
    // assign usb_dbg_if.txact     = usb.txact     ;
    // assign usb_dbg_if.rxdat     = usb.rxdat     ;
    // assign usb_dbg_if.rxval     = usb.rxval     ;
    // assign usb_dbg_if.rxact     = usb.rxact     ;
    // assign usb_dbg_if.rxrdy     = usb.rxrdy     ;

    usb_wishbone usb_wb_inst (
        .wb_clk_i         ( wb_clk_i         ),
        .wb_rst_i         ( wb_rst_i         ),
        .wb_sfr           ( wb_sfr           ),

        .wb_msd_sfr       ( wb_msd_sfr       ),
        .wb_msd_sdc       ( wb_msd_sdc       ),
        .wb_msd_dma       ( wb_msd_dma       ),
        .msd_tx_dma_irq_o ( msd_tx_dma_irq_o ),
        .msd_rx_dma_irq_o ( msd_rx_dma_irq_o ),
`ifdef USB_CDC_ENA
        .wb_cdc           ( wb_cdc           ),
        .cdc_rx_irq_o     ( cdc_rx_irq_o     ) ,
`endif
        .usb_clk60_i      ( usb_clk60_i      ),
        .usb_rst_i        ( ~usb_pll_lock_i  ),
        .usb              ( usb              ),
        .usb_rst_o        ( usb_rst          ),
        .usb_ack_tout_stretched_o       (  usb_ack_tout_stretched_o      ),
        .usb_ack_received_stretched_o   (  usb_ack_received_stretched_o  ), 
        .usb_ack_bad_packet_stretched_o  (  usb_ack_bad_packet_stretched_o )
        ,.dbg(dbg)
    );

    usb_gowin_top usb_backend (
        .clk480_i      ( usb_clk480_i   ),
        .clk60_i       ( usb_clk60_i    ),
        .pll_lock_i    ( usb_pll_lock_i ),
        .rst_i         ( usb_rst        ),
        .usb           ( usb            ),

        .utmi_txvalid_o (utmi_txvalid_o) ,
        .utmi_txready_o () ,

        .usb_dxp_io      ,
        .usb_dxn_io      ,
        .usb_rxdp_i      ,
        .usb_rxdn_i      ,
        .usb_pullup_en_o ,
        .usb_term_dp_io  ,
        .usb_term_dn_io
        ,.w_eop_o         (w_eop_o)
        ,.q_offset_o      (q_offset_o)
        ,.crc_dbg_dataout_o (crc_dbg_dataout_o)
        ,.stuck_in_2nd_or_3rd_state_o (stuck_in_2nd_or_3rd_state_o)
        ,.DRU_dbg_signals_o(DRU_dbg_signals_o)
        ,.dev_controller_dbg_o(dev_controller_dbg_o)
    );

endmodule

`resetall
