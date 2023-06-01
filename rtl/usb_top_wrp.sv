
`default_nettype none

module usb_top_wrp (
    input  wire            i_fclk_480M      ,
    input  wire            i_fclk_60M       ,
    input  wire            pll_locked       ,

    input  wire            i_wb_clk         ,
    input  wire            i_wb_reset       ,

    wb_if.slv              wbs              ,

    inout  wire            usb_dxp_io       ,
    inout  wire            usb_dxn_io       ,
    input  wire            usb_rxdp_i       ,
    input  wire            usb_rxdn_i       ,
    output wire            usb_pullup_en_o  ,
    inout  wire            usb_term_dp_io   ,
    inout  wire            usb_term_dn_io
);

    usb_device inst (
        .i_fclk_480M ,
        .i_fclk_60M  ,
        .pll_locked  ,

        .i_wb_clk                  ,
        .i_wb_reset                ,

        .i_wb_cyc    (wbs.cyc)     ,
        .i_wb_stb    (wbs.stb)     ,
        .i_wb_we     (wbs.we)      ,
        .i_wb_addr   (wbs.adr)     ,
        .i_wb_data   (wbs.dat_m2s) ,
        .i_wb_sel    (wbs.sel)     ,
        .o_wb_stall  ()            ,
        .o_wb_ack    (wbs.ack)     ,
        .o_wb_data   (wbs.dat_s2m) ,

        .usb_dxp_io       ,
        .usb_dxn_io       ,
        .usb_rxdp_i       ,
        .usb_rxdn_i       ,
        .usb_pullup_en_o  ,
        .usb_term_dp_io   ,
        .usb_term_dn_io   ,

        .usb_hdlc ()
    );

endmodule

`default_nettype wire
