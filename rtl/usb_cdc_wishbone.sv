
`default_nettype none

module usb_cdc_wishbone_sfr #(
    parameter TX_ADR_W ,
    parameter RX_ADR_W
)(
    input  wire        clk_i              ,
    input  wire        rst_i              ,
    wb_if.slv          wbs                ,

    input  wire              rx_rempty_i   ,
    input  wire [RX_ADR_W:0] rx_rnum_i     ,
    output wire              rx_rena_o     ,
    input  wire        [7:0] rx_rdat_i     ,

    input  wire              tx_wfull_i    ,
    input  wire [TX_ADR_W:0] tx_wnum_i     ,
    output logic             tx_wena_o     ,
    output logic       [7:0] tx_wdat_o     ,

    output wire              tx_ena_o      ,

    output logic             rx_irq_stb_o
);

    wire [5:0] wb_adr = wbs.adr[5:0];

    //
    // Address map:
    // 00 - read from rx fifo, write to tx fifo
    // 04 - read data available
    // 08 - writa space available
    // 0C - tx enable register
    //

    logic [7:0] tx_wdat;
    logic       tx_ena;
    logic       rx_irq;

    //
    // Wishbone read
    //

    always_ff @(posedge clk_i) begin
        case (wb_adr)
            5'h00   : wbs.dat_s2m <= {'0, rx_rdat_i}               ;
            5'h04   : wbs.dat_s2m <= {'0, rx_rnum_i, !rx_rempty_i} ;
            5'h08   : wbs.dat_s2m <= {'0, tx_wnum_i, !tx_wfull_i}  ;
            5'h0C   : wbs.dat_s2m <= {'0, tx_ena }                 ;
            default : wbs.dat_s2m <= 32'hdead_beaf  ;
        endcase
    end

    //
    // Wishbone write
    //

    always_ff @(posedge clk_i)
        if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'h00))
            tx_wdat <= wbs.dat_m2s[7:0];

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
            tx_ena  <= '0;
        else if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'h0C))
            tx_ena  <= wbs.dat_m2s[0];

    //
    // Fifo read & write strobe
    //

    logic rx_rena ;
    logic tx_wena ;

    wire  rx_rena_w;
    wire  tx_wena_w;

    assign rx_rena_w = wbs.cyc && wbs.stb && (wb_adr == '0) && wbs.ack && !wbs.we;
    assign tx_wena_w = wbs.cyc && wbs.stb && (wb_adr == '0) && wbs.ack &&  wbs.we;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            rx_rena <= '0;
            tx_wena <= '0;
        end else begin
            tx_wena <= tx_wena_w;
            rx_rena <= rx_rena_w;
        end

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
		    wbs.ack <= 1'b0;
	    else if (wbs.ack)
		    wbs.ack <= 1'b0;
	    else if (wbs.cyc && wbs.stb && !wbs.ack)
		    wbs.ack <= 1'b1;


    assign rx_rena_o = rx_rena;
    assign tx_wena_o = tx_wena;

    always_ff @(posedge clk_i)
        tx_wdat_o <= tx_wdat;

    assign tx_ena_o = tx_ena;

    //
    // RX data ready interrupt
    //

    logic [1:0] rx_empty_n_r;
    logic       rx_empty_n_stb;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            rx_empty_n_r <= '0;
            rx_empty_n_stb <= '0;
        end else begin
            rx_empty_n_r <= (rx_empty_n_r << 1) | !rx_rempty_i;
            rx_empty_n_stb <= (rx_empty_n_r[0] && !rx_empty_n_r[1]);
        end

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
            rx_irq <= 1'b0;
        else if (rx_empty_n_stb)
            rx_irq <= 1'b1;
        else if (wb_adr == 'h04 && wbs.ack)
            rx_irq <= 1'b0;

    logic rx_irq_r;
    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            rx_irq_r <= '0;
        end else begin
            rx_irq_r <= rx_irq;
        end

    assign rx_irq_stb_o = rx_irq && !rx_irq_r;

endmodule

module usb_cdc_wishbone #(
    parameter TX_DPTH_W ,
    parameter RX_DPTH_W
)(
    // Wishbone
    input  wire               wb_clk_i      ,
    input  wire               wb_rst_i      ,
    wb_if.slv                 wbs           ,
    output wire               rx_irq_o      ,

    // USB
    input  wire               usb_clk60_i   ,
    input  wire               usb_rst_i     ,
    // RX FIFO
    input  wire         [7:0] rx_wdat_i     ,
    input  wire               rx_wena_i     ,
    output wire               rx_wfull_o    ,
    // TX FIFO
    input  wire               tx_rena_i     ,
    output wire         [7:0] tx_rdat_o     ,
    output wire               tx_rempty_o   ,
    output wire [TX_DPTH_W:0] tx_rnum_o     ,

    output wire               tx_usb_ena_o
);

    wire               rx_rempty  ;
    wire [RX_DPTH_W:0] rx_rnum    ;
    wire               rx_rena    ;
    wire        [7:0]  rx_rdat    ;

    wire               tx_wfull   ;
    wire [TX_DPTH_W:0] tx_wnum    ;
    wire               tx_wena    ;
    wire        [7:0]  tx_wdat    ;

    wire               usb_tx_ena ;

    usb_cdc_wishbone_sfr #(
        .TX_ADR_W (TX_DPTH_W) ,
        .RX_ADR_W (RX_DPTH_W)
    ) wb_regs (
        .clk_i ( wb_clk_i  ) ,
        .rst_i ( wb_rst_i  ) ,
        .wbs   ( wbs       ) ,

        .rx_rnum_i   ( rx_rnum   ) ,
        .rx_rempty_i ( rx_rempty ) ,
        .rx_rena_o   ( rx_rena   ) ,
        .rx_rdat_i   ( rx_rdat   ) ,

        .tx_wnum_i   ( tx_wnum  ) ,
        .tx_wfull_i  ( tx_wfull ) ,
        .tx_wena_o   ( tx_wena  ) ,
        .tx_wdat_o   ( tx_wdat  ) ,

        .tx_ena_o    ( usb_tx_ena ),
        .rx_irq_stb_o( rx_irq_o   )
    );

    synchronizer #(.W(1), .INIT_VAL('0)) usb_tx_ena_sync (
        .clk_i  (usb_clk60_i) ,
        .rst_i  (usb_rst_i  ) ,
        .raw_i  (usb_tx_ena   ) ,
        .sync_o (tx_usb_ena_o )
    );

    async_fifo #(
        .DSIZE       ( 8         ),
        .ASIZE       ( RX_DPTH_W ),
        .FALLTHROUGH ( "TRUE"    )
    ) rx_fifo_inst (
        // USB side (write)
        .wclk         ( usb_clk60_i ) ,
        .wrst_n       ( ~usb_rst_i  ) ,
        .winc         ( rx_wena_i   ) ,
        .wdata        ( rx_wdat_i   ) ,
        .wfull        ( rx_wfull_o  ) ,
        .awfull       (),
        .wnum         (),
        .wavail_th    (),
        .wavail       (),
        .wemptyenough (),

        // Wishbone side (read)
        .rclk         ( wb_clk_i   ) ,
        .rrst_n       ( ~wb_rst_i  ) ,
        .rinc         ( rx_rena    ) ,
        .rdata        ( rx_rdat    ) ,
        .rempty       ( rx_rempty  ) ,
        .arempty      (            ) ,
        .rnum         ( rx_rnum    ) ,
        .rnum_th      ( '0 ) ,
        .rfullenough  ( )
    );

    async_fifo #(
        .DSIZE       ( 8         ),
        .ASIZE       ( TX_DPTH_W ),
        .FALLTHROUGH ( "TRUE"    )
    ) tx_fifo_inst (
        // Wishbone side (write)
        .wclk         ( wb_clk_i  ) ,
        .wrst_n       ( ~wb_rst_i ) ,
        .winc         ( tx_wena   ) ,
        .wdata        ( tx_wdat   ) ,
        .wfull        ( tx_wfull  ) ,
        .awfull       (           ) ,
        .wnum         ( tx_wnum   ) ,
        .wavail_th    ( '0        ) ,
        .wemptyenough (           ) ,
        .wavail       (           ),

        // USB side (read)
        .rclk         ( usb_clk60_i ) ,
        .rrst_n       ( ~usb_rst_i  ) ,
        .rinc         ( tx_rena_i   ) ,
        .rdata        ( tx_rdat_o   ) ,
        .rempty       ( tx_rempty_o ) ,
        .rnum         ( tx_rnum_o   ) ,
        .arempty      ( ),
        .rnum_th      ( ),
        .rfullenough  ( )
    );

endmodule

`resetall
