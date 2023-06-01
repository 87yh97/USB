
`default_nettype none

module usb_msd_wishbone_dma #(
    parameter RX_DPTH_W    = 8,
    parameter TX_DPTH_W    = 8,
    parameter TX_SD_DPTH_W = 8
)(
    // Wishbone
    input  wire               wb_clk_i      ,
    input  wire               wb_rst_i      ,

    wb_if.slv                 wb_sfr        ,
    wb_if.slv                 wb_sd         ,
    wb_if.mst                 wb_dma        ,

    // DMA interrupt
    output wire               rx_dma_done_o ,
    output wire               tx_dma_done_o ,

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
    // TX SD FIFO
    input  wire                  tx_sd_rena_i   ,
    output wire            [7:0] tx_sd_rdat_o   ,
    output wire                  tx_sd_rempty_o ,
    output wire [TX_SD_DPTH_W:0] tx_sd_rnum_o   ,
    // USB TX control flags
    output wire               usb_tx_ena_o   ,
    output wire               usb_tx_sd_dat_o
);
    // ----------------------------------------------------------
    // Arbiter (just as demux) for dma wishbone masters
    // ----------------------------------------------------------

    wb_if wb_dma_int [2] (wb_clk_i, wb_rst_i);

    wb_arbiter_wrp #(.MST_N(2)) wb_arb (
        .clk_i ( wb_clk_i ),
        .rst_i ( wb_rst_i ),

        .wbm   (wb_dma_int),
        .wbs   (wb_dma)
    );

    // ----------------------------------------------------------
    // Protocol Data RX FIFO
    // ----------------------------------------------------------

    logic [7:0]         rx_rdat        ;
    logic               rx_rena        ;
    logic               rx_rempty      ;
    logic               rx_arempty     ;
    logic [RX_DPTH_W:0] rx_rnum        ;
    logic [RX_DPTH_W:0] rx_rnum_th     ;
    logic               rx_rfullenough ;

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
        .awfull       () ,
        .wnum         () ,
        .wavail_th    (),
        .wavail       (),
        .wemptyenough (),

        // Wishbone side (read)
        .rclk         ( wb_clk_i   ) ,
        .rrst_n       ( ~wb_rst_i  ) ,
        .rinc         ( rx_rena    ) ,
        .rdata        ( rx_rdat    ) ,
        .rempty       ( rx_rempty  ) ,
        .arempty      ( rx_arempty ) ,
        .rnum         ( rx_rnum    ) ,
        .rnum_th      ( rx_rnum_th ),
        .rfullenough  ( rx_rfullenough )
    );

    logic [31:0] rx_dma_start_adr ;
    logic        rx_dma_start     ;
    logic [15:0] rx_dma_data_len  ;
    logic        rx_dma_idle      ;

    wb_if rx_dma_wbm (wb_clk_i, wb_rst_i);

    wb_32bit_dma_8bit_fifo_reader rx_fifo_reader (
        .clk_i       ( wb_clk_i ) ,
        .rst_i       ( wb_rst_i ) ,

        .start_adr_i ( rx_dma_start_adr ) ,
        .start_i     ( rx_dma_start     ) ,
        .data_len_i  ( rx_dma_data_len  ) ,
        .idle_o      ( rx_dma_idle      ) ,
        .done_stb_o  ( rx_dma_done_o    ) ,

        .wbm         ( wb_dma_int[0] ) ,

        .rd_dat_i    ( rx_rdat        ) ,
        .rd_ena_o    ( rx_rena        ) ,
        .rd_ready_i  ( rx_rfullenough )
    );

    logic [31:0] rx_status;

    assign rx_status = {'0,
        rx_rnum,
        rx_rempty,
        rx_arempty,
        rx_rfullenough,
        rx_dma_idle
    };

    assign rx_rnum_th = rx_dma_data_len[RX_DPTH_W:0];

    // ----------------------------------------------------------
    // Protocol Data TX FIFO
    // ----------------------------------------------------------

    logic               tx_wena         ;
    logic         [7:0] tx_wdat         ;
    logic               tx_wfull        ;
    logic               tx_awfull       ;
    logic [TX_DPTH_W:0] tx_wnum         ;
    logic [TX_DPTH_W:0] tx_wawail_th    ;
    logic               tx_wemptyenough ;

    async_fifo #(
        .DSIZE       ( 8         ),
        .ASIZE       ( TX_DPTH_W ),
        .FALLTHROUGH ( "TRUE"    )
    ) tx_fifo_inst (
        // Wishbone side (write)
        .wclk         ( wb_clk_i        ) ,
        .wrst_n       ( ~wb_rst_i       ) ,
        .winc         ( tx_wena         ) ,
        .wdata        ( tx_wdat         ) ,
        .wfull        ( tx_wfull        ) ,
        .awfull       ( tx_awfull       ) ,
        .wnum         ( tx_wnum         ) ,
        .wavail_th    ( tx_wawail_th    ) ,
        .wemptyenough ( tx_wemptyenough ) ,
        .wavail       (),

        // USB side (read)
        .rclk         ( usb_clk60_i    ) ,
        .rrst_n       ( ~usb_rst_i     ) ,
        .rinc         ( tx_rena_i   ) ,
        .rdata        ( tx_rdat_o   ) ,
        .rempty       ( tx_rempty_o ) ,
        .rnum         ( tx_rnum_o   ) ,
        .arempty      ( ),
        .rnum_th      ( ),
        .rfullenough  ( )
    );

    logic [31:0] tx_dma_start_adr ;
    logic        tx_dma_start     ;
    logic [15:0] tx_dma_data_len  ;
    logic        tx_dma_idle      ;

    wb_if tx_dma_wbm (wb_clk_i, wb_rst_i);

    wb_32bit_dma_8bit_fifo_writer tx_fifo_writer (
        .clk_i       ( wb_clk_i ) ,
        .rst_i       ( wb_rst_i ) ,

        .start_adr_i ( tx_dma_start_adr ) ,
        .start_i     ( tx_dma_start     ) ,
        .data_len_i  ( tx_dma_data_len  ) ,
        .idle_o      ( tx_dma_idle      ) ,
        .done_stb_o  ( tx_dma_done_o    ) ,

        .wbm         ( wb_dma_int[1]    ) ,

        .wr_dat_o    ( tx_wdat          ) ,
        .wr_ena_o    ( tx_wena          ) ,
        .wr_ready_i  ( tx_wemptyenough  )
    );

    assign tx_wawail_th = tx_dma_data_len[TX_DPTH_W:0];

    logic [31:0] tx_status;

    assign tx_status = {'0,
        tx_wnum         ,
        tx_wfull        ,
        tx_awfull       ,
        tx_wemptyenough ,
        tx_dma_idle
    };

    // ----------------------------------------------------------
    // SD Data TX FIFO
    // ----------------------------------------------------------

    logic                  tx_sd_wena         ;
    logic            [7:0] tx_sd_wdat         ;
    logic                  tx_sd_wfull        ;
    logic                  tx_sd_awfull       ;
    logic [TX_SD_DPTH_W:0] tx_sd_wnum         ;
    logic                  tx_sd_wemptyenough ;

    localparam [TX_SD_DPTH_W:0] TX_SD_WAWAIL_TH = (TX_SD_DPTH_W)'(4);

    async_fifo #(
        .DSIZE       ( 8            ),
        .ASIZE       ( TX_SD_DPTH_W ),
        .FALLTHROUGH ( "TRUE"       )
    ) tx_sd_fifo_inst (
        // Wishbone side (write)
        .wclk         ( wb_clk_i        ) ,
        .wrst_n       ( ~wb_rst_i       ) ,

        .winc         ( tx_sd_wena         ) ,
        .wdata        ( tx_sd_wdat         ) ,
        .wfull        ( tx_sd_wfull        ) ,
        .awfull       ( tx_sd_awfull       ) ,
        .wnum         ( tx_sd_wnum         ) ,
        .wavail_th    ( TX_SD_WAWAIL_TH    ) ,
        .wemptyenough ( tx_sd_wemptyenough ) ,
        .wavail       (),

        // USB side (read)
        .rclk         ( usb_clk60_i    ) ,
        .rrst_n       ( ~usb_rst_i     ) ,
        .rinc         ( tx_sd_rena_i   ) ,
        .rdata        ( tx_sd_rdat_o   ) ,
        .rempty       ( tx_sd_rempty_o ) ,
        .rnum         ( tx_sd_rnum_o   ) ,
        .arempty      ( ),
        .rnum_th      ( ),
        .rfullenough  ( )
    );

    wb_32bit_to_8bit_fifo_writer tx_sd_fifo_writer (
        .clk_i       ( wb_clk_i ) ,
        .rst_i       ( wb_rst_i ) ,
        .wbs         ( wb_sd    ) ,

        .wr_dat_o    ( tx_sd_wdat ) ,
        .wr_ena_o    ( tx_sd_wena ) ,
        .wr_ee_i     ( tx_sd_wemptyenough )
    );

    logic [31:0] tx_sd_status;

    assign tx_sd_status = {'0,
        tx_sd_wnum         ,
        tx_sd_wfull        ,
        tx_sd_awfull       ,
        tx_sd_wemptyenough
    };

    // ----------------------------------------------------------
    // Wishbone MSD control & status registers
    // ----------------------------------------------------------

endmodule

`resetall
