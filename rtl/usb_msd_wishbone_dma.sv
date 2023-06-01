
`default_nettype none

module usb_msd_wishbone_sfr (
    input  wire        clk_i              ,
    input  wire        rst_i              ,
    wb_if.slv          wbs                ,

    output wire [31:0] rx_dma_start_adr_o ,
    output wire [15:0] rx_dma_data_len_o  ,
    input  wire [31:0] rx_status_i        ,

    output wire [31:0] tx_dma_start_adr_o ,
    output wire [15:0] tx_dma_data_len_o  ,
    input  wire [31:0] tx_status_i        ,
    output wire        tx_dat_source_o    ,
    output wire        tx_usb_ena_o       ,

    // start strobes
    output logic       rx_dma_start_o     ,
    output logic       tx_dma_start_o
);

    wire [5:0] wb_adr = wbs.adr[5:0];

    logic [31:0] rx_dma_start_adr ; // 0
    logic [15:0] rx_dma_data_len  ; // 4
    logic [31:0] rx_status        ; // 8

    logic [31:0] tx_dma_start_adr ; // 12
    logic [15:0] tx_dma_data_len  ; // 16
    logic [31:0] tx_status        ; // 20

    struct packed {
        logic tx_usb_ena    ;
        logic tx_dat_source ;
    } tx_ctrl_reg; // 24

    always_ff @(posedge clk_i) begin
        if      (wb_adr == 5'd00) wbs.dat_s2m <= rx_dma_start_adr ;
        else if (wb_adr == 5'd04) wbs.dat_s2m <= rx_dma_data_len  ;
        else if (wb_adr == 5'd08) wbs.dat_s2m <= rx_status        ;
        else if (wb_adr == 5'd12) wbs.dat_s2m <= tx_dma_start_adr ;
        else if (wb_adr == 5'd16) wbs.dat_s2m <= tx_dma_data_len  ;
        else if (wb_adr == 5'd20) wbs.dat_s2m <= tx_status        ;
        else if (wb_adr == 5'd24) wbs.dat_s2m <= tx_ctrl_reg      ;
        else                      wbs.dat_s2m <= 32'hdead_beaf ;
    end

    always_ff @(posedge clk_i)
        if (wbs.cyc && wbs.stb && wbs.we) begin
            if      (wb_adr == 5'd0)  rx_dma_start_adr <= wbs.dat_m2s;
            else if (wb_adr == 5'd4)  rx_dma_data_len  <= wbs.dat_m2s[15:0];
            else if (wb_adr == 5'd12) tx_dma_start_adr <= wbs.dat_m2s;
            else if (wb_adr == 5'd16) tx_dma_data_len  <= wbs.dat_m2s[15:0];
            else if (wb_adr == 5'd24) tx_ctrl_reg      <= wbs.dat_m2s[1:0];
        end

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
            rx_dma_start_o <= '0;
        else begin
            if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'd4))
                rx_dma_start_o <= wbs.dat_m2s[31];
            else if (rx_dma_start_o)
                rx_dma_start_o <= 1'b0;
        end

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
            tx_dma_start_o <= '0;
        else begin
            if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'd16))
                tx_dma_start_o <= wbs.dat_m2s[31];
            else if (tx_dma_start_o)
                tx_dma_start_o <= 1'b0;
        end

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
		    wbs.ack <= 1'b0;
	    else if (wbs.ack)
		    wbs.ack <= 1'b0;
	    else if (wbs.cyc && wbs.stb && !wbs.ack)
		    wbs.ack <= 1'b1;

    assign rx_dma_start_adr_o = rx_dma_start_adr ;
    assign rx_dma_data_len_o  = rx_dma_data_len  ;
    assign tx_dma_start_adr_o = tx_dma_start_adr ;
    assign tx_dma_data_len_o  = tx_dma_data_len  ;

    assign tx_dat_source_o    = tx_ctrl_reg.tx_dat_source ;
    assign tx_usb_ena_o       = tx_ctrl_reg.tx_usb_ena    ;

    assign rx_status = rx_status_i;
    assign tx_status = tx_status_i;

endmodule

module usb_msd_wishbone_dma #(
    //parameter RX_DPTH_W    = 8,
    parameter RX_DPTH_W    = 8,
    parameter TX_DPTH_W    = 11
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
    // USB TX control flags
    output wire               usb_tx_ena_o    ,
    output wire               usb_tx_sd_dat_o
    ,output wire [15:0] dbg
);
    // ----------------------------------------------------------
    // Arbiter (just as mux) for dma wishbone masters
    // ----------------------------------------------------------

    wb_if wb_dma_int [2] (wb_clk_i, wb_rst_i);

    wb_arbiter_wrp #(.MST_N(2)) wb_arb (
        .clk_i ( wb_clk_i ),
        .rst_i ( wb_rst_i ),

        .wbm   (wb_dma_int),
        .wbs   (wb_dma)
    );

    // ----------------------------------------------------------
    // Wishbone MSD control & status registers
    // ----------------------------------------------------------

    logic [31:0] rx_dma_start_adr ;
    logic        rx_dma_start     ;
    logic [15:0] rx_dma_data_len  ;
    logic [31:0] rx_status        ;

    logic [31:0] tx_dma_start_adr ;
    logic        tx_dma_start     ;
    logic [15:0] tx_dma_data_len  ;
    logic [31:0] tx_status        ;

    logic        tx_dat_source    ; // 0 - dma, 1 - sdc

    usb_msd_wishbone_sfr wb_sfr_inst (
        .clk_i ( wb_clk_i ),
        .rst_i ( wb_rst_i ),
        .wbs   ( wb_sfr   ),

        .rx_dma_start_adr_o (rx_dma_start_adr),
        .rx_dma_data_len_o  (rx_dma_data_len),
        .rx_status_i        (rx_status),

        .tx_dma_start_adr_o (tx_dma_start_adr),
        .tx_dma_data_len_o  (tx_dma_data_len),
        .tx_status_i        (tx_status),
        .tx_dat_source_o    (tx_dat_source),

        .tx_usb_ena_o       (usb_tx_ena_o),

        .rx_dma_start_o     (rx_dma_start),
        .tx_dma_start_o     (tx_dma_start)
    );

    assign usb_tx_sd_dat_o = tx_dat_source;

    // ----------------------------------------------------------
    // Data RX FIFO
    // ----------------------------------------------------------

    logic [7:0]         rx_rdat        ;
    logic               rx_rena        ;
    logic               rx_rempty      ;
    logic               rx_arempty     ;
    logic [RX_DPTH_W:0] rx_rnum        ;
    logic [RX_DPTH_W:0] rx_rnum_th     ;
    logic               rx_rfullenough ;



    async_fifo_ft #(
        .DSIZE       ( 8         ),
        .ASIZE       ( RX_DPTH_W )
    ) rx_fifo_inst (
        // USB side (write)
        .wclk         ( usb_clk60_i ) ,
        .wrst_n       ( ~usb_rst_i  ) ,
        .winc         ( rx_wena_i   ) ,
        .wdata        ( rx_wdat_i   ) ,
        .wfull        ( rx_wfull_o  ) ,
        .awfull       (),
        .wnum         (),
        .wavail_th    ('0),
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
        .rnum_th      ( rx_rnum_th ) ,
        .rfullenough  ( rx_rfullenough )
    );

    logic        rx_dma_idle      ;

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

    assign rx_status = {'0,
        rx_rnum,
        rx_rempty,
        rx_arempty,
        rx_rfullenough,
        rx_dma_idle
    };

    assign rx_rnum_th = rx_dma_data_len[RX_DPTH_W:0];

    // ----------------------------------------------------------
    // Data TX FIFO
    // ----------------------------------------------------------

    // control inpupts
    logic               tx_wena         ;
    logic         [7:0] tx_wdat         ;
    logic [TX_DPTH_W:0] tx_wawail_th    ;
    // status outputs
    logic               tx_wfull        ;
    logic               tx_awfull       ;
    logic [TX_DPTH_W:0] tx_wawail       ;
    logic               tx_wemptyenough ;


`ifdef USB_MSD_TX_GOWIN_FIFO
    generate begin : tx_width_check
        if (TX_DPTH_W != 10) begin
            msd_tx_fifo_addr_width_must_be_11();
        end
    end endgenerate

    logic [TX_DPTH_W:0] tx_almst_full_th ;
    logic               tx_almst_full    ;
    logic [TX_DPTH_W:0] tx_wnum          ;

    assign tx_wemptyenough = ~tx_almst_full ;

    always_ff @(posedge wb_clk_i) begin
        tx_almst_full_th <= (2 ** TX_DPTH_W - tx_wawail_th);
        tx_wawail <= (2 ** TX_DPTH_W - tx_wnum);
    end

    msd_tx_fifo_gw gw_tx_fifo_inst (
        // Wishbone side (write)
        .WrClk         ( wb_clk_i         ),
        .WrReset       ( wb_rst_i         ),
        .WrEn          ( tx_wena          ),
        .Data          ( tx_wdat          ),
        .Full          ( tx_wfull         ),
        .Wnum          ( tx_wnum          ),
        .AlmostFullTh  ( tx_almst_full_th[TX_DPTH_W-1:0] ),
        .Almost_Full   ( tx_almst_full    ),

        // USB side (read)
        .RdClk         ( usb_clk60_i ),
		.RdReset       ( usb_rst_i   ),
		.RdEn          ( tx_rena_i   ),
        .Q             ( tx_rdat_o   ),
        .Empty         ( tx_rempty_o ),
        .Rnum          ( tx_rnum_o   )
	);
`else


    assign dbg[0] = tx_wena;
    assign dbg[1] = tx_wfull;
    assign dbg[2] = tx_wemptyenough;
    assign dbg[3] = tx_rena_i;
    assign dbg[4] = tx_rempty_o;
    assign dbg[5] = tx_dat_source;
    assign dbg[6] = 1'b0;
    assign dbg[7] = 1'b0;


    //assign dbg[0] = tx_rnum_o[0];
    //assign dbg[1] = tx_rnum_o[1];
    //assign dbg[2] = tx_rnum_o[2];
    //assign dbg[3] = tx_rnum_o[3];
    //assign dbg[4] = tx_rnum_o[4];
    //assign dbg[5] = tx_rnum_o[5];
    //assign dbg[6] = tx_rnum_o;
    //assign dbg[7] = tx_wena;

    async_fifo_ft #(
        .DSIZE      ( 8 ),
        .ASIZE      ( TX_DPTH_W )
    ) tx_fifo_inst ( 
        // Wishbone side (write)
        .wclk         ( wb_clk_i        ) ,
        .wrst_n       ( ~wb_rst_i       ) ,
        .winc         ( tx_wena         ) ,
        .wdata        ( tx_wdat         ) ,
        .wfull        ( tx_wfull        ) ,
        .awfull       ( tx_awfull       ) ,
        .wnum         (                 ) ,
        .wavail_th    ( tx_wawail_th    ) ,
        .wemptyenough ( tx_wemptyenough ) ,
        .wavail       ( tx_wawail       ),

        // USB side (read)
        .rclk         ( usb_clk60_i     ) ,
        .rrst_n       ( ~usb_rst_i      ) ,
        .rinc         ( tx_rena_i       ) ,
        .rdata        ( tx_rdat_o       ) ,
        .rempty       ( tx_rempty_o     ) ,
        .rnum         ( tx_rnum_o       ) ,
        .arempty      (                 ),
        .rnum_th      (                 ),
        .rfullenough  (                 )
    );



`endif


    logic       tx_dma_idle ;
    logic       tx_dma_wena ;
    logic [7:0] tx_dma_wdat ;

    wb_32bit_dma_8bit_fifo_writer tx_fifo_writer (
        .clk_i       ( wb_clk_i ) ,
        .rst_i       ( wb_rst_i ) ,

        .start_adr_i ( tx_dma_start_adr ) ,
        .start_i     ( tx_dma_start     ) ,
        .data_len_i  ( tx_dma_data_len  ) ,
        .idle_o      ( tx_dma_idle      ) ,
        .done_stb_o  ( tx_dma_done_o    ) ,

        .wbm         ( wb_dma_int[1]    ) ,

        .wr_dat_o    ( tx_dma_wdat      ) ,
        .wr_ena_o    ( tx_dma_wena      ) ,
        .wr_ready_i  ( tx_wemptyenough  )
    );

    logic       tx_sd_wena ;
    logic [7:0] tx_sd_wdat ;
    localparam [TX_DPTH_W:0] TX_SD_WAWAIL_TH = (TX_DPTH_W)'(4);

    //assign dbg[0] = tx_dma_start;
    //assign dbg[1] = tx_dma_idle;
    //assign dbg[2] = tx_dma_done_o;
    //assign dbg[3] = tx_dma_wena;
    //assign dbg[4] = tx_wemptyenough;
    //assign dbg[5] = tx_wena;
    //assign dbg[6] = tx_wfull;
    //assign dbg[7] = tx_rempty_o;

    wb_32bit_to_8bit_fifo_writer tx_fifo_sdc_bridge (
        .clk_i       ( wb_clk_i ) ,
        .rst_i       ( wb_rst_i ) ,
        .wbs         ( wb_sd    ) ,

        .wr_dat_o    ( tx_sd_wdat      ) ,
        .wr_ena_o    ( tx_sd_wena      ) ,
        .wr_ee_i     ( tx_wemptyenough )
    );

    assign tx_status = {'0,
        tx_wawail       ,
        tx_wfull        ,
        tx_awfull       ,
        tx_wemptyenough ,
        tx_dma_idle
    };

    assign tx_wena      = tx_dat_source ? tx_sd_wena      : tx_dma_wena  ;
    assign tx_wdat      = tx_dat_source ? tx_sd_wdat      : tx_dma_wdat  ;
    assign tx_wawail_th = tx_dat_source ? TX_SD_WAWAIL_TH : tx_dma_data_len[TX_DPTH_W:0];

endmodule

`resetall
