
`default_nettype none

module usb_wishbone_rst_gen (
    input  wire  clk_i  ,
    input  wire  rst_i  ,
    input  wire  req_i  ,
    output wire  rst_o
);
    logic [1:0] rst_r;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
            rst_r <= '1;
        else if (req_i)
            rst_r <= '1;
        else
            rst_r <= {rst_r[0], 1'b0};

    assign rst_o = rst_r[1];

endmodule


module usb_wishbone #(
    parameter [3:0] MSD_ENDP_NUM = 4'd1,
    parameter [3:0] CDC_ENDP_NUM = 4'd2
)(
    // Wishbone
    input  wire            wb_clk_i          ,
    input  wire            wb_rst_i          ,

    wb_if.slv              wb_sfr            ,

    wb_if.slv              wb_msd_sfr        ,
    wb_if.slv              wb_msd_sdc        ,
    wb_if.mst              wb_msd_dma        ,
    output wire            msd_tx_dma_irq_o  ,
    output wire            msd_rx_dma_irq_o  ,
`ifdef USB_CDC_ENA
    wb_if.slv              wb_cdc            ,
    output wire            cdc_rx_irq_o      ,
`endif

    // USB
    input  wire            usb_clk60_i       ,
    input  wire            usb_rst_i         ,
    usb_gowin_if.user      usb               ,
    output wire            usb_rst_o         ,
    output wire            usb_ack_tout_stretched_o         ,
    output wire            usb_ack_received_stretched_o     ,
    output wire            usb_ack_bad_packet_stretched_o    
    ,output wire [15:0] dbg
    
);

    // ----------------------------------------------------------
    // Control regs
    // ----------------------------------------------------------

    wire msd_rst_req        ;
    wire msd_get_max_lun    ;
    wire msd_rps_tx_err     ;

    wire usb_ack_tout       ;
    wire usb_ack_received   ;
    wire usb_ack_bad_packet ;

    wire msd_reset          ;

    usb_wb_ctrl_status_regs ctrl_regs (
        .clk_i             ( wb_clk_i ) ,
        .rst_i             ( wb_rst_i ) ,
        .wbs               ( wb_sfr   ),

        .msd_rst_req_i     ( msd_rst_req     ),
        .msd_get_max_lun_i ( msd_get_max_lun ),
        .msd_rps_tx_err_i  ( msd_rps_tx_err  ),

        .usb_ack_timeout_i      ( usb_ack_tout      ),
        .usb_ack_received_i     ( usb_ack_received  ),
        .usb_ack_bad_packet_i   ( usb_ack_bad_packet),

        .usb_rst_o         ( msd_reset       )
    );

    // ----------------------------------------------------------
    // Generate reset signals
    // ----------------------------------------------------------

    wire usb_rst_req;
    wire usb_rst;
    wire msd_dma_rst;

    synchronizer #(.W(1), .INIT_VAL('0)) rstreq_sync (
        .clk_i  (usb_clk60_i) ,
        .rst_i  (usb_rst_i  ) ,
        .raw_i  (msd_reset  ) ,
        .sync_o (usb_rst_req)
    );

    usb_wishbone_rst_gen usb_rst_gen (
        .clk_i (usb_clk60_i) ,
        .rst_i (usb_rst_i  ) ,
        .req_i (usb_rst_req) ,
        .rst_o (usb_rst    )
    );

    usb_wishbone_rst_gen msd_rst_gen (
        .clk_i (wb_clk_i   ) ,
        .rst_i (wb_rst_i   ) ,
        .req_i (msd_reset  ) ,
        .rst_o (msd_dma_rst)
    );

    assign usb_rst_o = usb_rst;

    // ----------------------------------------------------------
    // Sync ack timeout
    // ----------------------------------------------------------

    wire usb_ack_tout_stretched         ;
    wire usb_ack_received_stretched     ;
    wire usb_ack_bad_packet_stretched   ;

    assign usb_ack_tout_stretched_o       =  usb_ack_tout_stretched      ;
    assign usb_ack_received_stretched_o   =  usb_ack_received_stretched  ;
    assign usb_ack_bad_packet_stretched_o =  usb_ack_bad_packet_stretched;

    
    // N.B. usb.ack_tout can be too narrow (1 cycle) for WB clk domain
    // need pulse stretcher
    pulse_stretcher #(.W(8)) usb_ack_tout_stretcher (
        .clk_i       (usb_clk60_i),
        .rst_i       (usb_rst),
        .pulse_i     (usb.ack_tout),
        .stretched_o (usb_ack_tout_stretched)
    );

    synchronizer #(.W(1), .INIT_VAL('0)) ack_tout_sync_inst (
        .clk_i  ( wb_clk_i     ) ,
        .rst_i  ( wb_rst_i     ) ,
        .raw_i  ( usb_ack_tout_stretched ) ,
        .sync_o ( usb_ack_tout )
    );



    pulse_stretcher #(.W(8)) usb_ack_received_stretcher (
        .clk_i       (usb_clk60_i),
        .rst_i       (usb_rst),
        .pulse_i     (usb.ack_received),
        .stretched_o (usb_ack_received_stretched)
    );

    synchronizer #(.W(1), .INIT_VAL('0)) ack_received_sync_inst (
        .clk_i  ( wb_clk_i     ) ,
        .rst_i  ( wb_rst_i     ) ,
        .raw_i  ( usb_ack_received_stretched ) ,
        .sync_o ( usb_ack_received )
    );




    pulse_stretcher #(.W(8)) usb_ack_bad_packet_stretcher (
        .clk_i       (usb_clk60_i),
        .rst_i       (usb_rst),
        .pulse_i     (usb.ack_bad_packet),
        .stretched_o (usb_ack_bad_packet_stretched)
    );

    synchronizer #(.W(1), .INIT_VAL('0)) ack_bad_packet_sync_inst (
        .clk_i  ( wb_clk_i     ) ,
        .rst_i  ( wb_rst_i     ) ,
        .raw_i  ( usb_ack_bad_packet_stretched ) ,
        .sync_o ( usb_ack_bad_packet )
    );
    // ----------------------------------------------------------
    // Setup Requests Handler
    // ----------------------------------------------------------

    parameter CTRL_TX_ADR_W = 4;

    wire [7:0] ctrl_tx_fifo_wdat;
    wire       ctrl_tx_fifo_wena;
    wire       ctrl_tx_fifo_wfull;

    wire [7:0] ctrl_tx_fifo_rdat;
    wire       ctrl_tx_fifo_rena;
    wire       ctrl_tx_fifo_rempty;

    usb_req_handler req_handler (
        .clk_i   (usb_clk60_i) ,
        .rst_i   (usb_rst    ) ,

        .setup_i (usb.setup) ,
        .rxval_i (usb.rxval) ,
        .rxdat_i (usb.rxdat) ,

        .rsp_tx_dat_o  ( ctrl_tx_fifo_wdat  ) ,
        .rsp_tx_ena_o  ( ctrl_tx_fifo_wena  ) ,
        .rsp_tx_full_i ( ctrl_tx_fifo_wfull ) ,

        .rps_tx_err_o          ( msd_rst_req     ) ,
        .msd_rst_req_o         ( msd_get_max_lun ) ,
        .msd_get_max_lun_req_o ( msd_rps_tx_err  )
    );

    fifo_fwft #(
        .DATA_WIDTH  ( 8             ),
        .DEPTH_WIDTH ( CTRL_TX_ADR_W )
    ) ctrl_tx_fifo (
        .clk     ( usb_clk60_i ),
        .rst     ( usb_rst     ),

        .din     ( ctrl_tx_fifo_wdat   ),
        .wr_en   ( ctrl_tx_fifo_wena   ),
        .full    ( ctrl_tx_fifo_wfull  ),

        .dout    ( ctrl_tx_fifo_rdat   ),
        .rd_en   ( ctrl_tx_fifo_rena   ),
        .empty   ( ctrl_tx_fifo_rempty )
    );

    // ----------------------------------------------------------
    // MSD endpoint RX & TX logic
    // ----------------------------------------------------------

    localparam [11:0] MSD_MAX_LEN = 12'd512;
    // localparam [11:0] MSD_MAX_LEN = 12'd256;
    //localparam [11:0] MSD_MAX_LEN = 12'd64;

    localparam MSD_RX_FIFO_DPTH_W = 10;
    localparam MSD_TX_FIFO_DPTH_W = 10;

    logic [7:0] msd_rx_wdat  ;
    logic       msd_rx_wena  ;
    logic       msd_rx_wfull ;

    logic                        msd_tx_rena   ;
    logic                  [7:0] msd_tx_rdat   ;
    logic                        msd_tx_rempty ;
    logic [MSD_TX_FIFO_DPTH_W:0] msd_tx_rnum   ;

    logic       msd_tx_ena_async;
    logic       msd_tx_ena    ;
    logic       msd_tx_sd_mode;

    usb_msd_wishbone_dma #(
        .RX_DPTH_W (MSD_RX_FIFO_DPTH_W)  ,
        .TX_DPTH_W (MSD_TX_FIFO_DPTH_W)
    ) msd_fifos_inst (
        .wb_clk_i ,
        .wb_rst_i (msd_dma_rst) ,

        .wb_sfr   (wb_msd_sfr),
        .wb_sd    (wb_msd_sdc),
        .wb_dma   (wb_msd_dma),

        .rx_dma_done_o (msd_rx_dma_irq_o),
        .tx_dma_done_o (msd_tx_dma_irq_o),

        // USB Side
        .usb_clk60_i   ,
        .usb_rst_i     ( usb_rst     ),
        // RX FIFO
        .rx_wdat_i     ( msd_rx_wdat  ),
        .rx_wena_i     ( msd_rx_wena  ),
        .rx_wfull_o    ( msd_rx_wfull ),
        // TX FIFO
        .tx_rena_i     ( msd_tx_rena   ),
        .tx_rdat_o     ( msd_tx_rdat   ),
        .tx_rempty_o   ( msd_tx_rempty ),
        .tx_rnum_o     ( msd_tx_rnum   ),
        // Enable TX
        .usb_tx_ena_o    ( msd_tx_ena_async ),
        .usb_tx_sd_dat_o ( msd_tx_sd_mode )
        ,.dbg(dbg)
    );

    synchronizer #(.W(1), .INIT_VAL('0)) wb2usb_sync (
        .clk_i (usb_clk60_i),
        .rst_i (usb_rst),
        .raw_i (msd_tx_ena_async),
        .sync_o(msd_tx_ena)
    );

    // ----------------------------------------------------------
    // MSD endpoint RX & TX logic
    // ----------------------------------------------------------

    localparam [11:0] CDC_MAX_LEN = 12'd32;
    localparam CDC_RX_FIFO_DPTH_W = 9;
    localparam CDC_TX_FIFO_DPTH_W = 9;

    logic [7:0] cdc_rx_wdat  ;
    logic       cdc_rx_wena  ;
    logic       cdc_rx_wfull ;

    logic                        cdc_tx_rena   ;
    logic                  [7:0] cdc_tx_rdat   ;
    logic                        cdc_tx_rempty ;
    logic [CDC_TX_FIFO_DPTH_W:0] cdc_tx_rnum   ;

    logic                        cdc_tx_ena    ;
`ifdef USB_CDC_ENA
    usb_cdc_wishbone #(
        .TX_DPTH_W(CDC_TX_FIFO_DPTH_W) ,
        .RX_DPTH_W(CDC_RX_FIFO_DPTH_W)
    ) cdc_fifo_inst (
        // Wishbone
        .wb_clk_i                       ,
        .wb_rst_i     ( msd_dma_rst   ) ,
        .wbs          ( wb_cdc        ) ,
        .rx_irq_o     ( cdc_rx_irq_o  ) ,
        // USB
        .usb_clk60_i                    ,
        .usb_rst_i    ( usb_rst       ) ,

        // RX FIFO
        .rx_wdat_i    ( cdc_rx_wdat   ) ,
        .rx_wena_i    ( cdc_rx_wena   ) ,
        .rx_wfull_o   ( cdc_rx_wfull  ) ,

        // TX FIFO
        .tx_rena_i    ( cdc_tx_rena   ) ,
        .tx_rdat_o    ( cdc_tx_rdat   ) ,
        .tx_rempty_o  ( cdc_tx_rempty ) ,
        .tx_rnum_o    ( cdc_tx_rnum   ) ,

        .tx_usb_ena_o ( cdc_tx_ena    )
    );
`endif
    // ----------------------------------------------------------
    // USB RX Interface
    // ----------------------------------------------------------

    assign msd_rx_wdat = usb.rxdat;
    assign msd_rx_wena = usb.rxval && (usb.endpt == MSD_ENDP_NUM);

    assign cdc_rx_wdat = usb.rxdat;
    assign cdc_rx_wena = usb.rxval && (usb.endpt == CDC_ENDP_NUM);

    always_ff @(posedge usb_clk60_i, posedge usb_rst) begin
        if (usb_rst)
            usb.rxrdy <= 1'b0;
        else begin
            if (usb.endpt == MSD_ENDP_NUM)
                usb.rxrdy <= !msd_rx_wfull;
            else if (usb.endpt == CDC_ENDP_NUM)
                usb.rxrdy <= !cdc_rx_wfull;
            else
                usb.rxrdy <= 1'b0;
        end
    end

    // ----------------------------------------------------------
    // USB TX Interface
    // ----------------------------------------------------------

    assign ctrl_tx_fifo_rena = usb.txpop && (usb.endpt == '0);

    assign msd_tx_rena = usb.txpop && (usb.endpt == MSD_ENDP_NUM);
    assign cdc_tx_rena = usb.txpop && (usb.endpt == CDC_ENDP_NUM);

    always_comb begin
        if (usb.endpt == '0)
            usb.txdat = ctrl_tx_fifo_rdat;
        else if (usb.endpt == MSD_ENDP_NUM)
            usb.txdat = msd_tx_rdat;
        else if (usb.endpt == CDC_ENDP_NUM)
            usb.txdat = cdc_tx_rdat;
        else
            usb.txdat = '0;
    end

    always_ff @(posedge usb_clk60_i, posedge usb_rst) begin
        if (usb_rst) begin
            usb.txcork <= 1'b1;
        end
        else begin
            if (usb.endpt == 4'b0 && !ctrl_tx_fifo_rempty)
                usb.txcork <= 1'b0;
            else if (usb.endpt == MSD_ENDP_NUM) begin
                usb.txcork <= !(!msd_tx_rempty && msd_tx_ena);
                // if (msd_tx_sd_mode) begin
                //     usb.txcork <= !(msd_tx_rnum >= MSD_MAX_LEN);
                // end else begin
                //     usb.txcork <= !(!msd_tx_rempty && msd_tx_ena);
                // end
            end else if (usb.endpt == CDC_ENDP_NUM) begin
                usb.txcork <= !(!cdc_tx_rempty && cdc_tx_ena);
            end else
                usb.txcork <= 1'b1;
        end
    end

    always_ff @(posedge usb_clk60_i, posedge usb_rst) begin
        if (usb_rst)
            usb.txdat_len <= '0;
        else if (!usb.txact) begin
            if (usb.endpt == '0) begin
                usb.txdat_len <= 1;
            end
            else if (usb.endpt == MSD_ENDP_NUM) begin
                usb.txdat_len <= (msd_tx_rnum >= MSD_MAX_LEN)
                               ?  MSD_MAX_LEN
                               :  msd_tx_rnum;
                // if (msd_tx_sd_mode) begin
                //     usb.txdat_len <= MSD_MAX_LEN;
                // end else begin
                //     usb.txdat_len <= (msd_tx_rnum >= MSD_MAX_LEN)
                //                    ?  MSD_MAX_LEN
                //                    :  msd_tx_rnum;
                // end
            end
            else if (usb.endpt == CDC_ENDP_NUM) begin
                usb.txdat_len <= (cdc_tx_rnum >= CDC_MAX_LEN)
                               ?  CDC_MAX_LEN
                               :  cdc_tx_rnum;
            end
        end
    end

    always_ff @(posedge usb_clk60_i, posedge usb_rst) begin
        if (usb_rst) begin
            usb.txval <= 1'b0;
        end else begin
            if (usb.endpt == 4'b0)
                usb.txval <= !ctrl_tx_fifo_rempty;
            // else if (usb.endpt == MSD_ENDP_NUM)
            //     usb.txval <= !msd_tx_rempty && msd_tx_ena;
            else
                usb.txval <= 1'b0;
        end
    end

endmodule

`resetall
