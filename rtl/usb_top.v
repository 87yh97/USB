localparam WB_DW                        = 6'd32;
localparam WB_AW                        = 6'd32;
localparam WB_SELW                      = WB_DW/8;
localparam WB_WRITE_CYCLE               = 1'b1;
localparam WB_READ_CYCLE                = 1'b0;
localparam USB_RX_READY                 = 1'b1;
localparam USB_RX_NOT_READY             = 1'b0;
localparam USB_TX_READY                 = 1'b0;
localparam USB_TX_NOT_READY             = 1'b1;
localparam USB_CONTROL_DATA_READY       = 1'b1;
localparam USB_CONTROL_DATA_NOT_READY   = 1'b0;
localparam CDC_WB_MODE                  = 1'b0;
localparam CDC_USB_MODE                 = 1'b1;
localparam MSC_WB_MODE                  = 1'b0;
localparam MSC_USB_MODE                 = 1'b1;
localparam CONTROL_WB_MODE              = 1'b0;
localparam CONTROL_USB_MODE             = 1'b1;

localparam MAX_PACKET_SIZE              = 12'd512;
localparam CONTROL_MAX_PACKET_SIZE      = 12'd64;

localparam CONTROL_ENDPOINT             = 4'd0;
localparam CDC_ENDPOINT                 = 4'd2;
localparam MSC_ENDPOINT                 = 4'd5;


localparam CDC_ADDR             = 16'h0000;
localparam CDC_RDA_ADDR         = 16'h0004; //CDC receive data available, addr to check if there is something to read from usb
localparam CDC_TDA_ADDR         = 16'h0008; //CDC transmit data available, addr to check if there is place to write to usb
localparam CDC_MODE_ADDR        = 16'h0034;

localparam MSC_ADDR             = 16'h000C;
localparam MSC_RDA_ADDR         = 16'h0010; //MSC receive data available, addr to check if there is something to read from usb
localparam MSC_TDA_ADDR         = 16'h0014; //MSC transmit data available, addr to check if there is place to write to usb
localparam MSC_MODE_ADDR        = 16'h0018; //Used for switching msc module into CPU->FIFO/FIFO->USB transmit mode

localparam CONTROL_ADDR         = 16'h001C;
localparam CONTROL_RDA_ADDR     = 16'h0020;
localparam CONTROL_TDA_ADDR     = 16'h0024;
localparam CONTROL_MODE_ADDR    = 16'h0028;

localparam SETUP_ADDR         = 16'h002C;
localparam SETUP_RDA_ADDR     = 16'h0030;

//Сначала проверяем нет ли передачи, потом переводим в режим WB и потом передаем данные, потом возвращаем обратно в USB режим

module usb_device(
     input  wire                    i_fclk_480M
    ,input  wire                    i_fclk_60M
    ,input  wire                    pll_locked

    ,input  wire                    i_wb_clk
    ,input  wire                    i_wb_reset

    ,input  wire                    i_wb_cyc
    ,input  wire                    i_wb_stb
    ,input  wire                    i_wb_we
    ,input  wire     [WB_AW-1:0]    i_wb_addr
    ,input  wire     [WB_DW-1:0]    i_wb_data
    ,input  wire     [WB_SELW-1:0]  i_wb_sel
    ,output wire                    o_wb_stall //not used currently
    ,output reg                     o_wb_ack
    ,output reg      [WB_DW-1:0]    o_wb_data

    ,inout  wire                    usb_dxp_io
    ,inout  wire                    usb_dxn_io
    ,input  wire                    usb_rxdp_i
    ,input  wire                    usb_rxdn_i
    ,output wire                    usb_pullup_en_o
    ,inout  wire                    usb_term_dp_io
    ,inout  wire                    usb_term_dn_io

    ,output wire                    usb_hdlc
);





wire [WB_DW/2-1:0] addr_trunc = i_wb_addr[WB_DW/2-1:0];

wire [1:0]  PHY_XCVRSELECT;
wire        PHY_TERMSELECT;
wire [1:0]  PHY_OPMODE;
wire [1:0]  PHY_LINESTATE;
wire        PHY_TXVALID;
wire        PHY_TXREADY;
wire        PHY_RXVALID;
wire        PHY_RXACTIVE;
wire        PHY_RXERROR;
wire [7:0]  PHY_DATAIN;
wire [7:0]  PHY_DATAOUT;
wire        PHY_RESET;


wire [9:0]  DESCROM_RADDR;
wire [7:0]  DESCROM_RDAT;
wire [9:0]  DESC_DEV_ADDR;
wire [7:0]  DESC_DEV_LEN;
wire [9:0]  DESC_QUAL_ADDR;
wire [7:0]  DESC_QUAL_LEN;
wire [9:0]  DESC_FSCFG_ADDR;
wire [7:0]  DESC_FSCFG_LEN;
wire [9:0]  DESC_HSCFG_ADDR;
wire [7:0]  DESC_HSCFG_LEN;
wire [9:0]  DESC_OSCFG_ADDR;
wire [9:0]  DESC_STRLANG_ADDR;
wire [9:0]  DESC_STRVENDOR_ADDR;
wire [7:0]  DESC_STRVENDOR_LEN;
wire [9:0]  DESC_STRPRODUCT_ADDR;
wire [7:0]  DESC_STRPRODUCT_LEN;
wire [9:0]  DESC_STRSERIAL_ADDR;
wire [7:0]  DESC_STRSERIAL_LEN;
wire        DESCROM_HAVE_STRINGS;

wire        reset;
reg  [7:0]  usb_txdat;
reg         usb_txcork;
reg         usb_txval;
wire        usb_txpop;
wire        usb_txact;
wire [7:0]  usb_rxdat;
wire        usb_rxval;
wire        usb_rxact;
reg         usb_rxrdy;
wire [3:0]  endpt_sel;
wire        setup_active;
wire        setup_val;
wire [7:0]  setup_data;
reg         endpt0_send;
reg  [7:0]  endpt0_dat;
reg  [11:0] txdat_len;



wire [7:0]  cdc_rx_fifo_wr_data;
wire [7:0]  cdc_rx_fifo_rd_data;
wire        cdc_rx_fifo_wren;
wire        cdc_rx_fifo_rden;
wire [9:0]  cdc_rx_fifo_wnum;
wire [9:0]  cdc_rx_fifo_rnum;
wire        cdc_rx_fifo_empty;
wire        cdc_rx_fifo_full;
wire        cdc_rx_fifo_almost_empty;

reg  [7:0]  cdc_tx_fifo_wr_data;
wire [7:0]  cdc_tx_fifo_rd_data;
wire        cdc_tx_fifo_wren;
wire        cdc_tx_fifo_rden;
wire [9:0]  cdc_tx_fifo_wnum;
wire [9:0]  cdc_tx_fifo_rnum;
wire        cdc_tx_fifo_empty;
wire        cdc_tx_fifo_full;
wire        cdc_tx_fifo_almost_full;

reg         cdc_mode_wire;
reg         cdc_mode;


wire [7:0]  msc_rx_fifo_wr_data;
wire [7:0]  msc_rx_fifo_rd_data;
wire        msc_rx_fifo_wren;
wire        msc_rx_fifo_rden;
wire [9:0]  msc_rx_fifo_wnum;
wire [9:0]  msc_rx_fifo_rnum;
wire        msc_rx_fifo_empty;
wire        msc_rx_fifo_full;
wire        msc_rx_fifo_almost_empty;

reg  [7:0]  msc_tx_fifo_wr_data;
wire [7:0]  msc_tx_fifo_rd_data;
wire        msc_tx_fifo_wren;
wire        msc_tx_fifo_rden;
wire [9:0]  msc_tx_fifo_wnum;
wire [9:0]  msc_tx_fifo_rnum;
wire        msc_tx_fifo_empty;
wire        msc_tx_fifo_full;
wire        msc_tx_fifo_almost_full;

reg         msc_mode_wire;
reg         msc_mode;

reg  [7:0]  control_tx_fifo_wr_data;
wire [7:0]  control_tx_fifo_rd_data;
wire        control_tx_fifo_wren;
wire        control_tx_fifo_rden;
wire [6:0]  control_tx_fifo_wnum;
wire [6:0]  control_tx_fifo_rnum;
wire        control_tx_fifo_empty;
wire        control_tx_fifo_full;
wire        control_tx_fifo_almost_full;

wire [7:0]  control_rx_fifo_wr_data;
wire [7:0]  control_rx_fifo_rd_data;
wire        control_rx_fifo_wren;
wire        control_rx_fifo_rden;
wire [6:0]  control_rx_fifo_wnum;
wire [6:0]  control_rx_fifo_rnum;
wire        control_rx_fifo_empty;
wire        control_rx_fifo_full;
wire        control_rx_fifo_almost_empty;

reg         control_mode_wire;
reg         control_mode;

wire [7:0]  setup_rx_fifo_wr_data;
wire [7:0]  setup_rx_fifo_rd_data;
wire        setup_rx_fifo_wren;
wire        setup_rx_fifo_rden;
wire [6:0]  setup_rx_fifo_wnum;
wire [6:0]  setup_rx_fifo_rnum;
wire        setup_rx_fifo_empty;
wire        setup_rx_fifo_full;
wire        setup_rx_fifo_almost_empty;

assign reset = !i_wb_reset | !pll_locked;

assign usb_hdlc =   (i_wb_stb && i_wb_cyc && o_wb_ack) && (
                    (cdc_rx_fifo_almost_empty           && i_wb_we == WB_READ_CYCLE      && addr_trunc == CDC_ADDR          ) ||
                    (cdc_tx_fifo_almost_full            && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == CDC_ADDR          ) ||
                    (cdc_mode == CDC_USB_MODE           && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == CDC_ADDR          ) ||
                    (usb_txact                          && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == CDC_MODE_ADDR     ) ||
                    (msc_rx_fifo_almost_empty           && i_wb_we == WB_READ_CYCLE      && addr_trunc == MSC_ADDR          ) ||
                    (msc_tx_fifo_almost_full            && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == MSC_ADDR          ) ||
                    (msc_mode == MSC_USB_MODE           && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == MSC_ADDR          ) ||
                    (usb_txact                          && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == MSC_MODE_ADDR     ) ||
                    (control_rx_fifo_almost_empty       && i_wb_we == WB_READ_CYCLE      && addr_trunc == CONTROL_ADDR      ) ||
                    (control_tx_fifo_almost_full        && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == CONTROL_ADDR      ) ||
                    (control_mode == CONTROL_USB_MODE   && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == CONTROL_ADDR      ) ||
                    (usb_txact                          && i_wb_we == WB_WRITE_CYCLE     && addr_trunc == CONTROL_MODE_ADDR ) ||
                    (setup_rx_fifo_almost_empty         && i_wb_we == WB_READ_CYCLE      && addr_trunc == SETUP_ADDR        )
                    );

//===================MSC MODE CONTROL===================
always @(posedge i_wb_clk) begin

    if(reset) begin
        cdc_mode        <= CDC_USB_MODE;
        msc_mode        <= MSC_USB_MODE;
        control_mode    <= CONTROL_USB_MODE;

    end else if (i_wb_stb  && i_wb_cyc && o_wb_ack && !usb_txact && i_wb_we == WB_WRITE_CYCLE) begin
        if (addr_trunc == CDC_MODE_ADDR)
            cdc_mode <= cdc_mode_wire;
        else
        if (addr_trunc == MSC_MODE_ADDR)
            msc_mode <= msc_mode_wire;
        else
        if (addr_trunc == CONTROL_MODE_ADDR)
            control_mode <= control_mode_wire;
    end

end


//===================WB ACK Control===================
always @(posedge i_wb_clk, negedge i_wb_stb, negedge i_wb_cyc) begin // СДЕЛАТЬ РАЗДЕЛЕНИЕ ПО WE

        if (!i_wb_stb || !i_wb_cyc)
            o_wb_ack <= 1'b0;
        else begin
            if          (addr_trunc == CDC_ADDR         ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == CDC_RDA_ADDR     ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == CDC_TDA_ADDR     ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == CDC_MODE_ADDR    ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == MSC_ADDR         ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == MSC_RDA_ADDR     ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == MSC_TDA_ADDR     ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == MSC_MODE_ADDR    ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == CONTROL_ADDR     ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == CONTROL_RDA_ADDR ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == CONTROL_TDA_ADDR ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == CONTROL_MODE_ADDR) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == SETUP_ADDR       ) begin
                o_wb_ack <= 1'b1;
            end else if (addr_trunc == SETUP_RDA_ADDR   ) begin
                o_wb_ack <= 1'b1;
            end else begin
                o_wb_ack <= 1'b1;
            end
        end
end


//===================WB TX Data Control===================
always @(*) begin
    cdc_tx_fifo_wr_data = 8'b0; //changed on 14.05.2022 CHECK
    msc_tx_fifo_wr_data = 8'b0;
    control_tx_fifo_wr_data = 8'b0;
    cdc_mode_wire = 1'b0;
    msc_mode_wire = 1'b0;
    control_mode_wire = 1'b0;

    if(i_wb_we == WB_WRITE_CYCLE && i_wb_stb && i_wb_cyc) begin
        case(addr_trunc)
            CDC_ADDR : begin
                case (i_wb_sel)
                    4'b0001 : cdc_tx_fifo_wr_data = i_wb_data[7:0];
                    4'b0010 : cdc_tx_fifo_wr_data = i_wb_data[15:8];
                    4'b0100 : cdc_tx_fifo_wr_data = i_wb_data[23:16];
                    4'b1000 : cdc_tx_fifo_wr_data = i_wb_data[31:24];
                    default : cdc_tx_fifo_wr_data = i_wb_data[7:0];
                endcase
            end

            CDC_RDA_ADDR : begin

            end

            CDC_TDA_ADDR : begin

            end

            CDC_MODE_ADDR : begin
                cdc_mode_wire = |i_wb_data[31:0];
            end

            MSC_ADDR : begin
                case (i_wb_sel)
                    4'b0001 : msc_tx_fifo_wr_data = i_wb_data[7:0];
                    4'b0010 : msc_tx_fifo_wr_data = i_wb_data[15:8];
                    4'b0100 : msc_tx_fifo_wr_data = i_wb_data[23:16];
                    4'b1000 : msc_tx_fifo_wr_data = i_wb_data[31:24];
                    default : msc_tx_fifo_wr_data = i_wb_data[7:0];
                endcase
            end

            MSC_RDA_ADDR : begin

            end

            MSC_TDA_ADDR : begin

            end

            MSC_MODE_ADDR : begin
                msc_mode_wire = |i_wb_data[31:0];
            end

            CONTROL_ADDR : begin
                case (i_wb_sel)
                    4'b0001 : control_tx_fifo_wr_data = i_wb_data[7:0];
                    4'b0010 : control_tx_fifo_wr_data = i_wb_data[15:8];
                    4'b0100 : control_tx_fifo_wr_data = i_wb_data[23:16];
                    4'b1000 : control_tx_fifo_wr_data = i_wb_data[31:24];
                    default : control_tx_fifo_wr_data = i_wb_data[7:0];
                endcase
            end

            CONTROL_RDA_ADDR : begin

            end

            CONTROL_TDA_ADDR : begin

            end

            CONTROL_MODE_ADDR : begin
                control_mode_wire = |i_wb_data[31:0];
            end

            SETUP_ADDR : begin

            end

            SETUP_RDA_ADDR : begin

            end

            default : begin

            end

        endcase
    end
end

assign cdc_tx_fifo_wren = i_wb_stb  && i_wb_cyc && (i_wb_we == WB_WRITE_CYCLE) && !cdc_tx_fifo_full
                                    && o_wb_ack && (addr_trunc == CDC_ADDR)    && (cdc_mode == CDC_WB_MODE);

assign cdc_tx_fifo_rden = usb_txpop && usb_txact && (endpt_sel == CDC_ENDPOINT) && !cdc_tx_fifo_empty && (cdc_mode == CDC_USB_MODE);


FIFO_HS_Top cdc_tx_fifo(
    .Data           (cdc_tx_fifo_wr_data    ), //input [7:0] FIFO <-- WB
    .Reset          (reset                  ), //input Reset
    .WrClk          (i_wb_clk               ), //input WrClk
    .RdClk          (i_fclk_60M             ), //input RdClk
    .WrEn           (cdc_tx_fifo_wren       ), //input WrEn
    .RdEn           (cdc_tx_fifo_rden       ), //input RdEn
    .Wnum           (cdc_tx_fifo_wnum       ), //output [9:0] Wnum
    .Rnum           (cdc_tx_fifo_rnum       ), //output [9:0] Rnum
    .Almost_Empty   (                       ), //output Almost_Empty
    .Almost_Full    (cdc_tx_fifo_almost_full), //output Almost_Full
    .Q              (cdc_tx_fifo_rd_data    ), //output [7:0] FIFO --> USB
    .Empty          (cdc_tx_fifo_empty      ), //output Empty
    .Full           (cdc_tx_fifo_full       )  //output Full
);

assign msc_tx_fifo_wren = i_wb_stb  && i_wb_cyc     && (i_wb_we == WB_WRITE_CYCLE) && !msc_tx_fifo_full
                                    && o_wb_ack     && (addr_trunc == MSC_ADDR)    && (msc_mode == MSC_WB_MODE);

assign msc_tx_fifo_rden = usb_txpop && usb_txact && (endpt_sel == MSC_ENDPOINT) && !msc_tx_fifo_empty && (msc_mode == MSC_USB_MODE);

FIFO_HS_Top msc_tx_fifo(
    .Data           (msc_tx_fifo_wr_data    ), //input [7:0] FIFO <-- WB
    .Reset          (reset                  ), //input Reset
    .WrClk          (i_wb_clk               ), //input WrClk
    .RdClk          (i_fclk_60M             ), //input RdClk
    .WrEn           (msc_tx_fifo_wren       ), //input WrEn
    .RdEn           (msc_tx_fifo_rden       ), //input RdEn
    .Wnum           (msc_tx_fifo_wnum       ), //output [9:0] Wnum
    .Rnum           (msc_tx_fifo_rnum       ), //output [9:0] Rnum
    .Almost_Empty   (                       ), //output Almost_Empty
    .Almost_Full    (msc_tx_fifo_almost_full), //output Almost_Full
    .Q              (msc_tx_fifo_rd_data    ), //output [7:0] FIFO --> USB
    .Empty          (msc_tx_fifo_empty      ), //output Empty
    .Full           (msc_tx_fifo_full       )  //output Full
);

assign control_tx_fifo_wren = i_wb_stb  && i_wb_cyc     && (i_wb_we == WB_WRITE_CYCLE)  && !control_tx_fifo_full
                                        && o_wb_ack     && (addr_trunc == CONTROL_ADDR) && (control_mode == CONTROL_WB_MODE);

assign control_tx_fifo_rden = usb_txpop && usb_txact && (endpt_sel == CONTROL_ENDPOINT) && !control_tx_fifo_empty && (control_mode == CONTROL_USB_MODE);

FIFO_HS_Top_control control_tx_fifo(
    .Data           (control_tx_fifo_wr_data    ), //input [7:0] FIFO <-- WB
    .Reset          (reset                      ), //input Reset
    .WrClk          (i_wb_clk                   ), //input WrClk
    .RdClk          (i_fclk_60M                 ), //input RdClk
    .WrEn           (control_tx_fifo_wren       ), //input WrEn
    .RdEn           (control_tx_fifo_rden       ), //input RdEn
    .Wnum           (control_tx_fifo_wnum       ), //output [9:0] Wnum
    .Rnum           (control_tx_fifo_rnum       ), //output [9:0] Rnum
    .Almost_Empty   (                           ), //output Almost_Empty
    .Almost_Full    (control_tx_fifo_almost_full), //output Almost_Full
    .Q              (control_tx_fifo_rd_data    ), //output [7:0] FIFO --> USB
    .Empty          (control_tx_fifo_empty      ), //output Empty
    .Full           (control_tx_fifo_full       )  //output Full
);

//===================WB RX DATA CONTROL===================

always @(*) begin
    o_wb_data [31:0] = 32'b0;
    if (i_wb_we == WB_READ_CYCLE && i_wb_stb && i_wb_cyc) begin
        case(addr_trunc)
            CDC_ADDR : begin
                case (i_wb_sel)
                    4'b0001 : o_wb_data[7:0]   = cdc_rx_fifo_rd_data;
                    4'b0010 : o_wb_data[15:8]  = cdc_rx_fifo_rd_data;
                    4'b0100 : o_wb_data[23:16] = cdc_rx_fifo_rd_data;
                    4'b1000 : o_wb_data[31:24] = cdc_rx_fifo_rd_data;
                    default : o_wb_data[7:0]   = cdc_rx_fifo_rd_data;
                endcase
            end

            CDC_RDA_ADDR : begin
                o_wb_data[0] = !cdc_rx_fifo_empty && !usb_rxact;
            end

            CDC_TDA_ADDR : begin
                o_wb_data[0] = cdc_tx_fifo_empty  && !usb_txact; // before adding mode - !cdc_tx_fifo_full  && !usb_txact;
            end

            CDC_MODE_ADDR : begin
                o_wb_data[0] = cdc_mode;
            end

            MSC_ADDR : begin
                case (i_wb_sel)
                    4'b0001 : o_wb_data[7:0]   = msc_rx_fifo_rd_data;
                    4'b0010 : o_wb_data[15:8]  = msc_rx_fifo_rd_data;
                    4'b0100 : o_wb_data[23:16] = msc_rx_fifo_rd_data;
                    4'b1000 : o_wb_data[31:24] = msc_rx_fifo_rd_data;
                    default : o_wb_data[7:0]   = msc_rx_fifo_rd_data;
                endcase
            end

            MSC_RDA_ADDR : begin
                o_wb_data[0] = !msc_rx_fifo_empty && !usb_rxact;
            end

            MSC_TDA_ADDR : begin
                o_wb_data[0] =  msc_tx_fifo_empty && !usb_txact; //We may send data to the tx_fifo ONLY if previous packet was fully sent to USB
            end

            MSC_MODE_ADDR : begin
                o_wb_data[0] = msc_mode;
            end

            CONTROL_ADDR : begin
                case (i_wb_sel)
                    4'b0001 : o_wb_data[7:0]   = control_rx_fifo_rd_data;
                    4'b0010 : o_wb_data[15:8]  = control_rx_fifo_rd_data;
                    4'b0100 : o_wb_data[23:16] = control_rx_fifo_rd_data;
                    4'b1000 : o_wb_data[31:24] = control_rx_fifo_rd_data;
                    default : o_wb_data[7:0]   = control_rx_fifo_rd_data;
                endcase
            end

            CONTROL_RDA_ADDR : begin
                o_wb_data[0] = !control_rx_fifo_empty && !usb_rxact;
            end

            CONTROL_TDA_ADDR : begin
                o_wb_data[0] =  control_tx_fifo_empty && !usb_txact; //We may send data to the tx_fifo ONLY if previous packet was fully sent to USB
            end

            CONTROL_MODE_ADDR : begin
                o_wb_data[0] = control_mode;
            end

            SETUP_ADDR : begin
                case (i_wb_sel)
                    4'b0001 : o_wb_data[7:0]   = setup_rx_fifo_rd_data;
                    4'b0010 : o_wb_data[15:8]  = setup_rx_fifo_rd_data;
                    4'b0100 : o_wb_data[23:16] = setup_rx_fifo_rd_data;
                    4'b1000 : o_wb_data[31:24] = setup_rx_fifo_rd_data;
                    default : o_wb_data[7:0]   = setup_rx_fifo_rd_data;
                endcase
            end

            SETUP_RDA_ADDR : begin
                o_wb_data[0] = !setup_rx_fifo_empty && !setup_active;
            end

            default : begin

            end

        endcase
    end
end


assign cdc_rx_fifo_wr_data = usb_rxdat;

assign cdc_rx_fifo_wren = usb_rxval && usb_rxact && (endpt_sel == CDC_ENDPOINT) && !cdc_rx_fifo_full;
assign cdc_rx_fifo_rden = i_wb_stb && i_wb_cyc && (i_wb_we == WB_READ_CYCLE) && o_wb_ack && (addr_trunc == CDC_ADDR) && !cdc_rx_fifo_empty;//CHEEEEEEEEEEEEEEECK


FIFO_HS_Top cdc_rx_fifo(
    .Data           (cdc_rx_fifo_wr_data        ), //input [7:0] FIFO <-- USB
    .Reset          (reset                      ), //input Reset
    .WrClk          (i_fclk_60M                 ), //input WrClk
    .RdClk          (i_wb_clk                   ), //input RdClk
    .WrEn           (cdc_rx_fifo_wren           ), //input WrEn
    .RdEn           (cdc_rx_fifo_rden           ), //input RdEn
    .Wnum           (cdc_rx_fifo_wnum           ), //output [10:0] Wnum //!Allow host to read this num to know how much bytes it should take
    .Almost_Empty   (cdc_rx_fifo_almost_empty   ), //output Almost_Empty
    .Almost_Full    (                           ), //output Almost_Full
    .Rnum           (cdc_rx_fifo_rnum           ), //output [10:0] Rnum
    .Q              (cdc_rx_fifo_rd_data        ), //output [7:0] FIFO --> WB
    .Empty          (cdc_rx_fifo_empty          ), //output Empty
    .Full           (cdc_rx_fifo_full           )  //output Full
);

assign msc_rx_fifo_wr_data = usb_rxdat;

assign msc_rx_fifo_wren = usb_rxval && usb_rxact && (endpt_sel == MSC_ENDPOINT) && !msc_rx_fifo_full;
assign msc_rx_fifo_rden = i_wb_stb && i_wb_cyc && (i_wb_we == WB_READ_CYCLE) && o_wb_ack && (addr_trunc == MSC_ADDR) && !msc_rx_fifo_empty;

FIFO_HS_Top msc_rx_fifo(
    .Data           (msc_rx_fifo_wr_data        ), //input [7:0] FIFO <-- WB
    .Reset          (reset                      ), //input Reset
    .WrClk          (i_fclk_60M                 ), //input WrClk
    .RdClk          (i_wb_clk                   ), //input RdClk
    .WrEn           (msc_rx_fifo_wren           ), //input WrEn
    .RdEn           (msc_rx_fifo_rden           ), //input RdEn
    .Wnum           (msc_rx_fifo_wnum           ), //output [10:0] Wnum
    .Rnum           (msc_rx_fifo_rnum           ), //output [10:0] Rnum
    .Almost_Empty   (msc_rx_fifo_almost_empty   ), //output Almost_Empty
    .Almost_Full    (                           ), //output Almost_Full
    .Q              (msc_rx_fifo_rd_data        ), //output [7:0] FIFO --> USB
    .Empty          (msc_rx_fifo_empty          ), //output Empty
    .Full           (msc_rx_fifo_full           )  //output Full
);

assign control_rx_fifo_wr_data = usb_rxdat;

assign control_rx_fifo_wren = usb_rxval && usb_rxact && (endpt_sel == CONTROL_ENDPOINT) && !control_rx_fifo_full;
assign control_rx_fifo_rden = i_wb_stb && i_wb_cyc && (i_wb_we == WB_READ_CYCLE) && o_wb_ack && (addr_trunc == CONTROL_ADDR) && !control_rx_fifo_empty;

FIFO_HS_Top_control control_rx_fifo(
    .Data           (control_rx_fifo_wr_data        ), //input [7:0] FIFO <-- WB
    .Reset          (reset                          ), //input Reset
    .WrClk          (i_fclk_60M                     ), //input WrClk
    .RdClk          (i_wb_clk                       ), //input RdClk
    .WrEn           (control_rx_fifo_wren           ), //input WrEn
    .RdEn           (control_rx_fifo_rden           ), //input RdEn
    .Wnum           (control_rx_fifo_wnum           ), //output [10:0] Wnum
    .Rnum           (control_rx_fifo_rnum           ), //output [10:0] Rnum
    .Almost_Empty   (control_rx_fifo_almost_empty   ), //output Almost_Empty
    .Almost_Full    (                               ), //output Almost_Full
    .Q              (control_rx_fifo_rd_data        ), //output [7:0] FIFO --> USB
    .Empty          (control_rx_fifo_empty          ), //output Empty
    .Full           (control_rx_fifo_full           )  //output Full
);

assign setup_rx_fifo_wr_data = usb_rxdat;

assign setup_rx_fifo_wren = usb_rxval && setup_active && (endpt_sel == CONTROL_ENDPOINT) && !setup_rx_fifo_full;
assign setup_rx_fifo_rden = i_wb_stb && i_wb_cyc && (i_wb_we == WB_READ_CYCLE) && o_wb_ack && (addr_trunc == SETUP_ADDR) && !setup_rx_fifo_empty;

FIFO_HS_Top_setup setup_rx_fifo(
    .Data           (setup_rx_fifo_wr_data          ), //input [7:0] FIFO <-- WB
    .Reset          (reset                          ), //input Reset
    .WrClk          (i_fclk_60M                     ), //input WrClk
    .RdClk          (i_wb_clk                       ), //input RdClk
    .WrEn           (setup_rx_fifo_wren             ), //input WrEn
    .RdEn           (setup_rx_fifo_rden             ), //input RdEn
    .Wnum           (setup_rx_fifo_wnum             ), //output [10:0] Wnum
    .Rnum           (setup_rx_fifo_rnum             ), //output [10:0] Rnum
    .Almost_Empty   (setup_rx_fifo_almost_empty     ), //output Almost_Empty
    .Almost_Full    (                               ), //output Almost_Full
    .Q              (setup_rx_fifo_rd_data          ), //output [7:0] FIFO --> USB
    .Empty          (setup_rx_fifo_empty            ), //output Empty
    .Full           (setup_rx_fifo_full             )  //output Full
);

//=============================RX DATA READY=============================
always@(posedge i_fclk_60M) begin
    if (reset)
        usb_rxrdy <= USB_RX_NOT_READY;
    else begin
        if (usb_rxrdy == USB_RX_READY && (usb_rxact || (setup_active && endpt_sel == CONTROL_ENDPOINT))) //might be a problem
           usb_rxrdy <= USB_RX_READY;
        else
        if (endpt_sel == CDC_ENDPOINT      && !cdc_rx_fifo_full ) //For some reason CDC terminals crash if receive NAK
            usb_rxrdy <= USB_RX_READY;
        else
        if (endpt_sel == MSC_ENDPOINT      && !msc_rx_fifo_full     && msc_rx_fifo_empty) //maybe remove check on full
            usb_rxrdy <= USB_RX_READY;
        else
        if (endpt_sel == CONTROL_ENDPOINT  && !control_rx_fifo_full && control_rx_fifo_empty) //maybe remove check on full
            usb_rxrdy <= USB_RX_READY;
        else
        if (endpt_sel == CONTROL_ENDPOINT  && !setup_rx_fifo_full) //maybe remove check on full
            usb_rxrdy <= USB_RX_READY;
        else
            usb_rxrdy <= USB_RX_NOT_READY;
    end
end

//assign usb_rxrdy = USB_RX_NOT_READY;


//=============================TX DATA CORK===============================
always @(posedge i_fclk_60M) begin
    if (reset) begin
        usb_txcork <= USB_TX_NOT_READY;
        usb_txval <= USB_CONTROL_DATA_NOT_READY;
    end
    else begin
        usb_txval <= USB_CONTROL_DATA_NOT_READY;
        if (endpt_sel == CDC_ENDPOINT       && !cdc_tx_fifo_empty       &&  cdc_mode == CDC_USB_MODE)
            usb_txcork <= USB_TX_READY;                                                                             //!!ADD CHECK OF MAX PACKET SIZE TRANSFER (>512 bytes)
        else
        if (endpt_sel == MSC_ENDPOINT       && !msc_tx_fifo_empty       &&  msc_mode == MSC_USB_MODE)
            usb_txcork <= USB_TX_READY;
        else
        if (endpt_sel == CONTROL_ENDPOINT   && !control_tx_fifo_empty   &&  control_mode == CONTROL_USB_MODE) begin
            usb_txcork <= USB_TX_READY;
            usb_txval <= USB_CONTROL_DATA_READY;
        end
        else
            usb_txcork <= USB_TX_NOT_READY;
    end
end



//=============================TX DATA LENGTH=============================
always @(posedge i_fclk_60M) begin //should be i_fclk_60M but i changed it on i_wb_clk because the design wasnt fast enough which resulted in red slacks
    if (reset)
        txdat_len <= 12'd0;
    else if (usb_txact)
        txdat_len <= txdat_len;
    else begin
        if (endpt_sel == CDC_ENDPOINT) begin
            if (cdc_tx_fifo_wnum <= MAX_PACKET_SIZE)
                txdat_len <= cdc_tx_fifo_wnum;
            else
                txdat_len <= MAX_PACKET_SIZE;
        end else

        if (endpt_sel == MSC_ENDPOINT) begin
            if (msc_tx_fifo_wnum <= MAX_PACKET_SIZE)
                txdat_len <= msc_tx_fifo_wnum;
            else
                txdat_len <= MAX_PACKET_SIZE;
        end else

        if (endpt_sel == CONTROL_ENDPOINT) begin
            if (control_tx_fifo_wnum <= MAX_PACKET_SIZE)
                txdat_len <= control_tx_fifo_wnum;
            else
                txdat_len <= CONTROL_MAX_PACKET_SIZE;
        end
    end
end


always @(*) begin
    case(endpt_sel)
        CONTROL_ENDPOINT    : usb_txdat = control_tx_fifo_rd_data;
        CDC_ENDPOINT        : usb_txdat = cdc_tx_fifo_rd_data;
        MSC_ENDPOINT        : usb_txdat = msc_tx_fifo_rd_data;
        default             : usb_txdat = 8'hF;
    endcase
end




//=============================USB DEVICE CONTROLLER=============================
wire [7:0]    inf_alter_i; //!not used
wire [7:0]    inf_alter_o; //!not used
wire [7:0]    inf_sel_o;   //!not used
wire          inf_set_o;   //!not used
    USB_Device_Controller_Top u_usb_device_controller_top (
             .clk_i                 (i_fclk_60M             )
            ,.reset_i               (reset                  )
            ,.usbrst_o              (                       )
            ,.highspeed_o           (                       )
            ,.suspend_o             (                       )
            ,.online_o              (                       )
            ,.txdat_i               (usb_txdat              )
            ,.txval_i               (usb_txval              )//(endpt0_send && (endpt_sel==CONTROL_ENDPOINT))
            ,.txdat_len_i           (txdat_len              )
            ,.txcork_i              (usb_txcork             )
            ,.txiso_pid_i           (4'b0000                )
            ,.txpop_o               (usb_txpop              )
            ,.txact_o               (usb_txact              )
            ,.txpktfin_o            (                       )
            ,.rxdat_o               (usb_rxdat              )
            ,.rxval_o               (usb_rxval              )
            ,.rxact_o               (usb_rxact              )
            ,.rxrdy_i               (usb_rxrdy              )
            ,.rxpktval_o            (                       )
            ,.setup_o               (setup_active           )
            ,.endpt_o               (endpt_sel              )
            ,.sof_o                 (                       )
            ,.inf_alter_i           (8'd0                   )
            ,.inf_alter_o           (inf_alter_o            )
            ,.inf_sel_o             (inf_sel_o              )
            ,.inf_set_o             (inf_set_o              )
            ,.descrom_rdata_i       (DESCROM_RDAT           )
            ,.descrom_raddr_o       (DESCROM_RADDR          )
            ,.desc_dev_addr_i       (DESC_DEV_ADDR          )
            ,.desc_dev_len_i        (DESC_DEV_LEN           )
            ,.desc_qual_addr_i      (DESC_QUAL_ADDR         )
            ,.desc_qual_len_i       (DESC_QUAL_LEN          )
            ,.desc_fscfg_addr_i     (DESC_FSCFG_ADDR        )
            ,.desc_fscfg_len_i      (DESC_FSCFG_LEN         )
            ,.desc_hscfg_addr_i     (DESC_HSCFG_ADDR        )
            ,.desc_hscfg_len_i      (DESC_HSCFG_LEN         )
            ,.desc_oscfg_addr_i     (DESC_OSCFG_ADDR        )
            ,.desc_strlang_addr_i   (DESC_STRLANG_ADDR      )
            ,.desc_strvendor_addr_i (DESC_STRVENDOR_ADDR    )
            ,.desc_strvendor_len_i  (DESC_STRVENDOR_LEN     )
            ,.desc_strproduct_addr_i(DESC_STRPRODUCT_ADDR   )
            ,.desc_strproduct_len_i (DESC_STRPRODUCT_LEN    )
            ,.desc_strserial_addr_i (DESC_STRSERIAL_ADDR    )
            ,.desc_strserial_len_i  (DESC_STRSERIAL_LEN     )
            ,.desc_have_strings_i   (DESCROM_HAVE_STRINGS   )

            ,.utmi_dataout_o        (PHY_DATAOUT            )
            ,.utmi_txvalid_o        (PHY_TXVALID            )
            ,.utmi_txready_i        (PHY_TXREADY            )
            ,.utmi_datain_i         (PHY_DATAIN             )
            ,.utmi_rxactive_i       (PHY_RXACTIVE           )
            ,.utmi_rxvalid_i        (PHY_RXVALID            )
            ,.utmi_rxerror_i        (PHY_RXERROR            )
            ,.utmi_linestate_i      (PHY_LINESTATE          )
            ,.utmi_opmode_o         (PHY_OPMODE             )
            ,.utmi_xcvrselect_o     (PHY_XCVRSELECT         )
            ,.utmi_termselect_o     (PHY_TERMSELECT         )
            ,.utmi_reset_o          (PHY_RESET              )
         );





//=============================USB DEVICE DESCRIPTOR=============================
usb_desc
#(

         .VENDORID    (16'h33AA)
        ,.PRODUCTID   (16'h0000)
        ,.VERSIONBCD  (16'h0100)
        ,.HSSUPPORT   (1       )
        ,.SELFPOWERED (1       )
)
u_usb_desc (
         .CLK                    (i_fclk_60M          )
        ,.RESET                  (reset               )
        ,.i_pid                  (16'd0               )
        ,.i_vid                  (16'd0               )
        ,.i_descrom_raddr        (DESCROM_RADDR       )
        ,.o_descrom_rdat         (DESCROM_RDAT        )
        ,.o_desc_dev_addr        (DESC_DEV_ADDR       )
        ,.o_desc_dev_len         (DESC_DEV_LEN        )
        ,.o_desc_qual_addr       (DESC_QUAL_ADDR      )
        ,.o_desc_qual_len        (DESC_QUAL_LEN       )
        ,.o_desc_fscfg_addr      (DESC_FSCFG_ADDR     )
        ,.o_desc_fscfg_len       (DESC_FSCFG_LEN      )
        ,.o_desc_hscfg_addr      (DESC_HSCFG_ADDR     )
        ,.o_desc_hscfg_len       (DESC_HSCFG_LEN      )
        ,.o_desc_oscfg_addr      (DESC_OSCFG_ADDR     )
        ,.o_desc_strlang_addr    (DESC_STRLANG_ADDR   )
        ,.o_desc_strvendor_addr  (DESC_STRVENDOR_ADDR )
        ,.o_desc_strvendor_len   (DESC_STRVENDOR_LEN  )
        ,.o_desc_strproduct_addr (DESC_STRPRODUCT_ADDR)
        ,.o_desc_strproduct_len  (DESC_STRPRODUCT_LEN )
        ,.o_desc_strserial_addr  (DESC_STRSERIAL_ADDR )
        ,.o_desc_strserial_len   (DESC_STRSERIAL_LEN  )
        ,.o_descrom_have_strings (DESCROM_HAVE_STRINGS)
);





//=============================USB PHY=============================
    USB2_0_SoftPHY_Top u_USB_SoftPHY_Top
    (
         .clk_i            (i_fclk_60M     )
        ,.rst_i            (PHY_RESET      )
        ,.fclk_i           (i_fclk_480M    )
        ,.pll_locked_i     (pll_locked     )
        ,.utmi_data_out_i  (PHY_DATAOUT    )
        ,.utmi_txvalid_i   (PHY_TXVALID    )
        ,.utmi_op_mode_i   (PHY_OPMODE     )
        ,.utmi_xcvrselect_i(PHY_XCVRSELECT )
        ,.utmi_termselect_i(PHY_TERMSELECT )
        ,.utmi_data_in_o   (PHY_DATAIN     )
        ,.utmi_txready_o   (PHY_TXREADY    )
        ,.utmi_rxvalid_o   (PHY_RXVALID    )
        ,.utmi_rxactive_o  (PHY_RXACTIVE   )
        ,.utmi_rxerror_o   (PHY_RXERROR    )
        ,.utmi_linestate_o (PHY_LINESTATE  )
        ,.usb_dxp_io       (usb_dxp_io     )
        ,.usb_dxn_io       (usb_dxn_io     )
        ,.usb_rxdp_i       (usb_rxdp_i     )
        ,.usb_rxdn_i       (usb_rxdn_i     )
        ,.usb_pullup_en_o  (usb_pullup_en_o)
        ,.usb_term_dp_io   (usb_term_dp_io )
        ,.usb_term_dn_io   (usb_term_dn_io )
    );
endmodule
