
`default_nettype none

module usb_req_handler #(
    parameter bit [7:0] MSD_LUN_NUM = 8'd0
)(
    input   wire       clk_i         ,
    input   wire       rst_i         ,

    // setup request rx interface
    input   wire       setup_i       ,
    input   wire       rxval_i       ,
    input   wire [7:0] rxdat_i       ,

    // response tx fifo interface
    output logic [7:0] rsp_tx_dat_o  ,
    output logic       rsp_tx_ena_o  ,
    input  wire        rsp_tx_full_i ,
    output logic       rps_tx_err_o  ,

    // MSD requests status
    output logic       msd_rst_req_o ,
    output logic       msd_get_max_lun_req_o
);

    logic       val_r  ;
    logic [7:0] dat_r  ;
    logic [7:0] dat_rr ;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            val_r <= '0;
        end else begin
            val_r <= setup_i && rxval_i;
        end

    always_ff @(posedge clk_i) begin
        dat_r  <= rxdat_i ;
        dat_rr <= dat_r   ;
    end

    typedef enum {
        WAIT_REQ,
        SKIP_REQ,
        CHK_REQ
    } state_e;

    state_e state, state_next;
    logic [2:0] dat_cnt;
    logic req_done_r;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
            state <= WAIT_REQ;
        else
            state <= state_next;

    always_comb begin
        state_next = state;
        case (state)
            WAIT_REQ: if (val_r) begin
                state_next = (dat_r == 8'hA1) ? CHK_REQ : SKIP_REQ;
            end

            CHK_REQ: if (val_r) begin
                state_next = SKIP_REQ;
            end

            SKIP_REQ: if (val_r && dat_cnt == 0) begin
                state_next = WAIT_REQ;
            end
        endcase
    end

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            dat_cnt <= '0;
        end else begin
            if (state == WAIT_REQ)
                dat_cnt <= 6;
            else if (state == CHK_REQ)
                dat_cnt <= 5;
            else if (state == SKIP_REQ && val_r)
                dat_cnt <= dat_cnt - 1;
        end

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            req_done_r <= '0;
        end else begin
            req_done_r <= (val_r && dat_cnt == 0);
        end

    // -----------------------------------------------------------
    // check MSD requests
    // -----------------------------------------------------------

    logic msd_req_chk_ena     ;
    logic msd_rst_req         ;
    logic msd_get_max_lun_req ;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            msd_req_chk_ena     <= '0;
            msd_rst_req         <= '0;
            msd_get_max_lun_req <= '0;
        end else begin
            msd_req_chk_ena <= (state == CHK_REQ) && val_r;

            if (msd_rst_req)
                msd_rst_req <= '0;
            else if (msd_req_chk_ena)
                msd_rst_req <= (dat_rr == 'hFF);

            if (msd_get_max_lun_req)
                msd_get_max_lun_req <= '0;
            else if (msd_req_chk_ena)
                msd_get_max_lun_req <= (dat_rr == 'hFE);
        end

    assign msd_rst_req_o = msd_rst_req;
    assign msd_get_max_lun_req_o = msd_get_max_lun_req;

    // -----------------------------------------------------------
    // Send response data
    // -----------------------------------------------------------

    assign rsp_tx_dat_o = MSD_LUN_NUM;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            rps_tx_err_o <= '0;
            rsp_tx_ena_o <= '0;
        end else begin
            if (rsp_tx_ena_o)
                rsp_tx_ena_o <= 1'b0;
            else if (msd_get_max_lun_req && !rsp_tx_full_i)
                rsp_tx_ena_o <= 1'b1;

            if (rps_tx_err_o)
                rps_tx_err_o <= 1'b0;
            else if (msd_get_max_lun_req && rsp_tx_full_i)
                rps_tx_err_o <= 1'b1;
        end

    // -----------------------------------------------------------
    // Generate reset strobe
    // -----------------------------------------------------------



endmodule

`resetall
