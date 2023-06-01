
`default_nettype none

module usb_wb_ctrl_status_regs (
    input  wire    clk_i ,
    input  wire    rst_i ,
    wb_if.slv      wbs   ,

    input  wire    msd_rst_req_i        ,
    input  wire    msd_get_max_lun_i    ,
    input  wire    msd_rps_tx_err_i     ,

    input  wire    usb_ack_timeout_i    ,
    input  wire    usb_ack_received_i   ,
    input  wire    usb_ack_bad_packet_i ,   

    output wire    usb_rst_o
);

    wire [4:0] wb_adr = wbs.adr[4:0];

    // Address MAP:
    // 0 (W0)   - soft reset
    // 4 (R/WC) - setup req handler status
    // 8 (RO/WC)   - USB ACK timeout status
    // 12 (RO/WC) - USB ACK received 
    // 16 (RO/WC) - USB ACK bad packet
    // ----------------------------------------------------------
    // Generate USB reset strobe
    // ----------------------------------------------------------

    logic usb_rst       ;

    always_ff @(posedge clk_i, posedge rst_i) begin
        if (rst_i) begin
            usb_rst <= 1'b1;
        end else begin
            if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == '0)) begin
                usb_rst <= wbs.dat_m2s[0];
            end
        end
    end

    assign usb_rst_o = usb_rst;

    // ----------------------------------------------------------
    // USB status register
    // ----------------------------------------------------------

    struct packed {
        logic msd_rps_tx_err  ;
        logic msd_get_max_lun ;
        logic msd_reset       ;
    } status_r;

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
            status_r <= '0;
        else begin
            if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'd4))
                status_r <= '0;
            else begin
                if (msd_rst_req_i)
                    status_r.msd_reset <= 1'b1;
                if (msd_get_max_lun_i)
                    status_r.msd_get_max_lun <= 1'b1;
                if (msd_rps_tx_err_i)
                    status_r.msd_rps_tx_err <= 1'b1;
            end
        end


    // ----------------------------------------------------------
    // Ack timeout register
    // ----------------------------------------------------------

    logic usb_ack_timeout_q     ;
    logic usb_ack_timeout       ;
    logic usb_ack_received_q    ;
    logic usb_ack_received      ;
    logic usb_ack_bad_packet_q  ;
    logic usb_ack_bad_packet    ;




    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i) begin
            usb_ack_timeout_q   <= '0;
            usb_ack_timeout     <= '0;
            usb_ack_received_q  <= '0; 
            usb_ack_received    <= '0;
            usb_ack_bad_packet_q<= '0;
            usb_ack_bad_packet  <= '0;

        end else begin
            usb_ack_timeout_q <= usb_ack_timeout_i;
            usb_ack_received_q <= usb_ack_received_i;
            usb_ack_bad_packet_q <= usb_ack_bad_packet_i;

            if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'd8))
                usb_ack_timeout <= '0;
            else if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'd12))
                usb_ack_received <= '0;
            else if (wbs.cyc && wbs.stb && wbs.we && (wb_adr == 5'd16))
                usb_ack_bad_packet <= '0;
            else if (!usb_ack_timeout_q && usb_ack_timeout_i)
                usb_ack_timeout <= 1'b1;
            else if (!usb_ack_received_q && usb_ack_received_i)
                usb_ack_received <= 1'b1;
            else if (!usb_ack_bad_packet_q && usb_ack_bad_packet_i)
                usb_ack_bad_packet <= 1'b1;
        end

    // ----------------------------------------------------------
    // Wishbone read
    // ----------------------------------------------------------

    always_ff @(posedge clk_i)
        if      (wb_adr == 5'd8)
            wbs.dat_s2m <= {'0, usb_ack_timeout};
        else if (wb_adr == 5'd4)
            wbs.dat_s2m <= {'0, status_r};
        else if (wb_adr == 5'd12)
            wbs.dat_s2m <= {'0, usb_ack_received};
        else if (wb_adr == 5'd16)
            wbs.dat_s2m <= {'0, usb_ack_bad_packet};
        else
            wbs.dat_s2m <= 'hdead_beaf;



    // ----------------------------------------------------------
    // Wishbone ack
    // ----------------------------------------------------------

    always_ff @(posedge clk_i, posedge rst_i)
        if (rst_i)
		    wbs.ack <= 1'b0;
	    else if (wbs.ack)
		    wbs.ack <= 1'b0;
	    else if (wbs.cyc && wbs.stb && !wbs.ack)
		    wbs.ack <= 1'b1;

endmodule

`resetall
