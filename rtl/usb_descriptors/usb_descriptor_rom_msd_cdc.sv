
`default_nettype none

module usb_desc_rom #(
        // Vendor ID to report in device descriptor.
        parameter VENDORID = 16'h33AA,//Gowin USB Vender ID
        // Product ID to report in device descriptor.
        parameter PRODUCTID = 16'h0120,
        // Product version to report in device descriptor.
        parameter VERSIONBCD = 16'h0100,
        // Support high speed mode.
        parameter HSSUPPORT = 1,
        // Set to true if the device never draws power from the USB bus.
        parameter SELFPOWERED = 0
)(

        input  wire      CLK                    ,
        input  wire      RESET                  ,

        input  wire [9:0] i_descrom_raddr        ,
        output logic [7:0] o_descrom_rdat         ,

        output wire [9:0] o_desc_dev_addr        ,
        output wire [7:0] o_desc_dev_len         ,
        output wire [9:0] o_desc_qual_addr       ,
        output wire [7:0] o_desc_qual_len        ,
        output wire [9:0] o_desc_fscfg_addr      ,
        output wire [7:0] o_desc_fscfg_len       ,
        output wire [9:0] o_desc_hscfg_addr      ,
        output wire [7:0] o_desc_hscfg_len       ,
        output wire [9:0] o_desc_oscfg_addr      ,
        output wire       o_descrom_have_strings    ,
        output wire [9:0] o_desc_strlang_addr    ,
        output wire [9:0] o_desc_strvendor_addr  ,
        output wire [7:0] o_desc_strvendor_len   ,
        output wire [9:0] o_desc_strproduct_addr ,
        output wire [7:0] o_desc_strproduct_len  ,
        output wire [9:0] o_desc_strserial_addr  ,
        output wire [7:0] o_desc_strserial_len
);
    // Descriptor ROM
    localparam  DESC_DEV_ADDR         = 0;
    localparam  DESC_DEV_LEN          = 18;

    localparam  DESC_QUAL_ADDR        = 20;
    localparam  DESC_QUAL_LEN         = 10;

    localparam  DESC_FSCFG_ADDR       = 32;
    localparam  DESC_FSCFG_LEN        = 1;

    localparam  DESC_HSCFG_ADDR       = DESC_FSCFG_ADDR + DESC_FSCFG_LEN;
    localparam  DESC_HSCFG_LEN        = 55;

    localparam  DESC_OSCFG_ADDR       = DESC_HSCFG_ADDR + DESC_HSCFG_LEN;
    localparam  DESC_OSCFG_LEN        = 1 ;

    localparam  DESC_END_ADDR         = DESC_OSCFG_ADDR + DESC_OSCFG_LEN;

    assign  o_desc_dev_addr        = DESC_DEV_ADDR        ;
    assign  o_desc_dev_len         = DESC_DEV_LEN         ;
    assign  o_desc_qual_addr       = DESC_QUAL_ADDR       ;
    assign  o_desc_qual_len        = DESC_QUAL_LEN        ;
    assign  o_desc_fscfg_addr      = DESC_FSCFG_ADDR      ;
    assign  o_desc_fscfg_len       = DESC_FSCFG_LEN       ;
    assign  o_desc_hscfg_addr      = DESC_HSCFG_ADDR      ;
    assign  o_desc_hscfg_len       = DESC_HSCFG_LEN       ;
    assign  o_desc_oscfg_addr      = DESC_OSCFG_ADDR      ;

    assign  o_descrom_have_strings   = '0 ;
    assign  o_desc_strlang_addr      = '0 ;
    assign  o_desc_strvendor_addr    = '0 ;
    assign  o_desc_strvendor_len     = '0 ;
    assign  o_desc_strproduct_addr   = '0 ;
    assign  o_desc_strproduct_len    = '0 ;
    assign  o_desc_strserial_addr    = '0 ;
    assign  o_desc_strserial_len     = '0 ;

    localparam DESCROM_LEN = DESC_END_ADDR + 1;
    logic [7:0] descrom [0:DESCROM_LEN-1];

    initial begin
        // 18 bytes device descriptor
        descrom[DESC_DEV_ADDR + 0]  = 8'h12;// bLength = 18 bytes
        descrom[DESC_DEV_ADDR + 1]  = 8'h01;// bDescriptorType = device descriptor
        descrom[DESC_DEV_ADDR + 2]  = 8'h00; // (HSSUPPORT) ? 8'h00 : 8'h10;// bcdUSB = 1.10 || 2.00
        descrom[DESC_DEV_ADDR + 3]  = 8'h02; // (HSSUPPORT) ? 8'h02 : 8'h01;
        descrom[DESC_DEV_ADDR + 4]  = 8'h00;// bDeviceClass = none
        descrom[DESC_DEV_ADDR + 5]  = 8'h00;// bDeviceSubClass = none
        descrom[DESC_DEV_ADDR + 6]  = 8'h00;// bDeviceProtocol = none
        descrom[DESC_DEV_ADDR + 7]  = 8'h40;// bMaxPacketSize = 64 bytes
        descrom[DESC_DEV_ADDR + 8]  = VENDORID[7 : 0];// idVendor
        descrom[DESC_DEV_ADDR + 9]  = VENDORID[15 :8];
        descrom[DESC_DEV_ADDR + 10] = PRODUCTID[7 :0];// idProduct
        descrom[DESC_DEV_ADDR + 11] = PRODUCTID[15 :8];
        descrom[DESC_DEV_ADDR + 12] = VERSIONBCD[7 : 0];// bcdDevice
        descrom[DESC_DEV_ADDR + 13] = VERSIONBCD[15 : 8];
        descrom[DESC_DEV_ADDR + 14] = 8'h00; // iManufacturer
        descrom[DESC_DEV_ADDR + 15] = 8'h00; // iProduct
        descrom[DESC_DEV_ADDR + 16] = 8'h00; // iSerialNumber
        descrom[DESC_DEV_ADDR + 17] = 8'h01; // bNumConfigurations = 1
        // 2 bytes padding
        descrom[DESC_DEV_ADDR + 18] = 8'h00;
        descrom[DESC_DEV_ADDR + 19] = 8'h00;

        // 10 bytes device qualifier
        descrom[DESC_QUAL_ADDR + 0] = 8'h0a;// bLength = 10 bytes
        descrom[DESC_QUAL_ADDR + 1] = 8'h06;// bDescriptorType = device qualifier
        descrom[DESC_QUAL_ADDR + 2] = 8'h00;
        descrom[DESC_QUAL_ADDR + 3] = 8'h02;// bcdUSB = 2.0
        descrom[DESC_QUAL_ADDR + 4] = 8'h02;// bDeviceClass = Communication Device Class
        descrom[DESC_QUAL_ADDR + 5] = 8'h02;// bDeviceSubClass = ACM
        descrom[DESC_QUAL_ADDR + 6] = 8'h00;// bDeviceProtocol = none
        descrom[DESC_QUAL_ADDR + 7] = 8'h40;// bMaxPacketSize0 = 64 bytes
        descrom[DESC_QUAL_ADDR + 8] = 8'h01;// bNumConfigurations = 1
        descrom[DESC_QUAL_ADDR + 9] = 8'h00;// bReserved
        // 2 bytes padding
        descrom[DESC_QUAL_ADDR + 10] = 8'h00;
        descrom[DESC_QUAL_ADDR + 11] = 8'h00;

        // Full Speed Cfg
        descrom[DESC_FSCFG_ADDR + 0] = 8'h00;// bLength = 9 bytes

        // HIGH Speed Cfg
        descrom[DESC_HSCFG_ADDR + 0] = 8'h09;// bLength = 9 bytes
        descrom[DESC_HSCFG_ADDR + 1] = 8'h02;// bDescriptorType = configuration descriptor
        descrom[DESC_HSCFG_ADDR + 2] = DESC_HSCFG_LEN[7:0];// 59 bytes
        descrom[DESC_HSCFG_ADDR + 3] = DESC_HSCFG_LEN[15:8];// wTotalLength = 59 bytes
        descrom[DESC_HSCFG_ADDR + 4] = 8'h01;// bNumInterfaces = 2
        descrom[DESC_HSCFG_ADDR + 5] = 8'h01;// bConfigurationValue = 1
        descrom[DESC_HSCFG_ADDR + 6] = 8'h00;// iConfiguration = none
        descrom[DESC_HSCFG_ADDR + 7] = (SELFPOWERED)? 8'hc0 : 8'h80;// bmAttributes
        descrom[DESC_HSCFG_ADDR + 8] = 8'hFA;// bMaxPower = 500 mA
        // Interface Descriptor

        // Interface Descriptor
        // Serial interface
        descrom[DESC_HSCFG_ADDR + 9]  = 8'h09;// bLength = 9 bytes
        descrom[DESC_HSCFG_ADDR + 10] = 8'h04;// bDescriptorType = interface descriptor
        descrom[DESC_HSCFG_ADDR + 11] = 8'h00;// bInterfaceNumber = 0
        descrom[DESC_HSCFG_ADDR + 12] = 8'h00;// bAlternateSetting = 0
        descrom[DESC_HSCFG_ADDR + 13] = 8'h02;// bNumEndpoints = 2
        descrom[DESC_HSCFG_ADDR + 14] = 8'h02;// bInterfaceClass = CDC
        descrom[DESC_HSCFG_ADDR + 15] = 8'h02;// bInterfaceSubClass = ACM
        descrom[DESC_HSCFG_ADDR + 16] = 8'h00;// bInterafceProtocol = none
        descrom[DESC_HSCFG_ADDR + 17] = 8'h00;// iInterface = none

        // Endpoint Descriptor
        descrom[DESC_HSCFG_ADDR + 22-4] = 8'h07;// bLength = 7 bytes
        descrom[DESC_HSCFG_ADDR + 23-4] = 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_HSCFG_ADDR + 24-4] = 8'h82;// bEndpointAddress = INPUT 2
        descrom[DESC_HSCFG_ADDR + 25-4] = 8'h02;// bmAttributes = Bulk
        descrom[DESC_HSCFG_ADDR + 26-4] = 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_HSCFG_ADDR + 27-4] = 8'h02;
        descrom[DESC_HSCFG_ADDR + 28-4] = 8'h00;// bInterval = 0 ms
        // Endpoint Descriptor
        descrom[DESC_HSCFG_ADDR + 29-4] = 8'h07;// bLength = 7 bytes
        descrom[DESC_HSCFG_ADDR + 30-4] = 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_HSCFG_ADDR + 31-4] = 8'h02;// bEndpointAddress = OUTPUT 2
        descrom[DESC_HSCFG_ADDR + 32-4] = 8'h02;// TransferType = Bulk
        descrom[DESC_HSCFG_ADDR + 33-4] = 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_HSCFG_ADDR + 34-4] = 8'h02;
        descrom[DESC_HSCFG_ADDR + 35-4] = 8'h00;// bInterval = 0 ms

        // Interface Descriptor
        // MASS STORAGE interface
        descrom[DESC_HSCFG_ADDR + 36-4] = 8'h09;// bLength = 9 bytes
        descrom[DESC_HSCFG_ADDR + 37-4] = 8'h04;// bDescriptorType = interface descriptor
        descrom[DESC_HSCFG_ADDR + 38-4] = 8'h01;// bInterfaceNumber = 1
        descrom[DESC_HSCFG_ADDR + 39-4] = 8'h00;// bAlternateSetting = 0
        descrom[DESC_HSCFG_ADDR + 40-4] = 8'h02;// bNumEndpoints = 2 //8'h02
        descrom[DESC_HSCFG_ADDR + 41-4] = 8'h08;// bInterfaceClass = MSC //8'h08
        descrom[DESC_HSCFG_ADDR + 42-4] = 8'h06;// bInterfaceSubClass = SCSI Transparent command set //8'h06
        descrom[DESC_HSCFG_ADDR + 43-4] = 8'h50;// bInterafceProtocol = BBB //8'h50
        descrom[DESC_HSCFG_ADDR + 44-4] = 8'h00;// iInterface = none
        // Endpoint Descriptor
        descrom[DESC_HSCFG_ADDR + 45-4] = 8'h07;// bLength = 7 bytes
        descrom[DESC_HSCFG_ADDR + 46-4] = 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_HSCFG_ADDR + 47-4] = 8'h81;// bEndpointAddress = INPUT 4
        descrom[DESC_HSCFG_ADDR + 48-4] = 8'h02;// bmAttributes = Bulk
        descrom[DESC_HSCFG_ADDR + 49-4] = 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_HSCFG_ADDR + 50-4] = 8'h02;
        descrom[DESC_HSCFG_ADDR + 51-4] = 8'h00;// bInterval = 0 ms
        // Endpoint Descriptor
        descrom[DESC_HSCFG_ADDR + 52-4] = 8'h07;// bLength = 7 bytes
        descrom[DESC_HSCFG_ADDR + 53-4] = 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_HSCFG_ADDR + 54-4] = 8'h01;// bEndpointAddress = OUTPUT 4
        descrom[DESC_HSCFG_ADDR + 55-4] = 8'h02;// TransferType = Bulk
        descrom[DESC_HSCFG_ADDR + 56-4] = 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_HSCFG_ADDR + 57-4] = 8'h02;
        descrom[DESC_HSCFG_ADDR + 58-4] = 8'h00;// bInterval = 0 ms

        // Other speed cfg
        descrom[DESC_OSCFG_ADDR + 0]  = 8'h00;
    end

    // NOTE: address already registred inside controller
    // So, ROM will be infered
    assign o_descrom_rdat = descrom[i_descrom_raddr];

endmodule
