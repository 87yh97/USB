
//===========================================
module usb_desc_msd_cdc #(
        // Vendor ID to report in device descriptor.
        parameter VENDORID = 16'h33AA,//Gowin USB Vender ID
        // Product ID to report in device descriptor.
        parameter PRODUCTID = 16'h0120,
        // Product version to report in device descriptor.
        parameter VERSIONBCD = 16'h0100,
        // Optional description of manufacturer (max 126 characters).
        parameter VENDORSTR = "PSPOD",
        parameter VENDORSTR_LEN = 5,
        // Optional description of product (max 126 characters).
        parameter PRODUCTSTR = "USB Module",
        parameter PRODUCTSTR_LEN = 10,
        // Optional product serial number (max 126 characters).
        parameter SERIALSTR = "0001",
        parameter SERIALSTR_LEN = 4,
        //Interface 0 name
        parameter INTRFC0STR = "CDCCOM",
        parameter INTRFC0STR_LEN = 6,
        //Interface 0 name
        parameter INTRFC1STR = "MSCCOM",
        parameter INTRFC1STR_LEN = 6,
        // Support high speed mode.
        parameter HSSUPPORT = 1,
        // Set to true if the device never draws power from the USB bus.
        parameter SELFPOWERED = 0
)
(

        input        CLK                    ,
        input        RESET                  ,
        input  [15:0] i_pid                 ,
        input  [15:0] i_vid                 ,
        input  [9:0] i_descrom_raddr        ,
        output [7:0] o_descrom_rdat         ,
        output [9:0] o_desc_dev_addr        ,
        output [7:0] o_desc_dev_len         ,
        output [9:0] o_desc_qual_addr       ,
        output [7:0] o_desc_qual_len        ,
        output [9:0] o_desc_fscfg_addr      ,
        output [7:0] o_desc_fscfg_len       ,
        output [9:0] o_desc_hscfg_addr      ,
        output [7:0] o_desc_hscfg_len       ,
        output [9:0] o_desc_oscfg_addr      ,
        output [9:0] o_desc_strlang_addr    ,
        output [9:0] o_desc_strvendor_addr  ,
        output [7:0] o_desc_strvendor_len   ,
        output [9:0] o_desc_strproduct_addr ,
        output [7:0] o_desc_strproduct_len  ,
        output [9:0] o_desc_strserial_addr  ,
        output [7:0] o_desc_strserial_len   ,
        output       o_descrom_have_strings
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

    localparam  DESC_STRLANG_ADDR     = DESC_OSCFG_ADDR + DESC_OSCFG_LEN;

    localparam  DESC_STRVENDOR_ADDR   = DESC_STRLANG_ADDR + 4;
    localparam  DESC_STRVENDOR_LEN    = 2 + 2*VENDORSTR_LEN;

    localparam  DESC_STRPRODUCT_ADDR  = DESC_STRVENDOR_ADDR + DESC_STRVENDOR_LEN;
    localparam  DESC_STRPRODUCT_LEN   = 2 + 2*PRODUCTSTR_LEN;

    localparam  DESC_STRSERIAL_ADDR   = DESC_STRPRODUCT_ADDR + DESC_STRPRODUCT_LEN;
    localparam  DESC_STRSERIAL_LEN    = 2 + 2*SERIALSTR_LEN;

//    localparam  DESC_STRINTRFC0_ADDR  = DESC_STRSERIAL_ADDR + DESC_STRSERIAL_LEN;
//    localparam  DESC_STRINTRFC0_LEN   = 2 + 2*INTRFC0STR_LEN;
//
//    localparam  DESC_STRINTRFC1_ADDR  = DESC_STRINTRFC0_ADDR + DESC_STRINTRFC0_LEN;
//    localparam  DESC_STRINTRFC1_LEN   = 2 + 2*INTRFC1STR_LEN;

    localparam  DESC_END_ADDR         = DESC_STRSERIAL_ADDR + DESC_STRSERIAL_LEN; //DESC_STRINTRFC1_ADDR + DESC_STRINTRFC1_LEN;

    assign  o_desc_dev_addr        = DESC_DEV_ADDR        ;
    assign  o_desc_dev_len         = DESC_DEV_LEN         ;
    assign  o_desc_qual_addr       = DESC_QUAL_ADDR       ;
    assign  o_desc_qual_len        = DESC_QUAL_LEN        ;
    assign  o_desc_fscfg_addr      = DESC_FSCFG_ADDR      ;
    assign  o_desc_fscfg_len       = DESC_FSCFG_LEN       ;
    assign  o_desc_hscfg_addr      = DESC_HSCFG_ADDR      ;
    assign  o_desc_hscfg_len       = DESC_HSCFG_LEN       ;
    assign  o_desc_oscfg_addr      = DESC_OSCFG_ADDR      ;
    assign  o_desc_strlang_addr    = DESC_STRLANG_ADDR    ;
    assign  o_desc_strvendor_addr  = DESC_STRVENDOR_ADDR  ;
    assign  o_desc_strvendor_len   = DESC_STRVENDOR_LEN   ;
    assign  o_desc_strproduct_addr = DESC_STRPRODUCT_ADDR ;
    assign  o_desc_strproduct_len  = DESC_STRPRODUCT_LEN  ;
    assign  o_desc_strserial_addr  = DESC_STRSERIAL_ADDR  ;
    assign  o_desc_strserial_len   = DESC_STRSERIAL_LEN   ;

    // Truncate descriptor data to keep only the necessary pieces;
    // either just the full-speed stuff, || full-speed plus high-speed,
    // || full-speed plus high-speed plus string descriptors.

    localparam descrom_have_strings = (VENDORSTR_LEN > 0 || PRODUCTSTR_LEN > 0 || SERIALSTR_LEN > 0 || INTRFC0STR_LEN > 0 || INTRFC1STR_LEN > 0);
    localparam descrom_len = (HSSUPPORT || descrom_have_strings)?((descrom_have_strings)? DESC_END_ADDR : DESC_OSCFG_ADDR + DESC_OSCFG_LEN) : DESC_FSCFG_ADDR + DESC_FSCFG_LEN;
    assign o_descrom_have_strings = descrom_have_strings;
    reg [7:0] descrom [0 : descrom_len-1];
    integer i;
    integer z;

    always @(posedge CLK or posedge RESET)
      if(RESET) begin
        // 18 bytes device descriptor
        descrom[0]  <= 8'h12;// bLength = 18 bytes
        descrom[1]  <= 8'h01;// bDescriptorType = device descriptor
        descrom[2]  <= (HSSUPPORT)? 8'h00 :8'h10;// bcdUSB = 1.10 || 2.00
        descrom[3]  <= (HSSUPPORT)? 8'h02 :8'h01;
        descrom[4]  <= 8'h00;// bDeviceClass = none
        descrom[5]  <= 8'h00;// bDeviceSubClass = none
        descrom[6]  <= 8'h00;// bDeviceProtocol = none
        descrom[7]  <= 8'h40;// bMaxPacketSize = 64 bytes
        descrom[8]  <= VENDORID[7 : 0];// idVendor
        descrom[9]  <= VENDORID[15 :8];
        descrom[10] <= PRODUCTID[7 :0];// idProduct
        descrom[11] <= PRODUCTID[15 :8];
        descrom[12] <= VERSIONBCD[7 : 0];// bcdDevice
        descrom[13] <= VERSIONBCD[15 : 8];
        descrom[14] <= (VENDORSTR_LEN > 0)?  8'h01: 8'h00;// iManufacturer
        descrom[15] <= (PRODUCTSTR_LEN > 0)? 8'h02: 8'h00;// iProduct
        descrom[16] <= (SERIALSTR_LEN > 0)?  8'h03: 8'h00;// iSerialNumber
        descrom[17] <= 8'h01; // bNumConfigurations = 1
        // 2 bytes padding
        descrom[18] <= 8'h00;
        descrom[19] <= 8'h00;
        // 10 bytes device qualifier
        descrom[20 + 0] <= 8'h0A;// bLength = 10 bytes
        descrom[20 + 1] <= 8'h06;// bDescriptorType = device qualifier
        descrom[20 + 2] <= 8'h00;
        descrom[20 + 3] <= 8'h02;// bcdUSB = 2.0
        descrom[20 + 4] <= 8'h00;// bDeviceClass = none
        descrom[20 + 5] <= 8'h00;// bDeviceSubClass = none
        descrom[20 + 6] <= 8'h00;// bDeviceProtocol = none
        descrom[20 + 7] <= 8'h40;// bMaxPacketSize0 = 64 bytes
        descrom[20 + 8] <= 8'h01;// bNumConfigurations = 1
        descrom[20 + 9] <= 8'h00;// bReserved
         // 2 bytes padding
        descrom[20 + 10] <= 8'h00;
        descrom[20 + 11] <= 8'h00;







descrom[DESC_FSCFG_ADDR + 0] <= 8'h09;// bLength = 9 bytes
/*
          //======Full Speed Cfg
        descrom[DESC_FSCFG_ADDR + 0] <= 8'h09;// bLength = 9 bytes
        descrom[DESC_FSCFG_ADDR + 1] <= 8'h02;// bDescriptorType = configuration descriptor
        descrom[DESC_FSCFG_ADDR + 2] <= DESC_FSCFG_LEN[7:0];// 55 bytes
        descrom[DESC_FSCFG_ADDR + 3] <= DESC_FSCFG_LEN[15:8];// wTotalLength = 55 bytes
        descrom[DESC_FSCFG_ADDR + 4] <= 8'h02;// bNumInterfaces = 2
        descrom[DESC_FSCFG_ADDR + 5] <= 8'h01;// bConfigurationValue = 1
        descrom[DESC_FSCFG_ADDR + 6] <= 8'h00;// iConfiguration = none
        descrom[DESC_FSCFG_ADDR + 7] <= (SELFPOWERED)? 8'hc0 : 8'h80; // bmAttributes
        descrom[DESC_FSCFG_ADDR + 8] <= 8'hFA;// bMaxPower = 500 mA
        //---------------- Interface Descriptor -----------------
        //------------Serial COM 0 interface------------------
        descrom[DESC_FSCFG_ADDR + 9] <= 8'h09;// bLength = 9 bytes
        descrom[DESC_FSCFG_ADDR + 10] <= 8'h04;// bDescriptorType = interface descriptor
        descrom[DESC_FSCFG_ADDR + 11] <= 8'h00;// bInterfaceNumber = 0
        descrom[DESC_FSCFG_ADDR + 12] <= 8'h00;// bAlternateSetting = 0
        descrom[DESC_FSCFG_ADDR + 13] <= 8'h02;// bNumEndpoints = 2
        descrom[DESC_FSCFG_ADDR + 14] <= 8'h02;// bInterfaceClass = CDC
        descrom[DESC_FSCFG_ADDR + 15] <= 8'h02;// bInterfaceSubClass = AMC
        descrom[DESC_FSCFG_ADDR + 16] <= 8'h00;// bInterafceProtocol = none
        descrom[DESC_FSCFG_ADDR + 17] <= 8'h00;// iInterface = none
        //----------------- Endpoint Descriptor -----------------
        descrom[DESC_FSCFG_ADDR + 18] <= 8'h07;// bLength = 7 bytes
        descrom[DESC_FSCFG_ADDR + 19] <= 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_FSCFG_ADDR + 20] <= 8'h82;// bEndpointAddress = INPUT 2
        descrom[DESC_FSCFG_ADDR + 21] <= 8'h02;// TransferType= Bulk
        descrom[DESC_FSCFG_ADDR + 22] <= 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_FSCFG_ADDR + 23] <= 8'h02;
        descrom[DESC_FSCFG_ADDR + 24] <= 8'h00;// bInterval = 0 ms
        //----------------- Endpoint Descriptor -----------------
        descrom[DESC_FSCFG_ADDR + 25] <= 8'h07;// bLength = 7 bytes
        descrom[DESC_FSCFG_ADDR + 26] <= 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_FSCFG_ADDR + 27] <= 8'h02;// bEndpointAddress = OUTPUT 2
        descrom[DESC_FSCFG_ADDR + 28] <= 8'h02;// TransferType = Bulk
        descrom[DESC_FSCFG_ADDR + 29] <= 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_FSCFG_ADDR + 30] <= 8'h02;
        descrom[DESC_FSCFG_ADDR + 31] <= 8'h00;// bInterval = 0 ms



        //---------------- Interface Descriptor -----------------
        //------------Serial COM 1 interface------------------
        descrom[DESC_FSCFG_ADDR + 32] <= 8'h09;// bLength = 9 bytes
        descrom[DESC_FSCFG_ADDR + 33] <= 8'h04;// bDescriptorType = interface descriptor
        descrom[DESC_FSCFG_ADDR + 34] <= 8'h01;// bInterfaceNumber = 1
        descrom[DESC_FSCFG_ADDR + 35] <= 8'h00;// bAlternateSetting = 0
        descrom[DESC_FSCFG_ADDR + 36] <= 8'h02;// bNumEndpoints = 2
        descrom[DESC_FSCFG_ADDR + 37] <= 8'h02;// bInterfaceClass = CDC
        descrom[DESC_FSCFG_ADDR + 38] <= 8'h02;// bInterfaceSubClass = AMC
        descrom[DESC_FSCFG_ADDR + 39] <= 8'h00;// bInterafceProtocol = none
        descrom[DESC_FSCFG_ADDR + 40] <= 8'h00;// iInterface = none
        //----------------- Endpoint Descriptor -----------------
        descrom[DESC_FSCFG_ADDR + 41] <= 8'h07;// bLength = 7 bytes
        descrom[DESC_FSCFG_ADDR + 42] <= 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_FSCFG_ADDR + 43] <= 8'h85;// bEndpointAddress = INPUT 5
        descrom[DESC_FSCFG_ADDR + 44] <= 8'h02;// TransferType= Bulk
        descrom[DESC_FSCFG_ADDR + 45] <= 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_FSCFG_ADDR + 46] <= 8'h02;
        descrom[DESC_FSCFG_ADDR + 47] <= 8'h00;// bInterval = 0 ms
        //----------------- Endpoint Descriptor -----------------
        descrom[DESC_FSCFG_ADDR + 48] <= 8'h07;// bLength = 7 bytes
        descrom[DESC_FSCFG_ADDR + 49] <= 8'h05;// bDescriptorType = endpoint descriptor
        descrom[DESC_FSCFG_ADDR + 50] <= 8'h05;// bEndpointAddress = OUTPUT 2
        descrom[DESC_FSCFG_ADDR + 51] <= 8'h02;// TransferType = Bulk
        descrom[DESC_FSCFG_ADDR + 52] <= 8'h00;// wMaxPacketSize = 512 bytes
        descrom[DESC_FSCFG_ADDR + 53] <= 8'h02;
        descrom[DESC_FSCFG_ADDR + 54] <= 8'h00;// bInterval = 0 ms

*/








          //======HIGH Speed Cfg
          descrom[DESC_HSCFG_ADDR + 0] <= 8'h09;// bLength = 9 bytes
          descrom[DESC_HSCFG_ADDR + 1] <= 8'h02;// bDescriptorType = configuration descriptor
          descrom[DESC_HSCFG_ADDR + 2] <= DESC_HSCFG_LEN[7:0];// 82 bytes
          descrom[DESC_HSCFG_ADDR + 3] <= DESC_HSCFG_LEN[15:8];// wTotalLength = 82 bytes
          descrom[DESC_HSCFG_ADDR + 4] <= 8'h02;// bNumInterfaces = 2
          descrom[DESC_HSCFG_ADDR + 5] <= 8'h01;// bConfigurationValue = 1
          descrom[DESC_HSCFG_ADDR + 6] <= 8'h00;// iConfiguration = none
          descrom[DESC_HSCFG_ADDR + 7] <= (SELFPOWERED)? 8'hc0 : 8'h80;// bmAttributes
          descrom[DESC_HSCFG_ADDR + 8] <= 8'hFA;// bMaxPower = 500 mA



          //---------------- Interface Descriptor -----------------
          //------------Serial COM 0 interface------------------
          descrom[DESC_HSCFG_ADDR + 9] <= 8'h09;// bLength = 9 bytes
          descrom[DESC_HSCFG_ADDR + 10] <= 8'h04;// bDescriptorType = interface descriptor
          descrom[DESC_HSCFG_ADDR + 11] <= 8'h00;// bInterfaceNumber = 0
          descrom[DESC_HSCFG_ADDR + 12] <= 8'h00;// bAlternateSetting = 0
          descrom[DESC_HSCFG_ADDR + 13] <= 8'h02;// bNumEndpoints = 2
          descrom[DESC_HSCFG_ADDR + 14] <= 8'h02;// bInterfaceClass = CDC
          descrom[DESC_HSCFG_ADDR + 15] <= 8'h02;// bInterfaceSubClass = ACM
          descrom[DESC_HSCFG_ADDR + 16] <= 8'h00;// bInterafceProtocol = none
          descrom[DESC_HSCFG_ADDR + 17] <= 8'h00;// iInterface = none

          //----------------- Endpoint Descriptor -----------------
          descrom[DESC_HSCFG_ADDR + 22-4] <= 8'h07;// bLength = 7 bytes
          descrom[DESC_HSCFG_ADDR + 23-4] <= 8'h05;// bDescriptorType = endpoint descriptor
          descrom[DESC_HSCFG_ADDR + 24-4] <= 8'h82;// bEndpointAddress = INPUT 2
          descrom[DESC_HSCFG_ADDR + 25-4] <= 8'h02;// bmAttributes = Bulk
          descrom[DESC_HSCFG_ADDR + 26-4] <= 8'h00;// wMaxPacketSize = 512 bytes
          descrom[DESC_HSCFG_ADDR + 27-4] <= 8'h02;
          descrom[DESC_HSCFG_ADDR + 28-4] <= 8'h00;// bInterval = 0 ms
          //----------------- Endpoint Descriptor -----------------
          descrom[DESC_HSCFG_ADDR + 29-4] <= 8'h07;// bLength = 7 bytes
          descrom[DESC_HSCFG_ADDR + 30-4] <= 8'h05;// bDescriptorType = endpoint descriptor
          descrom[DESC_HSCFG_ADDR + 31-4] <= 8'h02;// bEndpointAddress = OUTPUT 2
          descrom[DESC_HSCFG_ADDR + 32-4] <= 8'h02;// TransferType = Bulk
          descrom[DESC_HSCFG_ADDR + 33-4] <= 8'h00;// wMaxPacketSize = 512 bytes
          descrom[DESC_HSCFG_ADDR + 34-4] <= 8'h02;
          descrom[DESC_HSCFG_ADDR + 35-4] <= 8'h00;// bInterval = 0 ms



          //---------------- Interface Descriptor -----------------
          //------------MASS STORAGE interface------------------
          descrom[DESC_HSCFG_ADDR + 36-4] <= 8'h09;// bLength = 9 bytes
          descrom[DESC_HSCFG_ADDR + 37-4] <= 8'h04;// bDescriptorType = interface descriptor
          descrom[DESC_HSCFG_ADDR + 38-4] <= 8'h01;// bInterfaceNumber = 1
          descrom[DESC_HSCFG_ADDR + 39-4] <= 8'h00;// bAlternateSetting = 0
          descrom[DESC_HSCFG_ADDR + 40-4] <= 8'h02;// bNumEndpoints = 2
          descrom[DESC_HSCFG_ADDR + 41-4] <= 8'h08;// bInterfaceClass = MSC
          descrom[DESC_HSCFG_ADDR + 42-4] <= 8'h06;// bInterfaceSubClass = SCSI
          descrom[DESC_HSCFG_ADDR + 43-4] <= 8'h50;// bInterafceProtocol = BOT(BBB)
          descrom[DESC_HSCFG_ADDR + 44-4] <= 8'h00;// iInterface = none

          //----------------- Endpoint Descriptor -----------------
          descrom[DESC_HSCFG_ADDR + 45-4] <= 8'h07;// bLength = 7 bytes
          descrom[DESC_HSCFG_ADDR + 46-4] <= 8'h05;// bDescriptorType = endpoint descriptor
          descrom[DESC_HSCFG_ADDR + 47-4] <= 8'h81;// bEndpointAddress = INPUT 1
          descrom[DESC_HSCFG_ADDR + 48-4] <= 8'h02;// bmAttributes = Bulk
          descrom[DESC_HSCFG_ADDR + 49-4] <= 8'h00;// wMaxPacketSize = 512 bytes
          descrom[DESC_HSCFG_ADDR + 50-4] <= 8'h02;
          descrom[DESC_HSCFG_ADDR + 51-4] <= 8'h00;// bInterval = 0 ms
          //----------------- Endpoint Descriptor -----------------
          descrom[DESC_HSCFG_ADDR + 52-4] <= 8'h07;// bLength = 7 bytes
          descrom[DESC_HSCFG_ADDR + 53-4] <= 8'h05;// bDescriptorType = endpoint descriptor
          descrom[DESC_HSCFG_ADDR + 54-4] <= 8'h01;// bEndpointAddress = OUTPUT 1
          descrom[DESC_HSCFG_ADDR + 55-4] <= 8'h02;// TransferType = Bulk
          descrom[DESC_HSCFG_ADDR + 56-4] <= 8'h00;// wMaxPacketSize = 512 bytes
          descrom[DESC_HSCFG_ADDR + 57-4] <= 8'h02;
          descrom[DESC_HSCFG_ADDR + 58-4] <= 8'h00;// bInterval = 0 ms





          // 1 byte // other_speed_configuration
          descrom[DESC_OSCFG_ADDR + 0] <= 8'h07;// Other Speed Configuration Descriptor replace HS/FS
          //descrom[DESC_FSCFG_ADDR + 1] <= 8'h02;// bDescriptorType = configuration descriptor
          //descrom[DESC_HSCFG_ADDR + 1] <= 8'h02;// bDescriptorType = configuration descriptor
          if(descrom_len > DESC_STRLANG_ADDR)begin
            // string descriptor 0 (supported languages)
            descrom[DESC_STRLANG_ADDR + 0] <= 8'h04;                // bLength = 4
            descrom[DESC_STRLANG_ADDR + 1] <= 8'h03;                // bDescriptorType = string descriptor
            descrom[DESC_STRLANG_ADDR + 2] <= 8'h09;
            descrom[DESC_STRLANG_ADDR + 3] <= 8'h04;         // wLangId[0] = 0x0409 = English U.S.

            descrom[DESC_STRVENDOR_ADDR + 0] <= 2 + 2*VENDORSTR_LEN;
            descrom[DESC_STRVENDOR_ADDR + 1] <= 8'h03;
            for(i = 0; i < VENDORSTR_LEN; i = i + 1) begin
                for(z = 0; z < 8; z = z + 1) begin
                    descrom[DESC_STRVENDOR_ADDR+ 2*i + 2][z] <= VENDORSTR[(VENDORSTR_LEN - 1 -i)*8+z];
                end
                descrom[DESC_STRVENDOR_ADDR+ 2*i + 3] <= 8'h00;
            end

            descrom[DESC_STRPRODUCT_ADDR + 0] <= 2 + 2*PRODUCTSTR_LEN;
            descrom[DESC_STRPRODUCT_ADDR + 1] <= 8'h03;
            for(i = 0; i < PRODUCTSTR_LEN; i = i + 1) begin
                for(z = 0; z < 8; z = z + 1) begin
                    descrom[DESC_STRPRODUCT_ADDR + 2*i + 2][z] <= PRODUCTSTR[(PRODUCTSTR_LEN - 1 - i)*8+z];
                end
                descrom[DESC_STRPRODUCT_ADDR + 2*i + 3] <= 8'h00;
            end

            descrom[DESC_STRSERIAL_ADDR + 0] <= 2 + 2*SERIALSTR_LEN;
            descrom[DESC_STRSERIAL_ADDR + 1] <= 8'h03;
            for(i = 0; i < SERIALSTR_LEN; i = i + 1) begin
                for(z = 0; z < 8; z = z + 1) begin
                    descrom[DESC_STRSERIAL_ADDR + 2*i + 2][z] <= SERIALSTR[(SERIALSTR_LEN - 1 - i)*8+z];
                end
                descrom[DESC_STRSERIAL_ADDR + 2*i + 3] <= 8'h00;
            end

//            descrom[DESC_STRINTRFC0_ADDR + 0] <= 2 + 2*INTRFC0STR_LEN;
//            descrom[DESC_STRINTRFC0_ADDR + 1] <= 8'h03;
//            for(i = 0; i < INTRFC0STR_LEN; i = i + 1) begin
//                for(z = 0; z < 8; z = z + 1) begin
//                    descrom[DESC_STRINTRFC0_ADDR + 2*i + 2][z] <= INTRFC0STR[(INTRFC0STR_LEN - 1 - i)*8+z];
//                end
//                descrom[DESC_STRINTRFC0_ADDR + 2*i + 3] <= 8'h00;
//            end
//
//            descrom[DESC_STRINTRFC1_ADDR + 0] <= 2 + 2*INTRFC1STR_LEN;
//            descrom[DESC_STRINTRFC1_ADDR + 1] <= 8'h03;
//            for(i = 0; i < INTRFC1STR_LEN; i = i + 1) begin
//                for(z = 0; z < 8; z = z + 1) begin
//                    descrom[DESC_STRINTRFC1_ADDR + 2*i + 2][z] <= INTRFC1STR[(INTRFC1STR_LEN - 1 - i)*8+z];
//                end
//                descrom[DESC_STRINTRFC1_ADDR + 2*i + 3] <= 8'h00;
//            end

        end
      end
      else begin
          descrom[8]  <= ((i_pid != 16'h0000)&&(i_pid != 16'hFFFF)) ? i_pid[7:0]  : VENDORID[7 : 0];// idVendor
          descrom[9]  <= ((i_pid != 16'h0000)&&(i_pid != 16'hFFFF)) ? i_pid[15:8] : VENDORID[15 : 8];
          descrom[10] <= ((i_vid != 16'h0000)&&(i_vid != 16'hFFFF)) ? i_vid[7:0]  : PRODUCTID[7 : 0];// idProduct
          descrom[11] <= ((i_vid != 16'h0000)&&(i_vid != 16'hFFFF)) ? i_vid[15:8] : PRODUCTID[15 : 8];
      end
    assign o_descrom_rdat = descrom[i_descrom_raddr];
endmodule
