
`default_nettype none

interface usb_gowin_if;
    logic [3:0]  endpt          ;
    logic        setup          ;
    logic [7:0]  txdat          ;
    logic        txcork         ;
    logic        txval          ;
    logic [11:0] txdat_len      ;
    logic        txpop          ;
    logic        txact          ;
    logic  [7:0] rxdat          ;
    logic        rxval          ;
    logic        rxact          ;
    logic        rxrdy          ;
    logic        ack_tout       ;
    logic        ack_received   ;
    logic        ack_bad_packet ;
    logic [3:0]  transact_state_o;


    modport user (
        input  endpt            ,
        input  setup            ,
        output txdat            ,
        output txcork           ,
        output txval            ,
        output txdat_len        ,
        input  txpop            ,
        input  txact            ,
        input  rxdat            ,
        input  rxval            ,
        input  rxact            ,
        output rxrdy            ,
        input  ack_tout         ,
        input  ack_received     ,
        input  ack_bad_packet   ,
        input transact_state_o
    );

    modport gwn (
        output endpt        ,
        output setup        ,
        input  txdat        ,
        input  txcork       ,
        input  txval        ,
        input  txdat_len    ,
        output txpop        ,
        output txact        ,
        output rxdat        ,
        output rxval        ,
        output rxact        ,
        input  rxrdy        ,
        output ack_tout     ,
        output ack_received ,
        output ack_bad_packet,
        output transact_state_o
    );

    modport mon (
        output endpt        ,
        output setup        ,
        output txdat        ,
        output txcork       ,
        output txval        ,
        output txdat_len    ,
        output txpop        ,
        output txact        ,
        output rxdat        ,
        output rxval        ,
        output rxact        ,
        output rxrdy        ,
        output ack_tout     ,
        output ack_received ,
        output ack_bad_packet,
        output transact_state_o
    );

endinterface

`resetall
