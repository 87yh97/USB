
#ifndef USB_SCSI_H_INCLUDED
#define USB_SCSI_H_INCLUDED

#include <stdint.h>
#include "usb_msd.h"

typedef struct {
    uint32_t signature         ;
    uint32_t tag               ;
    uint32_t data_transfer_len ;
    uint8_t  flags             ;
    uint8_t  lun               ;
    uint8_t  cmd_block_len     ;
    uint8_t  cmd_block [16]    ;
} __attribute__((packed)) scsi_cbw_t;

typedef struct {
    uint32_t signature         ;
    uint32_t tag               ;
    uint32_t data_residue      ;
    uint8_t  status            ;
} __attribute__((packed)) scsi_csw_t;

#define SCSI_CBW_LEN  (31)
#define SCSI_CBS_LEN  (13)

void scsi_set_usb_ctrl_dev(usb_ctrl_regs_t *dev);
void scsi_set_msd_dev(usb_msd_dma_regs_t *dev);
void scsi_execute_commad(void *rx_buf);

#endif
