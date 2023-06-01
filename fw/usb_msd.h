
#ifndef USB_MSD_H_INDLUDED
#define USB_MSD_H_INDLUDED

#include <stdint.h>

typedef struct {
    volatile uint32_t reset             ;
    volatile uint32_t setup_req_status  ;
    volatile uint32_t ack_tout          ;
    volatile uint32_t ack_received      ;
    volatile uint32_t ack_bad_packet    ;
} usb_ctrl_regs_t;

typedef struct {
    volatile uint32_t rx_dma_start_adr ;
    volatile uint32_t rx_dma_dat_len   ;
    volatile uint32_t rx_status        ;
    volatile uint32_t tx_dma_start_adr ;
    volatile uint32_t tx_dma_dat_len   ;
    volatile uint32_t tx_status        ;
    volatile uint32_t tx_control       ;
} usb_msd_dma_regs_t;

#define MSD_DMA_RX_STATUS_IDLE         0x1
#define MSD_DMA_RX_STATUS_FULL_ENOUGH  0x2
#define MSD_DMA_RX_STATUS_EMPRY        0x8
#define MSD_DMA_RX_STATUS_DAT_LEN_POS  4

#define MSD_DMA_RX_DAT_LEN_START       0x80000000

#define MSD_DMA_TX_STATUS_IDLE         0x1
#define MSD_DMA_TX_STATUS_EMPTY_ENOUGH 0x2
#define MSD_DMA_TX_STATUS_FULL         0x8
#define MSD_DMA_TX_STATUS_DAT_LEN_POS  4

#define MSD_DMA_TX_DAT_LEN_START       0x80000000

void usb_reset(usb_ctrl_regs_t* dev, int delay_val);
void usb_msd_recv_data(usb_msd_dma_regs_t* dev, void *dst, uint32_t data_len);
void usb_msd_recv_data_no_wait(usb_msd_dma_regs_t* dev, void *dst, uint32_t data_len);

void usb_msd_send_data(usb_msd_dma_regs_t* dev, const void *src, uint32_t data_len);
void usb_msd_wait_send_data(usb_msd_dma_regs_t *dev);
void usb_msd_wait_awail_space(usb_msd_dma_regs_t *dev, int space_avail);

#endif
