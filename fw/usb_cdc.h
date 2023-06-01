
#ifndef USB_CDC_H_INCLUDED
#define USB_CDC_H_INCLUDED

#include <stdint.h>

typedef struct {
    volatile uint32_t data       ;
    volatile uint32_t rx_status  ;
    volatile uint32_t tx_status  ;
    volatile uint32_t usb_tx_ena ;
} usb_cdc_regs_t;

#define USB_CDC_RX_NOT_EMPTY    (0x1)
#define USB_CDC_RX_NUM_POS      (1)
#define USB_CDC_RX_NUM_MASK     (0xFFFF)

#define USB_CDC_TX_NOT_FULL     (0x1)
#define USB_CDC_TX_NUM_POS      (1)
#define USB_CDC_TX_NUM_MASK     (0xFFFF)

#define USB_CDC_TX_WAIT_EMPTY(dev) \
    while ((dev->tx_status >> 1) != 0)

#define USB_CDC_TX_FIFO_SIZE    (32)

extern char usb_cdc_getc(usb_cdc_regs_t *dev);
extern void usb_cdc_putc(usb_cdc_regs_t *dev, char c);
extern void usb_cdc_send_data_block(usb_cdc_regs_t *dev, const uint8_t* data, int block_len);
extern void usb_cdc_send_data(usb_cdc_regs_t *dev, const uint8_t* data, int len);
extern void usb_cdc_recv_data(usb_cdc_regs_t *dev, uint8_t* buf, int len);
extern void usb_cdc_flush(usb_cdc_regs_t *dev, int len);

#endif
