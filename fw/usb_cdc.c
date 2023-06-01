
#include "usb_cdc.h"

char usb_cdc_getc(usb_cdc_regs_t *dev)
{
    while (0 == (dev->rx_status & USB_CDC_RX_NOT_EMPTY));
    return (char)dev->data;
}

void usb_cdc_putc(usb_cdc_regs_t *dev, char c)
{
    while (0 == (dev->tx_status & USB_CDC_TX_NOT_FULL));
    dev->data = c;
    dev->usb_tx_ena = 1;
    USB_CDC_TX_WAIT_EMPTY(dev);
    dev->usb_tx_ena = 0;
}

void usb_cdc_send_data_block(usb_cdc_regs_t *dev, const uint8_t* data, int block_len)
{
    int i = 0;
    do {
        if (dev->tx_status & USB_CDC_TX_NOT_FULL)
            dev->data = data[i++];
    } while (i != block_len);
    dev->usb_tx_ena = 1;
    USB_CDC_TX_WAIT_EMPTY(dev);
    dev->usb_tx_ena = 0;
}

void usb_cdc_send_data(usb_cdc_regs_t *dev, const uint8_t* data, int len)
{
    int rest = len;
    while (rest >= USB_CDC_TX_FIFO_SIZE) {
        usb_cdc_send_data_block(dev, data, USB_CDC_TX_FIFO_SIZE);
        data += USB_CDC_TX_FIFO_SIZE;
        rest -= USB_CDC_TX_FIFO_SIZE;
    }
    if (rest != 0) {
        usb_cdc_send_data_block(dev, data, rest);
    }
}

void usb_cdc_recv_data(usb_cdc_regs_t *dev, uint8_t* buf, int len)
{
    int i = 0;
    do {
        if (dev->rx_status & USB_CDC_RX_NOT_EMPTY)
            buf[i++] = dev->data;
    } while (i != len);
}

__attribute__((optimize("O0"))) void usb_cdc_flush(usb_cdc_regs_t *dev, int len)
{
    volatile uint32_t buf;
    volatile int i = 0;
    do {
        // if (dev->rx_status & USB_CDC_RX_NOT_EMPTY) {
            buf = dev->data;
            ++i;
        // }
    } while (i != len);
}
