
#include "usb_msd.h"

static void usb_msd_udelay(int t) {
	volatile int us = 1;
	while (t--) {
		us = 1;
		while (us--);
	}
}


void usb_reset(usb_ctrl_regs_t* dev, int delay_val)
{
    dev->reset = 1;
    usb_msd_udelay(delay_val);
    dev->reset = 0;
    usb_msd_udelay(delay_val);
}


static void usb_msd_wait_recv_data(usb_msd_dma_regs_t *dev, uint32_t wait_amount)
{
    uint32_t rx_dnum;
    while (1) {
        rx_dnum = (dev->rx_status >> MSD_DMA_RX_STATUS_DAT_LEN_POS);
        if (rx_dnum >= wait_amount)
            break;
    }
}


static void usb_msd_wait_rx_dma_done(usb_msd_dma_regs_t *dev)
{
    while (1) {
        usb_msd_udelay(2);
        uint32_t rx_status = dev->rx_status;
        if (rx_status & MSD_DMA_RX_STATUS_IDLE)
            break;
    }
}

void usb_msd_recv_data_no_wait(usb_msd_dma_regs_t* dev, void *dst, uint32_t data_len)
{
    dev->rx_dma_start_adr = (uint32_t)dst;
    dev->rx_dma_dat_len = (data_len - 1) | MSD_DMA_RX_DAT_LEN_START;
    usb_msd_wait_rx_dma_done(dev);
}

void usb_msd_recv_data(usb_msd_dma_regs_t *dev, void *dst, uint32_t data_len)
{
    usb_msd_wait_recv_data(dev, data_len);
    dev->rx_dma_start_adr = (uint32_t)dst;
    dev->rx_dma_dat_len = (data_len - 1) | MSD_DMA_RX_DAT_LEN_START;
    usb_msd_wait_rx_dma_done(dev);
}


static void usb_msd_wait_tx_dma_done(usb_msd_dma_regs_t *dev)
{
    while (1) {
        if (dev->tx_status & MSD_DMA_TX_STATUS_IDLE)
            break;
    }
}

#define TX_FIFO_SIZE 1025

void usb_msd_wait_send_data(usb_msd_dma_regs_t *dev)
{
    // TODO: add empty flag!
    uint32_t tx_dnum;
    while (1) {
        tx_dnum = (dev->tx_status >> MSD_DMA_TX_STATUS_DAT_LEN_POS);
        if (tx_dnum == TX_FIFO_SIZE)
            break;
    }
}

void usb_msd_wait_awail_space(usb_msd_dma_regs_t *dev, int space_avail)
{
    uint32_t tx_avail_space;
    while (1) {
        tx_avail_space = (dev->tx_status >> MSD_DMA_TX_STATUS_DAT_LEN_POS);
        if (tx_avail_space >= space_avail)
            break;
    }
}

void usb_msd_send_data(usb_msd_dma_regs_t* dev, const void *src, uint32_t data_len)
{
    // disable usb tx
    dev->tx_control = dev->tx_control & 0xfffffffd;
    dev->tx_dma_start_adr = (uint32_t)src;
    dev->tx_dma_dat_len = (data_len - 1) | MSD_DMA_TX_DAT_LEN_START;
    usb_msd_wait_tx_dma_done(dev);
    // enable usb tx
    dev->tx_control = dev->tx_control | 0x00000002;
    usb_msd_wait_send_data(dev);
    dev->tx_control = dev->tx_control & 0xfffffffd;
}
