
#include "usb_scsi.h"
#include "defecto_sdc.h"
#include "debug.h"

//
// Pointer to USB ctrl regs
//

static usb_ctrl_regs_t* USB_CTRL;

int delay(int N) {
    volatile int t = 0;
    for (int i=0; i<N; i++)
    {
        t++;
        DBG_PRINT("?");
    }
    return t;
}

void scsi_set_usb_ctrl_dev(usb_ctrl_regs_t *dev) {
    USB_CTRL = dev;
}

//
// Pointer to MSD device
//

static usb_msd_dma_regs_t* MSD_DEV;

void scsi_set_msd_dev(usb_msd_dma_regs_t *dev) {
    MSD_DEV = dev;
}

//
// SCSI request & response buffers
//

static scsi_cbw_t scsi_cbw_buf;

static scsi_csw_t scsi_csw_buf = {
    0x53425355,
    0,
    0,
    0
};

//
// SCSI command codes
//

#define SCSI_INQUIRY                      (0x12)
#define SCSI_REQUEST_SENSE                (0x03)
#define SCSI_READ_CAPACITY_10             (0x25)
#define SCSI_MODE_SENSE_6                 (0x1A)
#define SCSI_READ_10                      (0x28)
#define SCSI_WRITE_10                     (0x2A)
#define SCSI_TEST_UNIT_READY              (0x00)
#define SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL (0x1E)
#define SCSI_READ_FORMAT_CAPACITY         (0x23)

#define SCSI_CSW_STATUS_OK                (0x00)
#define SCSI_CSW_STATUS_FAIL              (0x01)

void scsi_unsupported_command(const scsi_cbw_t* scsi_cbw)
{
    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len;
    // Send fail status
    scsi_csw_buf.status = SCSI_CSW_STATUS_FAIL;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
    // Confirm
    // scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    // usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}

static uint8_t SCSI_INQUIRY_RESP[36] =
{
        0x00,           // Block device
        0x80,           // Removable media
        0x02,           // SPC-2
        0x02,           // Response data format = 0x02
        0x1F,           // Additional_length = length - 5
        0x00,
        0x00,
        0x00,
        'D', 'e', 'f', 'e', 'c', 't', 'o', ' ',
        'M', 'a', 's', 's', ' ', 'S', 't', 'o', 'r', 'a', 'g', 'e', ' ', ' ', ' ', ' ',
        '1', '.', '0', '0'
};

uint8_t SCSI_INQUIRY_SERIAL_NUMBER_RESP[45] = {
    0x00, 0x80, 0x02, 0x02, 0x1f,
    0x00, 0x00, 0x00, 0x4b, 0x69, 0x6e, 0x67, 0x73, 0x74, 0x6f, 0x6e, 0x44, 0x54, 0x20, 0x31, 0x30,
    0x31, 0x20, 0x47, 0x32, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x31, 0x2e, 0x30, 0x30, 0x36,
    0x53, 0x44, 0x53, 0x97, 0x50, 0x00, 0x00, 0x00
};

void scsi_handle_inquiry(const scsi_cbw_t* scsi_cbw)
{
    // check EVPD и CMDDT
    if (scsi_cbw->cmd_block[1] == 0) { // standart response
        usb_msd_send_data(MSD_DEV, SCSI_INQUIRY_RESP, scsi_cbw->cmd_block[4]);
        // send ok status
        scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - scsi_cbw->cmd_block[4];
        scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
        usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
    } else {
        usb_msd_send_data(MSD_DEV, SCSI_INQUIRY_SERIAL_NUMBER_RESP, 45);
        // send ok status
        scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - scsi_cbw->cmd_block[4];
        scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
        usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
    }
}

static uint8_t SCSI_SENSE_DATA_RESP[18] = {
    0xf0,       //VALID = 1, RESRONSE_CODE = 0x70
    0x00,
    0x05,       //S_ILLEGAL_REQUEST
    0x00, 0x00, 0x00, 0x00,
    0x0b,
    0x00, 0x00, 0x00, 0x00,
    0x20, 0x00,
    0x00, 0x00, 0x00, 0x00
};

void scsi_handle_request_sense(const scsi_cbw_t* scsi_cbw)
{
    usb_msd_send_data(MSD_DEV, SCSI_SENSE_DATA_RESP, 18);
    // send ok status
    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - 18;
    scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}

void scsi_send_ok_status(const scsi_cbw_t* scsi_cbw)
{
    // send ok status
    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - scsi_cbw->cmd_block[4];
    scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}

static uint8_t SCSI_READ_CAPACITY_RESP[8] = {
        // RETURNED LOGICAL BLOCK ADDRESS
        //0x00, 0x00, 0x0F, 0xFF,    //Addr last blocks = 2M/512 - 1
        // 0x07, 0xff, 0xff, 0xff,     //Addr last blocks = 2M/512 - 1
        0x0E, 0x8F, 0xEF, 0xFF,
        // 0x07, 0x47, 0xEF, 0xFF,
        // 0x07, 0x47, 0xFC, 0xB3,
        // 0x07, 0x47, 0xF7, 0xFF,
        // 0x0E, 0x8F, 0xF9, 0x67,
        // 0x00, 0x3B, 0x9A, 0xC9,
        // 0x00, 0x3A, 0x87, 0xff,
        // 0x01, 0xda, 0x87, 0xff,
        // BLOCK LENGTH IN BYTES
        0x00, 0x00, 0x02, 0x00       //Size blocks = 512 bytes
};

void scsi_handle_read_capacity(const scsi_cbw_t* scsi_cbw)
{
    usb_msd_send_data(MSD_DEV, SCSI_READ_CAPACITY_RESP, 8);
    // send ok status
    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - 8;
    scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}

uint8_t SCSI_MODE_SENSE_RESP[4] = {
        0x03, 0x00, 0x80, 0x00
};

// uint8_t SCSI_MODE_SENSE_RESP[12] = {
//     0x0C, 0x00, 0x80, 0x08,
//     // 0x0E, 0x8F, 0xF9, 0x68,
//     // 0x07, 0x47, 0xFC, 0xB4,
//     0x07, 0x47, 0xF0, 0x00,
//     0x00,//density code
//     0x00, 0x02, 0x00 //block length = 512
// };

void scsi_handle_read_mode_sense(const scsi_cbw_t* scsi_cbw)
{
    usb_msd_send_data(MSD_DEV, SCSI_MODE_SENSE_RESP, 4);
    // send ok status
    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - 4;
    // scsi_csw_buf.data_residue = 0;
    scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}


uint8_t sd_data_buf [512];

int blk_cnt = 0;
int blk_err_cnt = 0;
int retry_cnt = 0;

void scsi_handle_read(const scsi_cbw_t* scsi_cbw)
{
    sdc_dev_t * sdc = sdc_get_dev();
    int cnt = 1;
    uint32_t first_block, num_of_blocks, rest_of_blocks;
    first_block = ((scsi_cbw->cmd_block[2] << 24) |
                   (scsi_cbw->cmd_block[3] << 16) |
                   (scsi_cbw->cmd_block[4] << 8) |
                   (scsi_cbw->cmd_block[5]));
    num_of_blocks = ((scsi_cbw->cmd_block[7] << 8) |
                      scsi_cbw->cmd_block[8]);

    MSD_DEV->tx_control = 0x00;
    USB_CTRL->ack_tout = 0;
    //first_block += 2048;

    for (int i = 0; i < num_of_blocks; ++i) {
        // blk_cnt++;
        cnt = 1;
        sdc_read_single_block(sdc, first_block + i, (void *)sd_data_buf);
        usb_msd_send_data(MSD_DEV, sd_data_buf, 512);

        // if (USB_CTRL->ack_tout)
        //     blk_err_cnt++;
        delay(10);                   // wait for block ack (from PC) arrival
        while (USB_CTRL->ack_tout) {
            delay(10);
            USB_CTRL->ack_tout = 0;
            // retry_cnt++;
            usb_msd_send_data(MSD_DEV, sd_data_buf, 512);
        }
    }

    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - num_of_blocks * 512;
    scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}


void scsi_handl_test_unit_ready(const scsi_cbw_t* scsi_cbw)
{
    // send ok status
    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len;
    scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}

uint8_t SCSI_READ_FORMAT_CAPACITY_RESP[12] = {
    0x00, 0x00, 0x00,
    0x08,
    // 0x00, 0x00, 0x00, 0x80,
    // 0x07, 0x47, 0xFC, 0xB4,
    // 0x07, 0x47, 0xF0, 0x00,

    0x0E, 0x8F, 0xF9, 0x68,
    0x02,
    0x00, 0x02, 0x00
};

void scsi_read_format_capacity(const scsi_cbw_t* scsi_cbw)
{
    usb_msd_send_data(MSD_DEV, SCSI_READ_FORMAT_CAPACITY_RESP, 12);
    // send ok status
    scsi_csw_buf.data_residue = scsi_cbw->data_transfer_len - scsi_cbw->cmd_block[4];
    scsi_csw_buf.status = SCSI_CSW_STATUS_OK;
    usb_msd_send_data(MSD_DEV, &scsi_csw_buf, SCSI_CBS_LEN);
}

void scsi_execute_commad(void *rx_buf)
{
    scsi_cbw_t* scsi_cbw = (scsi_cbw_t*) rx_buf;

    MSD_DEV->tx_control = 0x00;

    // copy tag
    scsi_csw_buf.tag = scsi_cbw->tag;
    switch (scsi_cbw->cmd_block[0]) {
        case SCSI_INQUIRY:
            scsi_handle_inquiry(scsi_cbw);
            break;

        case SCSI_TEST_UNIT_READY:
            scsi_send_ok_status(scsi_cbw);
            break;

        case SCSI_REQUEST_SENSE:
            scsi_handle_request_sense(scsi_cbw);
            break;

        case SCSI_READ_CAPACITY_10:
            scsi_handle_read_capacity(scsi_cbw);
            break;

        case SCSI_READ_FORMAT_CAPACITY:
            scsi_read_format_capacity(scsi_cbw);
            break;

        case SCSI_MODE_SENSE_6:
            scsi_handle_read_mode_sense(scsi_cbw);
            break;

        case SCSI_READ_10:
            scsi_handle_read(scsi_cbw);
            break;

        // Unsupported command
        default:
            scsi_unsupported_command(scsi_cbw);
            break;
    }
}
