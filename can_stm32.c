#include <soc.h>
#include <zephyr/drivers/can.h>

void can_stm32_configure_filter(CAN_TypeDef * can, can_stm32_filter_struct_t * filter)
{
    /* enter filter init mode */
    can->FMR |= CAN_FMR_FINIT;

    /* configure filter bank for CAN 1 and CAN 2 */
    /* configure 27 filter bank for CAN 1 and 1 filter bank for CAN 2 */
    can->FMR |= (filter->filter_bank_assign_to_can_master) << CAN_FMR_CAN2SB_Pos;

    /* configure all filter bank under mask mode that mean comming message no need to map 1:1 */
    can->FM1R |= (filter->filter_mode << filter->filter_active);

    /* configure filter scable */
    can->FS1R |= (filter->filter_scale << filter->filter_active);

    /* configure pass message to which FIFO */
    /* a half passing filter message to FIFO 0, otherwise will move to FIFO 1 */
    can->FFA1R |= filter->filter_fifo_assignment << filter->filter_active;

    /* configure id for actived filter bank */
    can->sFilterRegister[filter->filter_active].FR1 = filter->filter_id;
    can->sFilterRegister[filter->filter_active].FR2 = filter->filter_mask;

    /* active filter */
    can->FA1R |= 1 << filter->filter_active;

    /* disable filter init mode */
    can->FMR &= ~CAN_FMR_FINIT;
}

static void can_stm32_enter_init_mode(CAN_TypeDef * can)
{
    /* enter init mode */
    can->MCR |= CAN_MCR_INRQ;
    while((can->MSR & CAN_MSR_INAK) == 0U){};
}

void can_stm32_exit_init_mode(CAN_TypeDef * can)
{
    /* leave init mode */
    can->MCR &= ~CAN_MCR_INRQ;
    while((can->MSR & CAN_MSR_INAK) != 0U){};
}

static void can_stm32_exit_sleep_mode(CAN_TypeDef * can)
{
    /* exit sleep mode */
    can->MCR &= ~CAN_MCR_SLEEP;
    while((can->MSR & CAN_MSR_SLAK) != 0U){};
}

static void can_stm32_configure_bit_timing(CAN_TypeDef * can, can_stm32_bit_timing_struct_t * bit_timing)
{
    /* set bit timing */
    can->BTR |= (bit_timing->baudrate_prescaler) | \
                (bit_timing->time_segment_1) | \
                (bit_timing->time_segment_2) | \
                (bit_timing->resync_jump_width);
}

static void can_stm32_calculate_bit_timing(uint32_t bit_rate, uint32_t sample_point, can_stm32_bit_timing_struct_t * bit_timing)
{

}

void can_stm32_init(CAN_TypeDef * can)
{
    uint32_t bit_rate = 0;
    uint32_t sample_point = 0;
    can_stm32_bit_timing_struct_t bit_timing;

    /* enter init mode */
    can_stm32_enter_init_mode(can);

    /* calculate bit timming */
    can_stm32_calculate_bit_timing(bit_rate, sample_point, &bit_timing);

    /* configure bit timming */
    can_stm32_configure_bit_timing(can, &bit_timing);

    /* exit sleep mode */
    can_stm32_exit_sleep_mode(can);

}

void can_stm32_set_mode(CAN_TypeDef * can, can_stm32_mode_enum_t can_mode)
{
    switch(can_mode)
    {
        case MODE_SILENT:
            can->BTR |= CAN_BTR_SILM;
            break;
        case MODE_LOOPBACK:
            can->BTR |= CAN_BTR_LBKM;
            break;
        default:
            break;
    }
}

void can_stm32_send_internal(CAN_TypeDef * can, can_frame * frame)
{
    uint8_t tx_index_mailbox = 0;

    /* check if mailbox is empty */
    for(tx_index_mailbox = 0; tx_index_mailbox < 2; tx_index_mailbox ++)
    {
        /* in case tx mailbox empty */
        if((can->sTxMailBox[tx_index_mailbox].TIR & CAN_TI0R_TXRQ) == 0U)
        {
        	can->sTxMailBox[tx_index_mailbox].TIR = 0;

            /* fullfill id */
            can->sTxMailBox[tx_index_mailbox].TIR |= (frame->id) << CAN_TI0R_EXID_Pos;

            /* select standard id or extend id */
            /* default configure as extended id */
            can->sTxMailBox[tx_index_mailbox].TIR |= CAN_TI0R_IDE;

            /* remote frame or data frame */
            can->sTxMailBox[tx_index_mailbox].TIR &= ~CAN_TI0R_RTR;

            /* configure data len */
            /* bxCAN only support 8 byte data */
            can->sTxMailBox[tx_index_mailbox].TDTR = frame->dlc;

            /* write data to TX mail box */

            /* write 4 low byte */
            can->sTxMailBox[tx_index_mailbox].TDLR = (frame->data[0] | frame->data[1] << 8 | frame->data[2] << 16 | frame->data[3] << 24);

            /* write 4 high byte */
            can->sTxMailBox[tx_index_mailbox].TDHR = (frame->data[4] | frame->data[5] << 8 | frame->data[6] << 16 | frame->data[7] << 24);

            /* request send */
            can->sTxMailBox[tx_index_mailbox].TIR |= CAN_TI0R_TXRQ;
            /* polling waiting for message already sent */
            while((can->sTxMailBox[tx_index_mailbox].TIR & CAN_TI0R_TXRQ) != 0U){};

            break;
 
        }
    }
}

void can_stm32_recv(CAN_TypeDef * can, can_frame * recv_frame)
{
    uint8_t rx_index_mailbox = 0;
    uint8_t sub_index = 0;
    volatile uint32_t * FIFO_array[2] = {(volatile uint32_t *)&can->RF0R, (volatile uint32_t *)&can->RF1R};

    /* polling waiting for message come */
    while(((can->RF0R & CAN_RF0R_FMP0) == 0U) && \
          ((can->RF1R & CAN_RF1R_FMP1) == 0U)){};

    /* FIFO 0 have message */
    if((can->RF0R & CAN_RF0R_FMP0) != 0U)
    {
        rx_index_mailbox = 0;
    }
    /* FIFO 1 have message */
    else if((can->RF1R & CAN_RF1R_FMP1) != 0U)
    {
        rx_index_mailbox = 1;
    }
    else
    {
        /* no message */
    }

    while((*(volatile uint32_t *)FIFO_array[rx_index_mailbox] & CAN_RF0R_FMP0) != 0U)
    {
        /* get id */
        recv_frame->id = can->sFIFOMailBox[rx_index_mailbox].RIR >> CAN_RI0R_EXID_Pos;
        /* get fram type */
        recv_frame->flags = can->sFIFOMailBox[rx_index_mailbox].RIR & CAN_RI0R_RTR;
        /* get data len */
        recv_frame->flags = can->sFIFOMailBox[rx_index_mailbox].RDTR & CAN_RDT0R_DLC;
        /* get 4 low byte */
        for(sub_index = 0; sub_index < 4; sub_index ++)
        {
            recv_frame->data[sub_index] = (can->sFIFOMailBox[rx_index_mailbox].RDLR >> (sub_index * 8)) & 0xFF;
        }
        /* get 4 high byte */
        for(sub_index = 0; sub_index < 4; sub_index ++)
        {
            recv_frame->data[sub_index + 4] = (can->sFIFOMailBox[rx_index_mailbox].RDHR >> (sub_index * 8)) & 0xFF;
        }
        /* release RX FIFO mailbox */
        *(volatile uint32_t *)(FIFO_array[rx_index_mailbox]) |= CAN_RF0R_RFOM0;
    }
}



/* can zephyr api mapping from device tree */
static int can_stm32_start(const struct device *dev)
{
    const struct can_stm32_config *cfg = (can_stm32_config *)dev->config;
    CAN_TypeDef *can = cfg->can;

    can_stm32_exit_init_mode(can);

    return 0;
}

static int can_stm32_stop(const struct device *dev)
{
    const struct can_stm32_config *cfg = (can_stm32_config *)dev->config;
    CAN_TypeDef *can = cfg->can;

    can_stm32_enter_init_mode(can);

    return 0;
}

static int can_stm32_set_mode(const struct device *dev, can_mode_t mode)
{
    const struct can_stm32_config *cfg = (can_stm32_config *)dev->config;
    CAN_TypeDef *can = cfg->can;

    can_stm32_set_mode(can, mode);

    return 0;
}

static int can_stm32_configure_timing(const struct device *dev, const struct can_timing *timing)
{
    const struct can_stm32_config *cfg = (can_stm32_config *)dev->config;
    CAN_TypeDef *can = cfg->can;
    can_stm32_bit_timing_struct_t bit_timing;

    /* configure bit timing */
    can_stm32_configure_bit_timing(can, &bit_timing);

    return 0;
}

static int can_stm32_send(const struct device *dev,                        \
                          const struct can_frame *frame,                   \
                          k_timeout_t timeout, can_tx_callback_t callback, \
                          void *user_data)
{
    const struct can_stm32_config *cfg = (can_stm32_config *)dev->config;
    CAN_TypeDef *can = cfg->can;

    can_stm32_send_internal(can, frame);

    return 0;
}

static const struct can_driver_api can_api_funcs = {
	.start = can_stm32_start,
	.stop = can_stm32_stop,
	.set_mode = can_stm32_set_mode,
	.set_timing = can_stm32_configure_timing,
	.send = can_stm32_send,
	.timing_min = {
		.sjw = 0x1,
		.prop_seg = 0x00,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x04,
		.prop_seg = 0x00,
		.phase_seg1 = 0x10,
		.phase_seg2 = 0x08,
		.prescaler = 0x400
	}
};



























































