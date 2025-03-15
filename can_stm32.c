#include <soc.h>

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t flags;
    uint8_t data[8];
} can_frame;

void can_stm32_init(CAN_TypeDef * can)
{
    /* enter init mode */
    can->MCR |= CAN_MCR_INRQ;
    while(can->MSR & CAN_MSR_INAK == 0U){};

    /* exit sleep mode */
    can->MCR &= ~CAN_MCR_SLEEP;
    while(can->MSR & CAN_MSR_SALK == 0U){};

    /* set bit timing */
    /* dummy waiting for re-calculation clock */
    can->BTR |= (CAN_BTR_BRP) | (CAN_BTR_TS1) | (CAN_BTR_TS2) | (CAN_BTR_SJW);


    /* allow CPU access to filter */
    can->FMR |= CAN_FMR_FINIT;

    /* configure filter bank for CAN 1 and CAN 2 */
    /* configure 27 filter bank for CAN 1 and 1 filter bank for CAN 2 */
    can->FMR |= 0x11011u << CAN_FMR_CAN2SB_Pos;

    /* configure mode for all filter bank */
    /* configure all filter bank under list mode that mean comming message need to map 1:1 */
    can->FM1R = 0xFFFFFFFu;

    /* configure filter scable */
    /* configure all filter bank for 32 bit mask mode */
    can->FS1R = 0xFFFFFFFu;

    /* configure pass message to which FIFO */
    /* a half passing filter message to FIFO 0, otherwise will move to FIFO 1 */
    can->FFA1R = 0x1FFFu;

    /* configure all filter active */
    can->FA1R = 0xFFFFFFFu;

    /* configure id for all filter bank */

    /* configure filter bank 0 */
    /* dummy configure all matched id = 0 */
    for (int i = 0; i < 28; i++) {
        can->sFilterRegister[i].FR1 = 0x0u;
        can->sFilterRegister[i].FR2 = 0x0u;
    }

    /* leave init mode */
    can->MCR &= ~CAN_MCR_INRQ;
    while(can->MSR & CAN_MSR_INAK != 0U){};

}

void can_stm32_send(CAN_TypeDef * can, can_frame * frame)
{
    uint8_t tx_index_mailbox = 0;

    /* check if mailbox is empty */
    for(tx_index_mailbox = 0; tx_index_mailbox < 2; tx_index_mailbox ++)
    {
        /* in case tx mailbox empty */
        if(can->sTxMailBox[tx_index_mailbox].TIR & CAN_TI0R_TXRQ == 0U)
        {
            /* fullfill id */
            can->sTxMailBox[tx_index_mailbox].TIR |= (frame->id) << CAN_TI0R_EXID_Pos;

            /* select standard id or extend id */
            /* default configure as extended id */
            can->sTxMailBox[tx_index_mailbox].TIR = CAN_TI0R_IDE;

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
            while(can->sTxMailBox[tx_index_mailbox].TIR & CAN_TI0R_TXRQ != 0U){};

            break;
 
        }
    }
}

void can_stm32_recv(CAN_TypeDef * can, can_frame * recv_frame)
{
    uint8_t rx_index_mailbox = 0;
    uint8_t sub_index = 0;

    /* polling waiting for message come */
    while(can->IER & CAN_IER_FMPIE0 == 0U || \
          can->IER & CAN_IER_FMPIE1 == 0U){};

    for(rx_index_mailbox =  0; rx_index_mailbox < 3; rx_index_mailbox ++)
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
    }
}

























