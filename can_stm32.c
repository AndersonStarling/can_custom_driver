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
    for(tx_index_mailbox = 0; tx_index_mailbox < 3; tx_index_mailbox ++)
    {
        /* in case tx mailbox empty */
        if(can->sTxMailBox[tx_index_mailbox].TIR & CAN_TI0R_TXRQ == 0U)
        {
            /* fullfill id */
            can->sTxMailBox[tx_index_mailbox].TIR = (frame->id) << CAN_TI0R_EXID_Pos;
        }
    }
}

























