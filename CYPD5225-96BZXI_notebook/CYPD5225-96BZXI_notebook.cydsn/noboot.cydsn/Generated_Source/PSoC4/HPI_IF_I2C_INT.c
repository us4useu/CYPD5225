/***************************************************************************//**
* \file HPI_IF_I2C_INT.c
* \version 4.0
*
* \brief
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in I2C mode.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "HPI_IF_PVT.h"
#include "HPI_IF_I2C_PVT.h"
#include "cyapicallbacks.h"


/*******************************************************************************
* Function Name: HPI_IF_I2C_ISR
****************************************************************************//**
*
*  Handles the Interrupt Service Routine for the SCB I2C mode.
*
*******************************************************************************/
CY_ISR(HPI_IF_I2C_ISR)
{
    uint32 diffCount;
    uint32 endTransfer;

#ifdef HPI_IF_I2C_ISR_ENTRY_CALLBACK
    HPI_IF_I2C_ISR_EntryCallback();
#endif /* HPI_IF_I2C_ISR_ENTRY_CALLBACK */

#if (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST)
    uint32 response;

    response = HPI_IF_I2C_ACK_ADDR;
#endif /* (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST) */

    endTransfer = 0u; /* Continue active transfer */

    /* Calls customer routine if registered */
    if(NULL != HPI_IF_customIntrHandler)
    {
        HPI_IF_customIntrHandler();
    }

    if(HPI_IF_CHECK_INTR_I2C_EC_MASKED(HPI_IF_INTR_I2C_EC_WAKE_UP))
    {
        /* Mask-off after wakeup */
        HPI_IF_SetI2CExtClkInterruptMode(HPI_IF_NO_INTR_SOURCES);
    }

    /* Master and Slave error tracking:
    * Add the master state check to track only the master errors when the master is active or
    * track slave errors when the slave is active or idle.
    * A special MMS case: in the address phase with misplaced Start: the master sets the LOST_ARB and
    * slave BUS_ERR. The valid event is LOST_ARB comes from the master.
    */
    if(HPI_IF_CHECK_I2C_FSM_MASTER)
    {
        #if(HPI_IF_I2C_MASTER)
        {
            /* INTR_MASTER_I2C_BUS_ERROR:
            * A misplaced Start or Stop condition occurred on the bus: complete the transaction.
            * The interrupt is cleared in I2C_FSM_EXIT_IDLE.
            */
            if(HPI_IF_CHECK_INTR_MASTER_MASKED(HPI_IF_INTR_MASTER_I2C_BUS_ERROR))
            {
                HPI_IF_mstrStatus |= (uint16) (HPI_IF_I2C_MSTAT_ERR_XFER |
                                                         HPI_IF_I2C_MSTAT_ERR_BUS_ERROR);

                endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
            }

            /* INTR_MASTER_I2C_ARB_LOST:
            * The MultiMaster lost arbitrage during transaction.
            * A Misplaced Start or Stop condition is treated as lost arbitration when the master drives the SDA.
            * The interrupt source is cleared in I2C_FSM_EXIT_IDLE.
            */
            if(HPI_IF_CHECK_INTR_MASTER_MASKED(HPI_IF_INTR_MASTER_I2C_ARB_LOST))
            {
                HPI_IF_mstrStatus |= (uint16) (HPI_IF_I2C_MSTAT_ERR_XFER |
                                                         HPI_IF_I2C_MSTAT_ERR_ARB_LOST);

                endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
            }

            #if(HPI_IF_I2C_MULTI_MASTER_SLAVE)
            {
                /* I2C_MASTER_CMD_M_START_ON_IDLE:
                * MultiMaster-Slave does not generate start, because Slave was addressed.
                * Pass control to slave.
                */
                if(HPI_IF_CHECK_I2C_MASTER_CMD(HPI_IF_I2C_MASTER_CMD_M_START_ON_IDLE))
                {
                    HPI_IF_mstrStatus |= (uint16) (HPI_IF_I2C_MSTAT_ERR_XFER |
                                                             HPI_IF_I2C_MSTAT_ERR_ABORT_XFER);

                    endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
                }
            }
            #endif

            /* The error handling common part:
            * Sets a completion flag of the master transaction and passes control to:
            *  - I2C_FSM_EXIT_IDLE - to complete transaction in case of: ARB_LOST or BUS_ERR.
            *  - I2C_FSM_IDLE      - to take chance for the slave to process incoming transaction.
            */
            if(0u != endTransfer)
            {
                /* Set completion flags for master */
                HPI_IF_mstrStatus |= (uint16) HPI_IF_GET_I2C_MSTAT_CMPLT;

                #if(HPI_IF_I2C_MULTI_MASTER_SLAVE)
                {
                    if(HPI_IF_CHECK_I2C_FSM_ADDR)
                    {
                        /* Start generation is set after another master starts accessing Slave.
                        * Clean-up master and turn to slave. Set state to IDLE.
                        */
                        if(HPI_IF_CHECK_I2C_MASTER_CMD(HPI_IF_I2C_MASTER_CMD_M_START_ON_IDLE))
                        {
                            HPI_IF_I2C_MASTER_CLEAR_START;

                            endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER; /* Pass control to Slave */
                        }
                        /* Valid arbitration lost on the address phase happens only when: master LOST_ARB is set and
                        * slave BUS_ERR is cleared. Only in that case set the state to IDLE without SCB IP re-enable.
                        */
                        else if((!HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_BUS_ERROR))
                               && HPI_IF_CHECK_INTR_MASTER_MASKED(HPI_IF_INTR_MASTER_I2C_ARB_LOST))
                        {
                            endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER; /* Pass control to Slave */
                        }
                        else
                        {
                            endTransfer = 0u; /* Causes I2C_FSM_EXIT_IDLE to be set below */
                        }

                        if(0u != endTransfer) /* Clean-up master to proceed with slave */
                        {
                            HPI_IF_CLEAR_TX_FIFO; /* Shifter keeps address, clear it */

                            HPI_IF_DISABLE_MASTER_AUTO_DATA_ACK; /* In case of reading disable autoACK */

                            /* Clean-up master interrupt sources */
                            HPI_IF_ClearMasterInterruptSource(HPI_IF_INTR_MASTER_ALL);

                            /* Disable data processing interrupts: they have to be cleared before */
                            HPI_IF_SetRxInterruptMode(HPI_IF_NO_INTR_SOURCES);
                            HPI_IF_SetTxInterruptMode(HPI_IF_NO_INTR_SOURCES);

                            HPI_IF_state = HPI_IF_I2C_FSM_IDLE;
                        }
                        else
                        {
                            /* Set I2C_FSM_EXIT_IDLE for BUS_ERR and ARB_LOST (that is really bus error) */
                            HPI_IF_state = HPI_IF_I2C_FSM_EXIT_IDLE;
                        }
                    }
                    else
                    {
                        /* Set I2C_FSM_EXIT_IDLE if any other state than address */
                        HPI_IF_state = HPI_IF_I2C_FSM_EXIT_IDLE;
                    }
                }
                #else
                {
                    /* In case of LOST*/
                    HPI_IF_state = HPI_IF_I2C_FSM_EXIT_IDLE;
                }
                #endif
            }
        }
        #endif
    }
    else /* (HPI_IF_CHECK_I2C_FSM_SLAVE) */
    {
        #if(HPI_IF_I2C_SLAVE)
        {
            /* INTR_SLAVE_I2C_BUS_ERROR or HPI_IF_INTR_SLAVE_I2C_ARB_LOST:
            * A Misplaced Start or Stop condition occurred on the bus: set a flag
            * to notify an error condition.
            */
            if(HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_BUS_ERROR |
                                                        HPI_IF_INTR_SLAVE_I2C_ARB_LOST))
            {
                if(HPI_IF_CHECK_I2C_FSM_RD)
                {
                    /* TX direction: master reads from slave */
                    HPI_IF_slStatus &= (uint8) ~HPI_IF_I2C_SSTAT_RD_BUSY;
                    HPI_IF_slStatus |= (uint8) (HPI_IF_I2C_SSTAT_RD_ERR |
                                                          HPI_IF_I2C_SSTAT_RD_CMPLT);
                }
                else
                {
                    /* RX direction: master writes into slave */
                    HPI_IF_slStatus &= (uint8) ~HPI_IF_I2C_SSTAT_WR_BUSY;
                    HPI_IF_slStatus |= (uint8) (HPI_IF_I2C_SSTAT_WR_ERR |
                                                          HPI_IF_I2C_SSTAT_WR_CMPLT);
                }

                HPI_IF_state = HPI_IF_I2C_FSM_EXIT_IDLE;
            }
        }
        #endif
    }

    /* States description:
    * Any Master operation starts from: the ADDR_RD/WR state as the master generates traffic on the bus.
    * Any Slave operation starts from: the IDLE state as the slave always waits for actions from the master.
    */

    /* FSM Master */
    if(HPI_IF_CHECK_I2C_FSM_MASTER)
    {
        #if(HPI_IF_I2C_MASTER)
        {
            /* INTR_MASTER_I2C_STOP:
            * A Stop condition was generated by the master: the end of the transaction.
            * Set completion flags to notify the API.
            */
            if(HPI_IF_CHECK_INTR_MASTER_MASKED(HPI_IF_INTR_MASTER_I2C_STOP))
            {
                HPI_IF_ClearMasterInterruptSource(HPI_IF_INTR_MASTER_I2C_STOP);

                HPI_IF_mstrStatus |= (uint16) HPI_IF_GET_I2C_MSTAT_CMPLT;
                HPI_IF_state       = HPI_IF_I2C_FSM_IDLE;
            }
            else
            {
                if(HPI_IF_CHECK_I2C_FSM_ADDR) /* Address stage */
                {
                    /* INTR_MASTER_I2C_NACK:
                    * The master sent an address but it was NACKed by the slave. Complete transaction.
                    */
                    if(HPI_IF_CHECK_INTR_MASTER_MASKED(HPI_IF_INTR_MASTER_I2C_NACK))
                    {
                        HPI_IF_ClearMasterInterruptSource(HPI_IF_INTR_MASTER_I2C_NACK);

                        HPI_IF_mstrStatus |= (uint16) (HPI_IF_I2C_MSTAT_ERR_XFER |
                                                                 HPI_IF_I2C_MSTAT_ERR_ADDR_NAK);

                        endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
                    }
                    /* INTR_TX_UNDERFLOW. The master sent an address:
                    *  - TX direction: the clock is stretched after the ACK phase, because the TX FIFO is
                    *    EMPTY. The TX EMPTY cleans all the TX interrupt sources.
                    *  - RX direction: the 1st byte is received, but there is no ACK permission,
                    *    the clock is stretched after 1 byte is received.
                    */
                    else
                    {
                        if(HPI_IF_CHECK_I2C_FSM_RD) /* Reading */
                        {
                            HPI_IF_state = HPI_IF_I2C_FSM_MSTR_RD_DATA;
                        }
                        else /* Writing */
                        {
                            HPI_IF_state = HPI_IF_I2C_FSM_MSTR_WR_DATA;
                            if(0u != HPI_IF_mstrWrBufSize)
                            {
                                /* Enable INTR.TX_EMPTY if there is data to transmit */
                                HPI_IF_SetTxInterruptMode(HPI_IF_INTR_TX_EMPTY);
                            }
                        }
                    }
                }

                if(HPI_IF_CHECK_I2C_FSM_DATA) /* Data phase */
                {
                    if(HPI_IF_CHECK_I2C_FSM_RD) /* Reading */
                    {
                        /* INTR_RX_FULL:
                        * RX direction: the master received 8 bytes.
                        * Get data from RX FIFO and decide whether to ACK or  NACK the following bytes.
                        */
                        if(HPI_IF_CHECK_INTR_RX_MASKED(HPI_IF_INTR_RX_FULL))
                        {
                            /* Calculate difference */
                            diffCount =  HPI_IF_mstrRdBufSize -
                                        (HPI_IF_mstrRdBufIndex + HPI_IF_GET_RX_FIFO_ENTRIES);

                            /* Proceed transaction or end it when RX FIFO becomes FULL again */
                            if(diffCount > HPI_IF_I2C_FIFO_SIZE)
                            {
                                diffCount = HPI_IF_I2C_FIFO_SIZE;
                            }
                            else
                            {
                                if(0u == diffCount)
                                {
                                    HPI_IF_DISABLE_MASTER_AUTO_DATA_ACK;

                                    diffCount   = HPI_IF_I2C_FIFO_SIZE;
                                    endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
                                }
                            }

                            for(; (0u != diffCount); diffCount--)
                            {
                                HPI_IF_mstrRdBufPtr[HPI_IF_mstrRdBufIndex] = (uint8)
                                                                                        HPI_IF_RX_FIFO_RD_REG;
                                HPI_IF_mstrRdBufIndex++;
                            }
                        }
                        /* INTR_RX_NOT_EMPTY:
                        * RX direction: the master received one data byte, ACK or NACK it.
                        * The last byte is stored and NACKed by the master. The NACK and Stop is
                        * generated by one command generate Stop.
                        */
                        else if(HPI_IF_CHECK_INTR_RX_MASKED(HPI_IF_INTR_RX_NOT_EMPTY))
                        {
                            /* Put data in component buffer */
                            HPI_IF_mstrRdBufPtr[HPI_IF_mstrRdBufIndex] = (uint8) HPI_IF_RX_FIFO_RD_REG;
                            HPI_IF_mstrRdBufIndex++;

                            if(HPI_IF_mstrRdBufIndex < HPI_IF_mstrRdBufSize)
                            {
                                HPI_IF_I2C_MASTER_GENERATE_ACK;
                            }
                            else
                            {
                               endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
                            }
                        }
                        else
                        {
                            /* Do nothing */
                        }

                        HPI_IF_ClearRxInterruptSource(HPI_IF_INTR_RX_ALL);
                    }
                    else /* Writing */
                    {
                        /* INTR_MASTER_I2C_NACK :
                        * The master writes data to the slave and NACK was received: not all the bytes were
                        * written to the slave from the TX FIFO. Revert the index if there is data in
                        * the TX FIFO and pass control to a complete transfer.
                        */
                        if(HPI_IF_CHECK_INTR_MASTER_MASKED(HPI_IF_INTR_MASTER_I2C_NACK))
                        {
                            HPI_IF_ClearMasterInterruptSource(HPI_IF_INTR_MASTER_I2C_NACK);

                            /* Rollback write buffer index: NACKed byte remains in shifter */
                            HPI_IF_mstrWrBufIndexTmp -= (HPI_IF_GET_TX_FIFO_ENTRIES +
                                                                   HPI_IF_GET_TX_FIFO_SR_VALID);

                            /* Update number of transferred bytes */
                            HPI_IF_mstrWrBufIndex = HPI_IF_mstrWrBufIndexTmp;

                            HPI_IF_mstrStatus |= (uint16) (HPI_IF_I2C_MSTAT_ERR_XFER |
                                                                     HPI_IF_I2C_MSTAT_ERR_SHORT_XFER);

                            HPI_IF_CLEAR_TX_FIFO;

                            endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
                        }
                        /* INTR_TX_EMPTY :
                        * TX direction: the TX FIFO is EMPTY, the data from the buffer needs to be put there.
                        * When there is no data in the component buffer, the underflow interrupt is
                        * enabled to catch when all the data has been transferred.
                        */
                        else if(HPI_IF_CHECK_INTR_TX_MASKED(HPI_IF_INTR_TX_EMPTY))
                        {
                            while(HPI_IF_I2C_FIFO_SIZE != HPI_IF_GET_TX_FIFO_ENTRIES)
                            {
                                /* The temporary mstrWrBufIndexTmp is used because slave could NACK the byte and index
                                * roll-back required in this case. The mstrWrBufIndex is updated at the end of transfer.
                                */
                                if(HPI_IF_mstrWrBufIndexTmp < HPI_IF_mstrWrBufSize)
                                {
                                #if(!HPI_IF_CY_SCBIP_V0)
                                   /* Clear INTR_TX.UNDERFLOW before putting the last byte into TX FIFO. This ensures
                                    * a proper trigger at the end of transaction when INTR_TX.UNDERFLOW single trigger
                                    * event. Ticket ID# 156735.
                                    */
                                    if(HPI_IF_mstrWrBufIndexTmp == (HPI_IF_mstrWrBufSize - 1u))
                                    {
                                        HPI_IF_ClearTxInterruptSource(HPI_IF_INTR_TX_UNDERFLOW);
                                        HPI_IF_SetTxInterruptMode(HPI_IF_INTR_TX_UNDERFLOW);
                                    }
                                 #endif /* (!HPI_IF_CY_SCBIP_V0) */

                                    /* Put data into TX FIFO */
                                    HPI_IF_TX_FIFO_WR_REG = (uint32) HPI_IF_mstrWrBufPtr[HPI_IF_mstrWrBufIndexTmp];
                                    HPI_IF_mstrWrBufIndexTmp++;
                                }
                                else
                                {
                                    break; /* No more data to put */
                                }
                            }

                        #if(HPI_IF_CY_SCBIP_V0)
                            if(HPI_IF_mstrWrBufIndexTmp == HPI_IF_mstrWrBufSize)
                            {
                                HPI_IF_SetTxInterruptMode(HPI_IF_INTR_TX_UNDERFLOW);
                            }

                            HPI_IF_ClearTxInterruptSource(HPI_IF_INTR_TX_ALL);
                        #else
                            HPI_IF_ClearTxInterruptSource(HPI_IF_INTR_TX_EMPTY);
                        #endif /* (HPI_IF_CY_SCBIP_V0) */
                        }
                        /* INTR_TX_UNDERFLOW:
                        * TX direction: all data from the TX FIFO was transferred to the slave.
                        * The transaction needs to be completed.
                        */
                        else if(HPI_IF_CHECK_INTR_TX_MASKED(HPI_IF_INTR_TX_UNDERFLOW))
                        {
                            /* Update number of transferred bytes */
                            HPI_IF_mstrWrBufIndex = HPI_IF_mstrWrBufIndexTmp;

                            endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
                        }
                        else
                        {
                            /* Do nothing */
                        }
                    }
                }

                if(0u != endTransfer) /* Complete transfer */
                {
                    /* Clean-up master after reading: only in case of NACK */
                    HPI_IF_DISABLE_MASTER_AUTO_DATA_ACK;

                    /* Disable data processing interrupts: they have to be cleared before */
                    HPI_IF_SetRxInterruptMode(HPI_IF_NO_INTR_SOURCES);
                    HPI_IF_SetTxInterruptMode(HPI_IF_NO_INTR_SOURCES);

                    if(HPI_IF_CHECK_I2C_MODE_NO_STOP(HPI_IF_mstrControl))
                    {
                        /* On-going transaction is suspended: the ReStart is generated by the API request */
                        HPI_IF_mstrStatus |= (uint16) (HPI_IF_I2C_MSTAT_XFER_HALT |
                                                                 HPI_IF_GET_I2C_MSTAT_CMPLT);

                        HPI_IF_state = HPI_IF_I2C_FSM_MSTR_HALT;
                    }
                    else
                    {
                        /* Complete transaction: exclude the data processing state and generate Stop.
                        * The completion status will be set after Stop generation.
                        * A special case is read: because NACK and Stop are generated by the command below.
                        * Lost arbitration can occur during NACK generation when
                        * the other master is still reading from the slave.
                        */
                        HPI_IF_I2C_MASTER_GENERATE_STOP;
                    }
                }
            }

        } /* (HPI_IF_I2C_MASTER) */
        #endif

    } /* (HPI_IF_CHECK_I2C_FSM_MASTER) */


    /* FSM Slave */
    else if(HPI_IF_CHECK_I2C_FSM_SLAVE)
    {
        #if(HPI_IF_I2C_SLAVE)
        {
            /* INTR_SLAVE_NACK:
            * The master completes reading the slave: the appropriate flags have to be set.
            * The TX FIFO is cleared after an overflow condition is set.
            */
            if(HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_NACK))
            {
                HPI_IF_ClearSlaveInterruptSource(HPI_IF_INTR_SLAVE_I2C_NACK);

                /* All entries that remain in TX FIFO are: FIFO Size + 1 (SHIFTER) */
                diffCount = (HPI_IF_GET_TX_FIFO_ENTRIES + HPI_IF_GET_TX_FIFO_SR_VALID);

                if(HPI_IF_slOverFlowCount > diffCount) /* Overflow */
                {
                    HPI_IF_slStatus |= (uint8) HPI_IF_I2C_SSTAT_RD_OVFL;
                }
                else /* No Overflow */
                {
                    /* Roll-back temporary index */
                    HPI_IF_slRdBufIndexTmp -= (diffCount - HPI_IF_slOverFlowCount);
                }

                /* Update slave of transferred bytes */
                HPI_IF_slRdBufIndex = HPI_IF_slRdBufIndexTmp;

                /* Clean-up TX FIFO */
                HPI_IF_SetTxInterruptMode(HPI_IF_NO_INTR_SOURCES);
                HPI_IF_slOverFlowCount = 0u;
                HPI_IF_CLEAR_TX_FIFO;

                /* Complete master reading */
                HPI_IF_slStatus &= (uint8) ~HPI_IF_I2C_SSTAT_RD_BUSY;
                HPI_IF_slStatus |= (uint8)  HPI_IF_I2C_SSTAT_RD_CMPLT;
                HPI_IF_state     =  HPI_IF_I2C_FSM_IDLE;
                
            #ifdef HPI_IF_I2C_SLAVE_CMPLT_CALLBACK
                /* Read complete */
                HPI_IF_I2C_SlaveCompleteCallback();
            #endif /* HPI_IF_I2C_SLAVE_CMPLT_CALLBACK */
            }


            /* INTR_SLAVE_I2C_WRITE_STOP:
            * The master completes writing to the slave: the appropriate flags have to be set.
            * The RX FIFO contains 1-8 bytes from the previous transaction which needs to be read.
            * There is a possibility that RX FIFO contains an address, it needs to leave it there.
            */
            if(HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_WRITE_STOP))
            {
                HPI_IF_ClearSlaveInterruptSource(HPI_IF_INTR_SLAVE_I2C_WRITE_STOP);

                /* Read bytes from RX FIFO when auto data ACK receive logic is enabled. Otherwise all data bytes
                * were already read from the RX FIFO except for address byte which has to stay here to be handled by
                * I2C_ADDR_MATCH.
                */
                if (0u != (HPI_IF_I2C_CTRL_REG & HPI_IF_I2C_CTRL_S_READY_DATA_ACK))
                {
                    while(0u != HPI_IF_GET_RX_FIFO_ENTRIES)
                    {
                        #if(HPI_IF_CHECK_I2C_ACCEPT_ADDRESS)
                        {
                            if((1u == HPI_IF_GET_RX_FIFO_ENTRIES) &&
                               (HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_ADDR_MATCH)))
                            {
                                break; /* Leave address in RX FIFO */
                            }
                        }
                        #endif

                        /* Put data in component buffer */
                        HPI_IF_slWrBufPtr[HPI_IF_slWrBufIndex] = (uint8) HPI_IF_RX_FIFO_RD_REG;
                        HPI_IF_slWrBufIndex++;
                    }

                    HPI_IF_DISABLE_SLAVE_AUTO_DATA;
                }

                if(HPI_IF_CHECK_INTR_RX(HPI_IF_INTR_RX_OVERFLOW))
                {
                    HPI_IF_slStatus |= (uint8) HPI_IF_I2C_SSTAT_WR_OVFL;
                }

                /* Clears RX interrupt sources triggered on data receiving */
                HPI_IF_SetRxInterruptMode(HPI_IF_NO_INTR_SOURCES);
                HPI_IF_ClearRxInterruptSource(HPI_IF_INTR_RX_ALL);

                /* Complete master writing */
                HPI_IF_slStatus &= (uint8) ~HPI_IF_I2C_SSTAT_WR_BUSY;
                HPI_IF_slStatus |= (uint8)  HPI_IF_I2C_SSTAT_WR_CMPLT;
                HPI_IF_state     =  HPI_IF_I2C_FSM_IDLE;

            #ifdef HPI_IF_I2C_SLAVE_CMPLT_CALLBACK
                /* Write complete */
                HPI_IF_I2C_SlaveCompleteCallback();
            #endif /* HPI_IF_I2C_SLAVE_CMPLT_CALLBACK */
            }


            /* INTR_SLAVE_I2C_ADDR_MATCH or INTR_SLAVE_I2C_GENERAL:
            * The address match or general call address event starts the slave operation:
            * after leaving the TX or RX direction has to be chosen.
            * The wakeup interrupt must be cleared only after an address match is set.
            */
        #if (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST)
            if (HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_ADDR_MATCH |
                                                         HPI_IF_INTR_SLAVE_I2C_GENERAL))
        #else
            if (HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_ADDR_MATCH))
        #endif /* (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST) */
            {
                /* Clear externally clocked address match interrupt source when internally clocked is set */
                HPI_IF_ClearI2CExtClkInterruptSource(HPI_IF_INTR_I2C_EC_WAKE_UP);

                #if (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER)
                {
                    if (NULL != HPI_IF_customAddressHandler)
                    {
                        /* Call custom address handler */
                        response = HPI_IF_customAddressHandler();
                    }
                    else
                    {
                        /* Read address from the RX FIFO. If there is no address underflow triggers but
                        * component does not use that source. */
                        (void) HPI_IF_RX_FIFO_RD_REG;
                        response = HPI_IF_I2C_ACK_ADDR;
                    }

                    /* Clears RX sources after address was received in the RX FIFO */
                    HPI_IF_ClearRxInterruptSource(HPI_IF_INTR_RX_ALL);
                }
                #endif

            #if (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST)
                if (response == HPI_IF_I2C_NAK_ADDR)
                {
                #if (!HPI_IF_CY_SCBIP_V0)
                    /* Disable write stop interrupt source as it triggers after address was NACKed. Ticket ID#156094 */
                    HPI_IF_DISABLE_INTR_SLAVE(HPI_IF_INTR_SLAVE_I2C_WRITE_STOP);
                #endif /* (!HPI_IF_CY_SCBIP_V0) */

                    /* Clear address match and stop history */
                    HPI_IF_ClearSlaveInterruptSource(HPI_IF_INTR_SLAVE_ALL);

                    /* ACK the address byte */
                    HPI_IF_I2C_SLAVE_GENERATE_NACK;
                }
                else
            #endif /* (HPI_IF_I2C_CUSTOM_ADDRESS_HANDLER_CONST) */
                {
                    if(HPI_IF_CHECK_I2C_STATUS(HPI_IF_I2C_STATUS_S_READ))
                    /* TX direction: master reads from slave */
                    {
                        HPI_IF_SetTxInterruptMode(HPI_IF_INTR_TX_EMPTY);

                        /* Set temporary index to address buffer clear from API */
                        HPI_IF_slRdBufIndexTmp = HPI_IF_slRdBufIndex;

                        /* Start master reading */
                        HPI_IF_slStatus |= (uint8) HPI_IF_I2C_SSTAT_RD_BUSY;
                        HPI_IF_state     = HPI_IF_I2C_FSM_SL_RD;
                    }
                    else
                    /* RX direction: master writes into slave */
                    {
                        /* Calculate available buffer size */
                        diffCount = (HPI_IF_slWrBufSize - HPI_IF_slWrBufIndex);

                    #if (HPI_IF_CY_SCBIP_V0)
                        if(diffCount < HPI_IF_I2C_FIFO_SIZE)
                        /* Receive data: byte-by-byte */
                        {
                            HPI_IF_SetRxInterruptMode(HPI_IF_INTR_RX_NOT_EMPTY);
                        }
                        else
                        /* Receive data: into RX FIFO */
                        {
                            if(diffCount == HPI_IF_I2C_FIFO_SIZE)
                            {
                                /* NACK when RX FIFO become FULL */
                                HPI_IF_ENABLE_SLAVE_AUTO_DATA;
                            }
                            else
                            {
                                /* Stretch clock when RX FIFO becomes FULL */
                                HPI_IF_ENABLE_SLAVE_AUTO_DATA_ACK;
                                HPI_IF_SetRxInterruptMode(HPI_IF_INTR_RX_FULL);
                            }
                        }

                    #else
                        #if(HPI_IF_CHECK_I2C_ACCEPT_ADDRESS)
                        {
                            /* Enable RX.NOT_EMPTY interrupt source to receive byte by byte.
                            * The byte by byte receive is always chosen for the case when an address is accepted
                            * in RX FIFO. Ticket ID#175559.
                            */
                            HPI_IF_SetRxInterruptMode(HPI_IF_INTR_RX_NOT_EMPTY);
                        }
                        #else
                        {
                            if(diffCount < HPI_IF_I2C_FIFO_SIZE)
                            /* Receive data: byte-by-byte */
                            {
                                HPI_IF_SetRxInterruptMode(HPI_IF_INTR_RX_NOT_EMPTY);
                            }
                            else
                            /* Receive data: into RX FIFO */
                            {
                                if(diffCount == HPI_IF_I2C_FIFO_SIZE)
                                {
                                    /* NACK when RX FIFO become FULL */
                                    HPI_IF_ENABLE_SLAVE_AUTO_DATA;
                                }
                                else
                                {
                                    /* Stretch clock when RX FIFO becomes FULL */
                                    HPI_IF_ENABLE_SLAVE_AUTO_DATA_ACK;
                                    HPI_IF_SetRxInterruptMode(HPI_IF_INTR_RX_FULL);
                                }
                            }
                        }
                        #endif
                    #endif /* (HPI_IF_CY_SCBIP_V0) */

                        /* Start master reading */
                        HPI_IF_slStatus |= (uint8) HPI_IF_I2C_SSTAT_WR_BUSY;
                        HPI_IF_state     = HPI_IF_I2C_FSM_SL_WR;
                    }

                    /* Clear address match and stop history */
                    HPI_IF_ClearSlaveInterruptSource(HPI_IF_INTR_SLAVE_ALL);

                #if (!HPI_IF_CY_SCBIP_V0)
                    /* Enable write stop interrupt source as it triggers after address was NACKed. Ticket ID#156094 */
                    HPI_IF_ENABLE_INTR_SLAVE(HPI_IF_INTR_SLAVE_I2C_WRITE_STOP);
                #endif /* (!HPI_IF_CY_SCBIP_V0) */

                    /* ACK the address byte */
                    HPI_IF_I2C_SLAVE_GENERATE_ACK;
                }
            }

            /* HPI_IF_INTR_RX_FULL:
            * Get data from the RX FIFO and decide whether to ACK or NACK the following bytes
            */
            if(HPI_IF_CHECK_INTR_RX_MASKED(HPI_IF_INTR_RX_FULL))
            {
                /* Calculate available buffer size to take into account that RX FIFO is FULL */
                diffCount =  HPI_IF_slWrBufSize -
                            (HPI_IF_slWrBufIndex + HPI_IF_I2C_FIFO_SIZE);

                if(diffCount > HPI_IF_I2C_FIFO_SIZE) /* Proceed transaction */
                {
                    diffCount   = HPI_IF_I2C_FIFO_SIZE;
                    endTransfer = 0u;  /* Continue active transfer */
                }
                else /* End when FIFO becomes FULL again */
                {
                    endTransfer = HPI_IF_I2C_CMPLT_ANY_TRANSFER;
                }

                for(; (0u != diffCount); diffCount--)
                {
                    /* Put data in component buffer */
                    HPI_IF_slWrBufPtr[HPI_IF_slWrBufIndex] = (uint8) HPI_IF_RX_FIFO_RD_REG;
                    HPI_IF_slWrBufIndex++;
                }

                if(0u != endTransfer) /* End transfer sending NACK */
                {
                    HPI_IF_ENABLE_SLAVE_AUTO_DATA_NACK;

                    /* INTR_RX_FULL triggers earlier than INTR_SLAVE_I2C_STOP:
                    * disable all RX interrupt sources.
                    */
                    HPI_IF_SetRxInterruptMode(HPI_IF_NO_INTR_SOURCES);
                }

                HPI_IF_ClearRxInterruptSource(HPI_IF_INTR_RX_FULL);
            }
            /* HPI_IF_INTR_RX_NOT_EMPTY:
            * The buffer size is less than 8: it requires processing in byte-by-byte mode.
            */
            else if(HPI_IF_CHECK_INTR_RX_MASKED(HPI_IF_INTR_RX_NOT_EMPTY))
            {
                diffCount = HPI_IF_RX_FIFO_RD_REG;

                if(HPI_IF_slWrBufIndex < HPI_IF_slWrBufSize)
                {
                    HPI_IF_I2C_SLAVE_GENERATE_ACK;

                    /* Put data into component buffer */
                    HPI_IF_slWrBufPtr[HPI_IF_slWrBufIndex] = (uint8) diffCount;
                    HPI_IF_slWrBufIndex++;
                }
                else /* Overflow: there is no space in write buffer */
                {
                    HPI_IF_I2C_SLAVE_GENERATE_NACK;

                    HPI_IF_slStatus |= (uint8) HPI_IF_I2C_SSTAT_WR_OVFL;
                }

                HPI_IF_ClearRxInterruptSource(HPI_IF_INTR_RX_NOT_EMPTY);
            }
            else
            {
                /* Does nothing */
            }


            /* HPI_IF_INTR_TX_EMPTY:
            * The master reads the slave: provide data to read or 0xFF in the case of the end of the buffer
            * The overflow condition must be captured, but not set until the end of transaction.
            * There is a possibility of a false overflow due to TX FIFO utilization.
            */
            if(HPI_IF_CHECK_INTR_TX_MASKED(HPI_IF_INTR_TX_EMPTY))
            {
                while(HPI_IF_I2C_FIFO_SIZE != HPI_IF_GET_TX_FIFO_ENTRIES)
                {
                    /* Temporary slRdBufIndexTmp is used because the master can NACK the byte and
                    * index roll-back is required in this case. The slRdBufIndex is updated at the end
                    * of the read transfer.
                    */
                    if(HPI_IF_slRdBufIndexTmp < HPI_IF_slRdBufSize)
                    /* Data from buffer */
                    {
                        HPI_IF_TX_FIFO_WR_REG = (uint32) HPI_IF_slRdBufPtr[HPI_IF_slRdBufIndexTmp];
                        HPI_IF_slRdBufIndexTmp++;
                    }
                    else
                    /* Probably Overflow */
                    {
                        HPI_IF_TX_FIFO_WR_REG = HPI_IF_I2C_SLAVE_OVFL_RETURN;

                        if(HPI_IF_slOverFlowCount <= HPI_IF_I2C_TX_OVERFLOW_COUNT)
                        {
                            /* Get counter in range of overflow. */
                            HPI_IF_slOverFlowCount++;
                        }
                    }
                }

                HPI_IF_ClearTxInterruptSource(HPI_IF_INTR_TX_EMPTY);
            }

        }  /* (HPI_IF_I2C_SLAVE) */
        #endif
    }


    /* FSM EXIT:
    * Slave:  INTR_SLAVE_I2C_BUS_ERROR, INTR_SLAVE_I2C_ARB_LOST
    * Master: INTR_MASTER_I2C_BUS_ERROR, INTR_MASTER_I2C_ARB_LOST.
    */
    else
    {
        HPI_IF_I2CFwBlockReset();
        
    #ifdef HPI_IF_I2C_SLAVE_CMPLT_CALLBACK
        #if(HPI_IF_I2C_SLAVE)
        {
            /* Error condition: read or write complete is set */
            HPI_IF_I2C_SlaveCompleteCallback();
        }
        #endif
    #endif /* HPI_IF_I2C_SLAVE_CMPLT_CALLBACK */
    }

#ifdef HPI_IF_I2C_ISR_EXIT_CALLBACK
    HPI_IF_I2C_ISR_ExitCallback();
#endif /* HPI_IF_I2C_ISR_EXIT_CALLBACK */

}


/* [] END OF FILE */
