/***************************************************************************//**
* \file HPI_IF_I2C_BOOT.c
* \version 4.0
*
* \brief
*  This file provides the source code of the bootloader communication APIs
*  for the SCB Component I2C mode.
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

#include "HPI_IF_BOOT.h"
#include "HPI_IF_I2C_PVT.h"

#if defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_I2C_BTLDR_COMM_ENABLED)

/***************************************
*    Private I/O Component Vars
***************************************/

/* Writes to this buffer */
static uint8 HPI_IF_slReadBuf[HPI_IF_I2C_BTLDR_SIZEOF_READ_BUFFER];

/* Reads from this buffer */
static uint8 HPI_IF_slWriteBuf[HPI_IF_I2C_BTLDR_SIZEOF_WRITE_BUFFER];

/* Flag to release buffer to be read */
static uint32 HPI_IF_applyBuffer;


/***************************************
*    Private Function Prototypes
***************************************/

static void HPI_IF_I2CResposeInsert(void);


/*******************************************************************************
* Function Name: HPI_IF_I2CCyBtldrCommStart
****************************************************************************//**
*
*  Starts the I2C component and enables its interrupt.
*  Every incoming I2C write transaction is treated as a command for the
*  bootloader.
*  Every incoming I2C read transaction returns 0xFF until the bootloader
*  provides a response to the executed command.
*
* \globalvars
*  HPI_IF_applyBuffer - the flag to release the buffer with a response
*  to be read by the host.
*
*******************************************************************************/
void HPI_IF_I2CCyBtldrCommStart(void)
{
    HPI_IF_I2CSlaveInitWriteBuf(HPI_IF_slWriteBuf, HPI_IF_I2C_BTLDR_SIZEOF_WRITE_BUFFER);
    HPI_IF_I2CSlaveInitReadBuf (HPI_IF_slReadBuf, 0u);

    HPI_IF_SetCustomInterruptHandler(&HPI_IF_I2CResposeInsert);
    HPI_IF_applyBuffer = 0u;

    HPI_IF_Start();
}


/*******************************************************************************
* Function Name: HPI_IF_I2CCyBtldrCommStop
****************************************************************************//**
*
*  Disables the I2C component.
*
*******************************************************************************/
void HPI_IF_I2CCyBtldrCommStop(void)
{
    HPI_IF_Stop();
}


/*******************************************************************************
* Function Name: HPI_IF_I2CCyBtldrCommReset
****************************************************************************//**
*
*  Resets read and write I2C buffers to the initial state and resets the slave
*  status.
*
* \globalvars
*  HPI_IF_slRdBufSize - used to store slave read buffer size.
*  HPI_IF_slRdBufIndex - used to store the current index within the
*  slave read buffer.
*  HPI_IF_slWrBufIndex - used to store current index within slave
*  write buffer.
*  HPI_IF_slStatus  - used to store current status of I2C slave.
*
*******************************************************************************/
void HPI_IF_I2CCyBtldrCommReset(void)
{
    /* Make read buffer full */
    HPI_IF_slRdBufSize = 0u;

    /* Reset write buffer and Read buffer */
    HPI_IF_slRdBufIndex = 0u;
    HPI_IF_slWrBufIndex = 0u;

    /* Clear read and write status */
    HPI_IF_slStatus = 0u;
}


/*******************************************************************************
* Function Name: HPI_IF_I2CCyBtldrCommRead
****************************************************************************//**
*
*  Allows the caller to read data from the bootloader host (the host writes the
*  data). The function handles polling to allow a block of data to be completely
*  received from the host device.
*
*  \param pData: Pointer to storage for the block of data to be read from the
*   bootloader host
*  \param size: Number of bytes to be read.
*  \param count: Pointer to the variable to write the number of bytes actually
*   read.
*  \param timeOut: Number of units in 10 ms to wait before returning
*   because of a timeout.
*
*  \return
*   Returns CYRET_SUCCESS if no problem was encountered or returns the value
*   that best describes the problem. For more information refer to the
*   "Return Codes" section of the System Reference Guide.
*
* \globalvars
*  HPI_IF_slStatus  - used to store current status of I2C slave.
*  HPI_IF_slWriteBuf - used to store received command.
*  HPI_IF_slWrBufIndex - used to store current index within slave
*  write buffer.
*
*******************************************************************************/
cystatus HPI_IF_I2CCyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;
    uint32 timeoutMs;

    status = CYRET_BAD_PARAM;

    if ((NULL != pData) && (size > 0u))
    {
        status = CYRET_TIMEOUT;
        timeoutMs = ((uint32) 10u * timeOut); /* Convert from 10mS check to 1mS checks */

        while (0u != timeoutMs)
        {
            /* Check if host complete write */
            if (0u != (HPI_IF_I2C_SSTAT_WR_CMPLT & HPI_IF_slStatus))
            {
                /* Copy number of written bytes */
                *count = (uint16) HPI_IF_slWrBufIndex;

                /* Clear slave status and write index */
                HPI_IF_slStatus = 0u;
                HPI_IF_slWrBufIndex = 0u;

                /* Copy command into bootloader buffer */
                (void) memcpy((void *) pData, (const void *) HPI_IF_slWriteBuf,
                                              HPI_IF_BYTES_TO_COPY(*count, size));

                status = CYRET_SUCCESS;
                break;
            }

            CyDelay(HPI_IF_WAIT_1_MS);
            --timeoutMs;
        }
    }

    return(status);
}


/*******************************************************************************
* Function Name: HPI_IF_I2CCyBtldrCommWrite
****************************************************************************//**
*
*  Allows the caller to write data to the bootloader host (the host reads the
*  data). The function does not use timeout and returns after data has been
*  copied into the slave read buffer. This data is available to be read by the
*  bootloader host until following host data write.
*
*  \param pData: Pointer to the block of data to be written to the bootloader
*   host.
*  \param size: Number of bytes to be written.
*  \param count: Pointer to the variable to write the number of bytes actually
*   written.
*  \param timeOut: Number of units in 10 ms to wait before returning
*   because of a timeout.
*
*  \return
*   Returns CYRET_SUCCESS if no problem was encountered or returns the value
*   that best describes the problem. For more information refer to the
*   "Return Codes" section of the System Reference Guide.
*
* \globalvars
*  HPI_IF_applyBuffer - the flag to release the buffer with
*  to be read by the host.
*
*******************************************************************************/
cystatus HPI_IF_I2CCyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)
{
    cystatus status;

    status = CYRET_BAD_PARAM;

    if ((NULL != pData) && (size > 0u))
    {
        /* Copy response into read buffer */
        *count = size;
        (void) memcpy((void *) HPI_IF_slReadBuf, (const void *) pData, (uint32) size);

        /* Read buffer is ready to be released to host */
        HPI_IF_applyBuffer = (uint32) size;

        if (0u != timeOut)
        {
            /* Suppress compiler warning */
        }

        status = CYRET_SUCCESS;
    }

    return(status);
}


/*******************************************************************************
* Function Name: HPI_IF_I2CResposeInsert
****************************************************************************//**
*
*  Releases the read buffer to be read when a response is copied to the buffer
*  and a new read transaction starts.
*  Closes the read buffer when write transaction is started.
*
* \globalvars
*  HPI_IF_applyBuffer - the flag to release the buffer with a response
*  to be read by the host.
*
*******************************************************************************/
static void HPI_IF_I2CResposeInsert(void)
{
    if (HPI_IF_CHECK_INTR_SLAVE_MASKED(HPI_IF_INTR_SLAVE_I2C_ADDR_MATCH))
    {
        if (HPI_IF_CHECK_I2C_STATUS(HPI_IF_I2C_STATUS_S_READ))
        {
            /* Address phase, host reads: release read buffer */
            if (0u != HPI_IF_applyBuffer)
            {
                HPI_IF_slRdBufSize  = HPI_IF_applyBuffer;
                HPI_IF_slRdBufIndex = 0u;
                HPI_IF_applyBuffer  = 0u;
            }
        }
        else
        {
            /* Address phase, host writes: close read buffer */
            if (HPI_IF_slRdBufIndex != HPI_IF_slRdBufSize)
            {
                HPI_IF_slRdBufIndex = HPI_IF_slRdBufSize;
            }
        }
    }
}

#endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (HPI_IF_I2C_BTLDR_COMM_ENABLED) */


/* [] END OF FILE */
