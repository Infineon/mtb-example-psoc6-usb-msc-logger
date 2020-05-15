/*****************************************************************************
* File Name: usb_scsi.c
*
* Description:
*  This file provides the source code to implement the SCSI protocol.
*
* Note:
*
******************************************************************************
* Copyright (2020), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/

#include "usb_scsi.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Variable to keep track of when to send a Test Unit Ready
 * fail status to force a read from the host. */
uint8_t forceOS = false;

uint8_t statusFileTimer = 0;

bool mediaEjectedState = false;

/* Variable to keep track of failed commands. */
bool commandFailed = false;

/* Should not exceet 8 characters. */
const unsigned char vendorIDT10[] = "CYPRESS ";

/* Should not exceet 16 characters. */
const unsigned char productID[] = "PSoC Logger";

/* Product Revision */
const unsigned char productRev[] = "9999";

/*******************************************************************************
* Function Name: usb_scsi_serve_timeout()
********************************************************************************
* Summary:
*  Serve the timeout handler for USB SCSI.
*
*******************************************************************************/
void usb_scsi_serve_timeout(void)
{
    if (forceOS == true)
    {
        statusFileTimer++;
        if (statusFileTimer > STATUS_FILE_TIMEOUT)
        {
            statusFileTimer = STATUS_FILE_TIMEOUT;
        }
    }
}

/*******************************************************************************
* Function Name: usb_scsi_prevent_media_removal()
********************************************************************************
* Summary:
*  Handle the Prevent Media Removal scenario.
*
* Parameters:
*  prevent: is prevent enabled or not
*
* Return:
*  Success if prevent is disabled, error if prevent is enabled.
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_prevent_media_removal(uint8_t prevent)
{
    cy_en_usb_dev_status_t status = CY_USB_DEV_DRV_HW_ERROR;

    if(prevent == false)
    {
        /* Send pass if prevent is not enabled. */
        status = CY_USB_DEV_SUCCESS;
    }
    else
    {
        /* Send a fail response for command to force
         * the OS to initiate a Request Sense. */
        status = CY_USB_DEV_BAD_PARAM;
        commandFailed = true;
    }

    return(status);
}

/*******************************************************************************
* Function Name: usb_scsi_test_unit_ready()
********************************************************************************
* Summary:
*  Responds to the periodic Test Unit Ready Command from host. 
*
* Return:
*  Success if no timeout or in ejected state. 
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_test_unit_ready(void)
{
    cy_en_usb_dev_status_t status = CY_USB_DEV_DRV_HW_ERROR;

    if(mediaEjectedState == true)
    {
        /* Send a fail response for Test Unit Ready command
         * since the media is in ejected state. */
        status = CY_USB_DEV_DRV_HW_ERROR;
    }
    else
    {
        if(forceOS == false)
        {
            /* Send pass by default. */
            status = CY_USB_DEV_SUCCESS;
        }
        else
        {
            if(statusFileTimer >= STATUS_FILE_TIMEOUT)
            {
                statusFileTimer = 0;

                /* Send a fail response for Test Unit Ready command to force
                 * the OS to initiate a Request Sense. */
                status = CY_USB_DEV_BAD_PARAM;
            }
            else
            {
                /*Send pass for one more request before failing. */
                status = CY_USB_DEV_SUCCESS;
            }
        }
    }

    return(status);
}

/*******************************************************************************
* Function Name: usb_scsi_request_sense()
********************************************************************************
* Summary:
*  Responds to the SCSI Request Sense command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_request_sense(cy_stc_usb_dev_msc_context_t *context)
{
    memset(context->in_buffer, 0, context->packet_in_size);
    context->in_buffer[0] = SENSE_RESPONSE_CODE;
    context->in_buffer[7] = SENSE_ADDITIONAL_LENGTH;

    if ((forceOS == false) && (commandFailed == false) && (mediaEjectedState == false))
    {
        context->in_buffer[2] = SENSE_KEY_NO_SENSE;
        context->in_buffer[12] = SENSE_ASC_NO_SENSE;
        context->in_buffer[13] = SENSE_ASCQ_NO_SENSE;
    }
    else
    {
        if (commandFailed == true)
        {
            context->in_buffer[2] = SENSE_KEY_ILLEGAL_REQUEST;
            context->in_buffer[12] = SENSE_ASC_INVALID_FIELD_IN_CDB;
            context->in_buffer[13] = SENSE_ASCQ_NO_SENSE;
            commandFailed = false;
        }

        if (forceOS == true)
        {
            context->in_buffer[2] = SENSE_KEY_NOT_READY;
            context->in_buffer[12] = SENSE_ASC_MEDIA_REMOVAL;
            context->in_buffer[13] = SENSE_ASCQ_NO_SENSE;
            forceOS = false;
            statusFileTimer = 0;
        }

        /* If the host has sent a command to eject the drive,
         * send not ready until re-mounted. */
        if(mediaEjectedState == true)
        {
            context->in_buffer[2] = SENSE_KEY_NOT_READY;
            context->in_buffer[12] = SENSE_ASC_MEDIA_REMOVAL;
            context->in_buffer[13] = SENSE_ASCQ_NO_SENSE;
        }
    }
    context->packet_in_size = 18;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_format_unit()
********************************************************************************
* Summary:
*  Function returns fail as this file system does not support formatting.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always fail
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_format_unit(cy_stc_usb_dev_msc_context_t *context)
{
    commandFailed = true;
    context->packet_in_size = 0;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_DRV_HW_ERROR;
}

/*******************************************************************************
* Function Name: usb_scsi_inquiry()
********************************************************************************
* Summary:
*  Sends the details about the Mass Storage device to host.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_inquiry(cy_stc_usb_dev_msc_context_t *context)
{
    uint32_t index;

    context->in_buffer[0] = INQUIRY_PERIPHERAL_TYPE;
    context->in_buffer[1] = INQUIRY_REMOVABLE;
    context->in_buffer[2] = INQUIRY_NO_STANDARD;
    context->in_buffer[3] = INQUIRY_RESPONSE_DATA_FORMAT;
    context->in_buffer[4] = INQUIRY_ADDITIONAL_LENGTH;
    context->in_buffer[5] = INQUIRY_SCSI_STORAGE_CONTROLLER_PRESENT;
    context->in_buffer[6] = INQUIRY_MEDIUM_CHANGER_DEVICE;
    context->in_buffer[7] = 0;

    for (index = 0; index < sizeof(vendorIDT10); index++)
    {
        context->in_buffer[8+index] = vendorIDT10[index];
    }
    for (index = 0; index < sizeof(productID); index++)
    {
        context->in_buffer[16+index] = productID[index];
    }
    for (index = 0; index < sizeof(productRev); index++)
    {
        context->in_buffer[32+index] = productRev[index];
    }

    context->packet_in_size = 36;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_mode_select_6()
********************************************************************************
* Summary:
*  This command is not supported as of now.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always fail
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_mode_select_6(cy_stc_usb_dev_msc_context_t *context)
{
    context->packet_in_size = 0;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_DRV_HW_ERROR;
}

/*******************************************************************************
* Function Name: usb_scsi_mode_sense_6()
********************************************************************************
* Summary:
*  Responds to the SCSI Mode Sense6 command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_mode_sense_6(cy_stc_usb_dev_msc_context_t *context)
{
    context->in_buffer[0] = MODE_SENSE6_BLOCK_LENGTH;
    context->in_buffer[1] = DEFAULT_MEDIUM_TYPE;
    context->in_buffer[2] = 0;
    context->in_buffer[3] = 0;
    context->packet_in_size = 4;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_mode_select_10()
********************************************************************************
* Summary:
*  This command is not supported as of now.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always fail
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_mode_select_10(cy_stc_usb_dev_msc_context_t *context)
{
    context->packet_in_size = 0;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_DRV_HW_ERROR;
}

/*******************************************************************************
* Function Name: usb_scsi_mode_sense_10()
********************************************************************************
* Summary:
*  Responds to the SCSI Mode Sense10 command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_mode_sense_10(cy_stc_usb_dev_msc_context_t *context)
{
    memset(context->in_buffer, 0, context->packet_in_size);
    context->in_buffer[0] = MODE_SENSE10_BLOCK_LENGTH;
    context->in_buffer[1] = DEFAULT_MEDIUM_TYPE;
    context->packet_in_size = 8;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_start_stop_unit()
********************************************************************************
* Summary:
*  This command responds to the eject/mount requests from the host.
*
* Parameters:
*  eject_indicator: bit mask to indicate ejection
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_start_stop_unit(uint8_t eject_indicator)
{
    if((eject_indicator & LOEJ_BIT_FIELD) != 0)
    {
        if((eject_indicator & START_BIT_FIELD) == true)
        {
            mediaEjectedState = false;
        }
        else
        {
            mediaEjectedState = true;
        }
    }

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_read_format_capacities()
********************************************************************************
* Summary:
*  Responds to the SCSI Read Format Capacities command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_read_format_capacities(cy_stc_usb_dev_msc_context_t *context)
{
    context->in_buffer[0] = 0;
    context->in_buffer[1] = 0;
    context->in_buffer[2] = 0;
    context->in_buffer[3] = FORMAT_CAP_LIST_LENGTH;
    context->in_buffer[4] = CY_HI8(CY_HI16(MSC_NUM_OF_BLOCKS));
    context->in_buffer[5] = CY_LO8(CY_HI16(MSC_NUM_OF_BLOCKS));
    context->in_buffer[6] = CY_HI8(CY_LO16(MSC_NUM_OF_BLOCKS));
    context->in_buffer[7] = CY_LO8(CY_LO16(MSC_NUM_OF_BLOCKS));
    context->in_buffer[8] = FORMAT_CAP_FORMATTED_MEDIA;
    context->in_buffer[9] = CY_LO8(CY_HI16(MSC_BLOCKSIZE));
    context->in_buffer[10] = CY_HI8(CY_LO16(MSC_BLOCKSIZE));
    context->in_buffer[11] = CY_LO8(CY_LO16(MSC_BLOCKSIZE));
    context->packet_in_size = 12;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_read_capacity()
********************************************************************************
* Summary:
*  Responds to the SCSI Read Capacity command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_read_capacity(cy_stc_usb_dev_msc_context_t *context)
{
    context->in_buffer[0] = CY_HI8(CY_HI16(MSC_NUM_OF_BLOCKS - 1));
    context->in_buffer[1] = CY_LO8(CY_HI16(MSC_NUM_OF_BLOCKS - 1));
    context->in_buffer[2] = CY_HI8(CY_LO16(MSC_NUM_OF_BLOCKS - 1));
    context->in_buffer[3] = CY_LO8(CY_LO16(MSC_NUM_OF_BLOCKS - 1));
    context->in_buffer[4] = CY_HI8(CY_HI16(MSC_BLOCKSIZE));
    context->in_buffer[5] = CY_LO8(CY_HI16(MSC_BLOCKSIZE));
    context->in_buffer[6] = CY_HI8(CY_LO16(MSC_BLOCKSIZE));
    context->in_buffer[7] = CY_LO8(CY_LO16(MSC_BLOCKSIZE));
    context->packet_in_size = 8;
    context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_read_sense_10()
********************************************************************************
* Summary:
*  Responds to the SCSI Read Sense10 command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
* Return:
*  Always success
*
*******************************************************************************/
cy_en_usb_dev_status_t usb_scsi_read_sense_10(cy_stc_usb_dev_msc_context_t *context, uint8_t *mem)
{
    uint32_t epIndex, index;

    if (context->toggle_in == 0)
    {
        if (context->bytes_to_transfer > CY_USB_DEV_MSC_EP_BUF_SIZE)
        {
            context->packet_in_size = CY_USB_DEV_MSC_EP_BUF_SIZE;
        }
        else
        {
            context->packet_in_size = context->bytes_to_transfer;
        }

        if ((context->start_location + context->packet_in_size) > MSC_TOTAL_MEM_SIZE)
        {
            context->packet_in_size = MSC_TOTAL_MEM_SIZE - context->start_location;
        }

        if (context->start_location > MSC_SRAM_END_LOC)
        {
            /* Return spaces if host tries to read memory locations outside emulated memory in SRAM. */
            for (epIndex = 0, index = context->start_location; epIndex < context->packet_in_size; epIndex++, index++)
            {
                context->in_buffer[epIndex] = 0x20;
            }
            context->toggle_in = 1;
        }
        else
        {
            /* Allow reads from emulated memory if the requested location is within the SRAM. */
            for(epIndex = 0, index = context->start_location; epIndex < context->packet_in_size; epIndex++, index++)
            {
                context->in_buffer[epIndex] = mem[index];
            }
        }
    }

    if ((context->start_location + context->packet_in_size) > MSC_TOTAL_MEM_SIZE)
    {
        context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;
    }

    context->toggle_in = 0;

    return CY_USB_DEV_SUCCESS;
}

/*******************************************************************************
* Function Name: usb_scsi_write_10()
********************************************************************************
* Summary:
*  Handles the SCSI Write10 command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
*******************************************************************************/
void usb_scsi_write_10(cy_stc_usb_dev_msc_context_t *context)
{
    if (context->toggle_out == 0)
    {
        if ((context->start_location + context->packet_out_size) > MSC_TOTAL_MEM_SIZE)
        {
            context->packet_out_size = MSC_TOTAL_MEM_SIZE - context->start_location;
            context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;
        }
    }

    context->start_location += context->packet_out_size;
    context->bytes_to_transfer -= context->packet_out_size;

    context->cmd_status.data_residue -= context->packet_out_size;

    if (context->bytes_to_transfer == 0 || context->state == CY_USB_DEV_MSC_STATUS_TRANSPORT)
    {
        context->cmd_status.status = CY_USB_DEV_SUCCESS;
        context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;
    }
    context->toggle_out = 0;
}

/*******************************************************************************
* Function Name: usb_scsi_verify_10()
********************************************************************************
* Summary:
*  Handles the SCSI Verify10 command.
*
* Parameters:
*  context: pointer to the USB MSC context
*
*******************************************************************************/
void usb_scsi_verify_10(cy_stc_usb_dev_msc_context_t *context)
{
    if ((context->start_location + context->packet_out_size) > MSC_TOTAL_MEM_SIZE)
    {
        context->packet_out_size = MSC_TOTAL_MEM_SIZE - context->start_location;
        context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;
    }

    context->start_location += context->packet_out_size;
    context->bytes_to_transfer -= context->packet_out_size;

    context->cmd_status.data_residue -= context->packet_out_size;

    if (context->bytes_to_transfer == 0 || context->state == CY_USB_DEV_MSC_STATUS_TRANSPORT)
    {
        context->cmd_status.status = CY_USB_DEV_SUCCESS;
        context->state = CY_USB_DEV_MSC_STATUS_TRANSPORT;
    }
}
