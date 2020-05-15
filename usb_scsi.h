/*****************************************************************************
* File Name: usb_scsi.h
*
* Description:
*  This file contains the function prototypes and constants used in
*  the usb_scsi.c.
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

#ifndef USB_SCSI_H_
#define USB_SCSI_H_

#include "cy_usb_dev_msc.h"
#include "config.h"

/*******************************************************************************
* Constants
********************************************************************************/
#define MSC_TOTAL_MEM_SIZE      600000u
#define MSC_BLOCKSIZE           512
#define MSC_NUM_OF_BLOCKS       (MSC_TOTAL_MEM_SIZE/MSC_BLOCKSIZE)
#define MSC_SRAM_END_LOC        (MSC_BLOCKSIZE*7+CONFIG_LOG_FILE_SIZE)

#define STATUS_FILE_TIMEOUT                         200

/* Start Stop Unit */
#define LOEJ_BIT_FIELD                              0x02
#define START_BIT_FIELD                             0x01

/* Request Sense */
#define SENSE_RESPONSE_CODE                         0x70
#define SENSE_KEY_NO_SENSE                          0x00
#define SENSE_KEY_NOT_READY                         0x02
#define SENSE_KEY_ILLEGAL_REQUEST                   0x05
#define SENSE_KEY_UNIT_ATTENTION                    0x06
#define SENSE_ADDITIONAL_LENGTH                     0x0A
#define SENSE_ASC_NO_SENSE                          0x00
#define SENSE_ASC_MEDIA_REMOVAL                     0x3A
#define SENSE_ASC_INVALID_FIELD_IN_CDB              0x24
#define SENSE_ASCQ_NO_SENSE                         0x00

/* Inquiry */
#define DIRECT_ACCESS_DEVICE                        0x00

#define INQUIRY_PERIPHERAL_TYPE                     DIRECT_ACCESS_DEVICE
#define INQUIRY_REMOVABLE                           0x80
#define INQUIRY_NO_STANDARD                         0x00
#define INQUIRY_RESPONSE_DATA_FORMAT                0x01
#define INQUIRY_ADDITIONAL_LENGTH                   0x20
#define INQUIRY_SCSI_STORAGE_CONTROLLER_PRESENT     0x80
#define INQUIRY_MEDIUM_CHANGER_DEVICE               0x08

/* Mode Sense 6 */
#define DEFAULT_MEDIUM_TYPE                         0x00
#define MODE_SENSE6_BLOCK_LENGTH                    0x03

/* Mode Sense 10 */
#define MODE_SENSE10_BLOCK_LENGTH                   0x06

/* Read format capacity */
#define FORMAT_CAP_LIST_LENGTH                      0x08
#define FORMAT_CAP_FORMATTED_MEDIA                  0x02

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void usb_scsi_serve_timeout(void);
cy_en_usb_dev_status_t usb_scsi_prevent_media_removal(uint8_t prevent);
cy_en_usb_dev_status_t usb_scsi_test_unit_ready(void);
cy_en_usb_dev_status_t usb_scsi_request_sense(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_format_unit(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_inquiry(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_mode_select_6(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_mode_sense_6(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_mode_select_10(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_mode_sense_10(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_start_stop_unit(uint8_t eject_indicator);
cy_en_usb_dev_status_t usb_scsi_read_format_capacities(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_read_capacity(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_read_capacity(cy_stc_usb_dev_msc_context_t *context);
cy_en_usb_dev_status_t usb_scsi_read_sense_10(cy_stc_usb_dev_msc_context_t *context, uint8_t *mem);
void usb_scsi_write_10(cy_stc_usb_dev_msc_context_t *context);
void usb_scsi_verify_10(cy_stc_usb_dev_msc_context_t *context);

#endif /* USB_SCSI_H_ */
