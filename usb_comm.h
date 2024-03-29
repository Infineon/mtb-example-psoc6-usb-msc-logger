/*****************************************************************************
* File Name: usb_comm.h
*
* Description:
*  This file contains the function prototypes and constants used in
*  the usb_comm.c.
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

#ifndef USB_COMM_H_
#define USB_COMM_H_

#include "cy_usb_dev.h"
#include "cy_usb_dev_msc.h"

/*******************************************************************************
* Constants
********************************************************************************/
#define MSC_OUT_ENDPOINT_ADDR   0x02
#define MSC_IN_ENDPOINT_ADDR    0x81
#define MSC_OUT_ENDPOINT        0x02
#define MSC_IN_ENDPOINT         0x01

/*******************************************************************************
* USB Communication Functions
*******************************************************************************/
void     usb_comm_init(void);
void     usb_comm_connect(void);
void     usb_comm_link_fs(uint8_t *fs);
bool     usb_comm_is_ready(void);
void     usb_comm_refresh(void);
void     usb_comm_process(void);


#endif /* USB_COMM_H_ */
