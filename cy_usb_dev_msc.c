/***************************************************************************//**
* \file cy_usb_dev_msc.c
* \version 0.1
*
* Provides Mass Storage class-specific API implementation.
*
********************************************************************************
* \copyright
* Copyright 2020, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_usb_dev_msc.h"

#if defined(CY_IP_MXUSBFS)

/*******************************************************************************
* Function Name: Cy_USB_Dev_Msc_Init
****************************************************************************//**
*
* Initializes the Mass Storage class.
* This function must be called to enable USB Device Mass Storage functionality.
*
* \param config
* Pass NULL as an argument (left for future purposes).
*
* \param context
* The pointer to the context structure \ref cy_stc_usb_dev_msc_context_t
* allocated by the user. The structure is used during the MSC Class operation
* for internal configuration and data retention. The user must not modify
* anything in this structure.
*
* \param devContext
* The pointer to the USB Device context structure \ref cy_stc_usb_dev_context_t.
*
* \return
* Status code of the function execution \ref cy_en_usb_dev_status_t.
*
*******************************************************************************/
cy_en_usb_dev_status_t Cy_USB_Dev_Msc_Init(void const *config,
                                             cy_stc_usb_dev_msc_context_t      *context,
                                             cy_stc_usb_dev_context_t          *devContext)
{
    /* Suppress a compiler warning about unused variables */
    (void) config;

    if ((NULL == context) || (NULL == devContext))
    {
        return CY_USB_DEV_BAD_PARAM;
    }

    context->state = CY_USB_DEV_MSC_READY_STATE;
    context->toggle_in = 0;
    context->toggle_out = 0;
    context->cmd_status.signature = MSC_CSW_SIGNATURE;

    /* Store device context */
    context->devContext = devContext;

    return Cy_USB_Dev_RegisterClass(&context->classItem, &context->classObj, context, devContext);
}

#endif /* CY_IP_MXUSBFS) */


/* [] END OF FILE */

