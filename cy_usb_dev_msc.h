/***************************************************************************//**
* \file cy_usb_dev_msc.h
* \version 0.1
*
* Provides Mass Storage class-specific API declarations.
*
********************************************************************************
* \copyright
* Copyright 2020, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
* \addtogroup group_usb_dev_msc
* This section provides API description for the Mass Storage class.
* \{
*    \defgroup group_usb_dev_msc_macros          Macros
*    \defgroup group_usb_dev_msc_functions       Functions
*    \defgroup group_usb_dev_msc_data_structures Data Structures
* \}
*/

#if !defined(CY_USB_DEV_MSC_H)
#define CY_USB_DEV_MSC_H

#include "cy_usb_dev.h"

#if defined(CY_IP_MXUSBFS)

#if defined(__cplusplus)
extern "C" {
#endif


#define CY_USB_DEV_MSC_CMD_SIZE         0x10
#define CY_USB_DEV_MSC_EP_BUF_SIZE      64u

/*******************************************************************************
*                          Enumerated Types
*******************************************************************************/
typedef enum
{
    CY_USB_DEV_MSC_READY_STATE,
    CY_USB_DEV_MSC_COMMAND_TRANSPORT,
    CY_USB_DEV_MSC_DATA_OUT,
    CY_USB_DEV_MSC_DATA_IN,
    CY_USB_DEV_MSC_NO_DATA,
    CY_USB_DEV_MSC_STATUS_TRANSPORT,
    CY_USB_DEV_MSC_STALL_IN_ENDPOINT,
    CY_USB_DEV_MSC_STALL_OUT_ENDPOINT,
    CY_USB_DEV_MSC_WAIT_FOR_RESET,
} cy_en_usb_dev_msc_state_t;

/*******************************************************************************
*                          Type Definitions
*******************************************************************************/

/**
* \addtogroup group_usb_dev_msc_data_structures
* \{
*/

typedef struct
{
    uint32_t signature;
    uint32_t tag;
    uint32_t data_transfer_length;
    uint8_t  flags;
    uint8_t  lun;
    uint8_t  length;
    uint8_t  cmd[CY_USB_DEV_MSC_CMD_SIZE];
} cy_stc_usb_dev_msc_cmd_block_t;

typedef struct
{
    uint32_t signature;
    uint32_t tag;
    uint32_t data_residue;
    uint8_t  status;
} cy_stc_usb_dev_msc_cmd_status_t;

/** Mass Storage class context structure.
* All fields for the MSC context structure are internal. Firmware never reads or
* writes these values. Firmware allocates the structure and provides the
* address of the structure to the middleware in MSC function calls. Firmware
* must ensure that the defined instance of this structure remains in scope while
* the middleware is in use.
*/
typedef struct
{
    /** \cond INTERNAL*/

    /** Pointer to device context */
    cy_stc_usb_dev_context_t *devContext;

    /** Mass Storage class functions pointers */
    cy_stc_usb_dev_class_t classObj;

    /** Mass Storage class linked list item */
    cy_stc_usb_dev_class_ll_item_t classItem;

    /** Current state **/
    cy_en_usb_dev_msc_state_t state;

    /* Command Block */
    cy_stc_usb_dev_msc_cmd_block_t  cmd_block;

    /* Command Status */
    cy_stc_usb_dev_msc_cmd_status_t cmd_status;

    /* Toggle flag for IN endpoint */
    uint8_t toggle_in;

    /* Toggle flag for OUT endpoint */
    uint8_t toggle_out;

    /* Packet in size */
    uint8_t packet_in_size;

    /* Packet out size */
    uint8_t packet_out_size;

    /* Bytes to transfer */
    uint32_t bytes_to_transfer;

    /* Start location */
    uint32_t start_location;

    /* IN Endpoint Buffer */
    uint8_t in_buffer[CY_USB_DEV_MSC_EP_BUF_SIZE];

    /* OUT Endpoint Buffer */
    uint8_t out_buffer[CY_USB_DEV_MSC_EP_BUF_SIZE];

    /** \endcond */

} cy_stc_usb_dev_msc_context_t;


/** \} group_usb_dev_msc_data_structures */


/*******************************************************************************
*                          Function Prototypes
*******************************************************************************/

/**
* \addtogroup group_usb_dev_msc_functions
* \{
*/
cy_en_usb_dev_status_t Cy_USB_Dev_Msc_Init(void const *config,
                                           cy_stc_usb_dev_msc_context_t      *context,
                                           cy_stc_usb_dev_context_t           *devContext);

__STATIC_INLINE void Cy_USB_Dev_Msc_RegisterUserCallback(cy_cb_usb_dev_request_received_t requestReceivedHandle,
                                                           cy_cb_usb_dev_request_cmplt_t  requestCompletedHandle,
                                                           cy_stc_usb_dev_msc_context_t *context);

__STATIC_INLINE cy_stc_usb_dev_class_t * Cy_USB_Dev_Msc_GetClass(cy_stc_usb_dev_msc_context_t *context);
/** \} group_usb_dev_msc_functions */


/*******************************************************************************
*                         API Constants
*******************************************************************************/

/**
* \addtogroup group_usb_dev_msc_macros
* \{
*/
#define CY_USB_DEV_MSC_GET_MAX_LUN		0xFE
#define CY_USB_DEV_MSC_RESET			0xFF

#define CY_USB_DEV_MSC_SCSI_TEST_UNIT_READY             0x00
#define CY_USB_DEV_MSC_SCSI_REQUEST_SENSE               0x03
#define CY_USB_DEV_MSC_SCSI_FORMAT_UNIT                 0x04
#define CY_USB_DEV_MSC_SCSI_INQUIRY                     0x12
#define CY_USB_DEV_MSC_SCSI_MODE_SELECT6                0x15
#define CY_USB_DEV_MSC_SCSI_MODE_SENSE6                 0x1A
#define CY_USB_DEV_MSC_SCSI_START_STOP_UNIT             0x1B
#define CY_USB_DEV_MSC_SCSI_MEDIA_REMOVAL               0x1E
#define CY_USB_DEV_MSC_SCSI_READ_FORMAT_CAPACITIES      0x23
#define CY_USB_DEV_MSC_SCSI_READ_CAPACITY               0x25
#define CY_USB_DEV_MSC_SCSI_READ10                      0x28
#define CY_USB_DEV_MSC_SCSI_WRITE10                     0x2A
#define CY_USB_DEV_MSC_SCSI_VERIFY10                    0x2F
#define CY_USB_DEV_MSC_SCSI_MODE_SELECT10               0x55
#define CY_USB_DEV_MSC_SCSI_MODE_SENSE10                0x5A

#define CY_USB_DEV_MSC_CMD_BLOCK_SIZE                   0x1F
#define CY_USB_DEV_MSC_CMD_STATUS_SIZE                  0x0D

#define MSC_CBW_SIGNATURE                               0x43425355
#define MSC_CSW_SIGNATURE                               0x53425355

/** \} group_usb_dev_msc_macros */


/*******************************************************************************
*                          Internal Constants
*******************************************************************************/


/*******************************************************************************
*                           In-line Function Implementation
*******************************************************************************/

/**
* \addtogroup group_usb_dev_msc_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_USB_Dev_Msc_RegisterUserCallback
****************************************************************************//**
*
* Registers the user callbacks to handle Mass Storage class requests.
*
* \param requestReceivedHandle
* The pointer to a callback function.
* This function is called when setup packet was received from the USB Host but was
* not recognized. Therefore this might require Mass Storage class processing.
* To remove the callback function, pass a NULL as the function pointer.
*
* \param requestCompletedHandle
* The pointer to a callback function.
* This function is called when the USB Device received data from the USB Host
* as part of current request processing. The requestReceivedHandle function
* must enable notification to trigger this event. This makes sense only when class
* request processing requires a data stage.
* To remove the callback function, pass a NULL as the function pointer.
*
* \param context
* The pointer to the context structure \ref cy_stc_usb_dev_context_t allocated
* by the user. The structure is used during the Mass Storage Class operation for
* internal configuration and data retention. The user must not modify anything
* in this structure.
*
*******************************************************************************/
__STATIC_INLINE void Cy_USB_Dev_Msc_RegisterUserCallback(cy_cb_usb_dev_request_received_t requestReceivedHandle,
                                                           cy_cb_usb_dev_request_cmplt_t    requestCompletedHandle,
                                                           cy_stc_usb_dev_msc_context_t   *context)
{
    Cy_USB_Dev_RegisterClassRequestRcvdCallback(requestReceivedHandle,   Cy_USB_Dev_Msc_GetClass(context));
    Cy_USB_Dev_RegisterClassRequestCmpltCallback(requestCompletedHandle, Cy_USB_Dev_Msc_GetClass(context));
}


/*******************************************************************************
* Function Name: Cy_USB_Dev_Msc_GetClass
****************************************************************************//**
*
* Returns pointer to the class structure for Mass Storage class.
*
* \param context
* The pointer to the context structure \ref cy_stc_usb_dev_context_t allocated
* by the user. The structure is used during the Mass Storage Class operation for
* internal configuration and data retention. The user must not modify anything
* in this structure.
*
* \return
* Status pointer to the class \ref cy_stc_usb_dev_class_t.
*
*******************************************************************************/
__STATIC_INLINE cy_stc_usb_dev_class_t * Cy_USB_Dev_Msc_GetClass(cy_stc_usb_dev_msc_context_t *context)
{
    return &(context->classObj);
}

/** \} group_usb_dev_msc_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_IP_MXUSBFS) */

#endif /* (CY_USB_DEV_MSC_H) */


/* [] END OF FILE */
