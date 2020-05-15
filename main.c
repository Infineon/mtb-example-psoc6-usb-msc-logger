/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the USB Mass Storage Logger
*              Application for ModusToolbox.
*
* Related Document: See Readme.md
*
*
*******************************************************************************
* (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "file_system.h"
#include "usb_comm.h"

#include <stdio.h>

/*******************************************************************************
* Constants
********************************************************************************/
#define MESSAGE_TEMPLATE "> Button press %d time(s)\r\n"
#define MESSAGE_SIZE    32u

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Pointer to the emulated file system */
uint8 *emulated_memory;

/* Flag to indicate when the user button is pressed */
volatile bool button_flag = false;

/* Counter to indicate how many times the button was pressed */
uint8_t button_count = 0;

/* String to be written to the file system */
char log_message[MESSAGE_SIZE];

/*******************************************************************************
* Local Functions
********************************************************************************/
void button_isr_handler(void *arg, cyhal_gpio_event_t event);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function for CM4 CPU. It initializes the USB block and 
*  enumerates as a Mass Storage Device. It links a local file system emulated in 
*  the SRAM. In the main loop, it constantely checks for button presses and 
*  process any USB requests.
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize the User Button */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, button_isr_handler, NULL);

    /* Initialize the USB block */
    usb_comm_init();

    /* Initialize the file system on SRAM */
    file_system_init(&emulated_memory);

    /* Link the file system to the USB Mass Storage Device Class */
    usb_comm_link_fs(emulated_memory);

    /* Start the USB enumeration */
    usb_comm_connect();

    for(;;)
    {
        /* Process any USB requests */
        usb_comm_process();

        /* Check if the user button was pressed */
        if (button_flag)
        {
            /* Clear flag */
            button_flag = false;

            /* Build the message */
            sprintf(log_message, MESSAGE_TEMPLATE, ++button_count);

            /* Write to the file system */
            if (false == file_system_write(log_message, strlen(log_message)))
            {
                /* If failed, clear the file */
                file_system_erase_all();
            }

            /* Emulate ejection every 5 times */
            if ((button_count % 5) == 0)
            {
                usb_comm_refresh();
            }            
        }
    }
}

/*******************************************************************************
* Function Name: button_isr_handler
********************************************************************************
* Summary:
*  Button ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void button_isr_handler(void *arg, cyhal_gpio_event_t event)
{
    (void) arg;
    (void) event;

    button_flag = true;
}



/* [] END OF FILE */
