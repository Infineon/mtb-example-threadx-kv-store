/*******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the KV Store Application Example
 *              to be used as starting template.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Header Files
 *******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_kvstore_cat5.h"
#include "cyabs_rtos.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define key 1

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void readData();
void writeData();

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
uint8_t dataReadWrite[9];
mtb_kvstore_t kvstore_obj; //Kvstore block device

/*******************************************************************************
 * Function Name: main()
 ********************************************************************************
 * Summary:
 * This is the main function for CPU. It...
 *    1. Initializes the BSP
 *    2. Enables Global interrupt
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
int main(void)
{
    uint8_t readbyte;

    cybsp_init();
    __enable_irq();

    // Prevent the device from going to sleep while the program is active
    cyhal_syspm_lock_deepsleep();

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    printf("\x1b[2J\x1b[;H");
    printf("**********************************************************\n");
    printf("                       KV Store                     \n");
    printf("**********************************************************\n");
    for(;;)
    {
        printf("\n******************** MENU **********************\r\n");
        printf("**1) Press '1' to read stored data            **\r\n");
        printf("**2) Press '2' to write/overwrite the data    **\r\n");
        printf("**3) Press 'r' to reset kv-store (delete data)**\r\n");
        printf("************************************************\r\n\n");

        mtb_kvstore_init(&kvstore_obj);

        while(!cyhal_uart_readable(&cy_retarget_io_uart_obj))
        {
            cy_rtos_delay_milliseconds(100);
        }
        cyhal_uart_getc(&cy_retarget_io_uart_obj , &readbyte, 0);
        switch (readbyte)
        {
        case '1':
            readData();
            break;
        case '2':
            writeData();
            break;
        case 'r':
            mtb_kvstore_reset(&kvstore_obj);
            printf("Reset successful\n\n");
            break;
        default:
            printf("Invalid Input\r\n\n");
            break;
        }
    }
}

/******************************************************************************
 * Function Name: readData
 *******************************************************************************
 * Summary:
 *  Reads the data stored in the external flash.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void readData()
{
    cy_rslt_t result;
    uint32_t *readsize=(uint32_t *)malloc(sizeof(uint32_t));
    *readsize=8; //No. of characters to be read

    result = mtb_kvstore_read_numeric_key(&kvstore_obj, key, dataReadWrite, readsize);
    if(result == CY_RSLT_SUCCESS)
    {
        printf("The date stored is: %c%c/%c%c/%s\n\n", dataReadWrite[0], dataReadWrite[1], dataReadWrite[2], dataReadWrite[3], &dataReadWrite[4]);
    }
    else
    {
        printf("No data found. Press 2 to write the data\n\n");
    }
    free(readsize);
    memset(dataReadWrite, 0, sizeof(dataReadWrite));
}

/******************************************************************************
 * Function Name: writeData
 *******************************************************************************
 * Summary:
 *  reads the input through uart and Writes the input to the external flash.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void writeData()
{
    cy_rslt_t result;
    uint8_t uart_read_value;

    printf("Enter the date in this format: 'ddmmyyyy': \n");
    for (;;)
    {
        /* Check if a character is received */
        while(!cyhal_uart_readable(&cy_retarget_io_uart_obj))
        {
            cy_rtos_delay_milliseconds(100);
        }
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 0) == CY_RSLT_SUCCESS)
        {
            if (strlen((const char*)dataReadWrite) < 8)  // Check if the input buffer is not full
            {
                dataReadWrite[strlen((const char*)dataReadWrite)] = uart_read_value;  // Append the received character to the input buffer
                dataReadWrite[strlen((const char*)dataReadWrite) + 1] = '\0';  // Ensure null-terminated string
            }
            if (strlen((const char*)dataReadWrite) == 8)  // Check if the input buffer contains exactly eight characters
            {
                //read stored data at key
                result = mtb_kvstore_write_numeric_key(&kvstore_obj, key, dataReadWrite, sizeof(dataReadWrite), true);
                if (result == CY_RSLT_SUCCESS)
                {
                    printf("The date %c%c/%c%c/%s stored successfully\n\n",dataReadWrite[0], dataReadWrite[1], dataReadWrite[2], dataReadWrite[3], &dataReadWrite[4]);
                    memset(dataReadWrite, 0, sizeof(dataReadWrite));
                }
                else
                {
                    printf("Error writing data to key\n\n");
                }
                break;
            }
        }
    }
}
/* [] END OF FILE */
