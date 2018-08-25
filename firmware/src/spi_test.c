/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    spi_test.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "spi_test.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

SPI_TEST_DATA spi_testData;
static uint8_t __attribute__ ((aligned (16))) spi_test_spi_tx_buffer[] = "Hello World";

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* state machine for the SPI */
static void SPI_Task(void)
{
    /* run the state machine here for SPI */
    switch (spi_testData.spiStateMachine)
    {
        default:
        case SPI_TEST_SPI_STATE_START:
            /* set the state to 'wait' early so that the interrupt doesn't
                finish fast and write the state and then is overwritten */
            spi_testData.spiStateMachine =  SPI_TEST_SPI_STATE_WAIT;
            PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12);
            spi_testData.drvSPIBufferHandle = DRV_SPI_BufferAddWrite(spi_testData.handleSPI0,
                spi_test_spi_tx_buffer, sizeof(spi_test_spi_tx_buffer),
                0, 0);
            PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12);

            if (DRV_SPI_BUFFER_HANDLE_INVALID == spi_testData.drvSPIBufferHandle)
            {
                /* try again if we get a bad handle */
                spi_testData.spiStateMachine =  SPI_TEST_SPI_STATE_START;
            }
        break;

        case SPI_TEST_SPI_STATE_WAIT:
        {
            if ( DRV_SPI_BufferStatus(spi_testData.drvSPIBufferHandle) & DRV_SPI_BUFFER_EVENT_COMPLETE)
            {
                spi_testData.spiStateMachine = SPI_TEST_SPI_STATE_START;
            }
        }
        break;

        case SPI_TEST_SPI_STATE_DONE:
        break;
    }
}


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SPI_TEST_Initialize ( void )

  Remarks:
    See prototype in spi_test.h.
 */

void SPI_TEST_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    spi_testData.state = SPI_TEST_STATE_INIT;

    spi_testData.handleSPI0 = DRV_HANDLE_INVALID;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
}


/******************************************************************************
  Function:
    void SPI_TEST_Tasks ( void )

  Remarks:
    See prototype in spi_test.h.
 */

void SPI_TEST_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( spi_testData.state )
    {
        /* Application's initial state. */
        case SPI_TEST_STATE_INIT:
        {
            bool appInitialized = true;
       

            if (DRV_HANDLE_INVALID == spi_testData.handleSPI0)
            {
                spi_testData.handleSPI0 = DRV_SPI_Open(0, DRV_IO_INTENT_WRITE);
                appInitialized &= (DRV_HANDLE_INVALID != spi_testData.handleSPI0);
            }
        
            if (appInitialized)
            {
                /* initialize the SPI state machine */
                spi_testData.spiStateMachine = SPI_TEST_SPI_STATE_START;
            
                spi_testData.state = SPI_TEST_STATE_SERVICE_TASKS;
            }
            break;
        }

        case SPI_TEST_STATE_SERVICE_TASKS:
        {
            /* run the state machine for servicing the SPI */
            SPI_Task();
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
