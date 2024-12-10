/******************************************************************************
* File Name:   ns_ipc.c
*
* Description: IPC init
*
* Related Document: See README.md
*
*******************************************************************************
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include <stdio.h>

/*******************************************************************************
* Macros
*******************************************************************************/
/** Make sure that cm33-s app should use same ipc channel to recv message from cm33-ns */
#define CM33_NS_IPC_CH_NUM                  (CY_IPC_CHAN_USER)
#define CM33_NS_IPC_CH_MASK                 (CY_IPC_CH_MASK(CM33_NS_IPC_CH_NUM))
#define CM33_NS_IPC_INTR_NUM                (CY_IPC_INTR_USER)
#define CM33_NS_IPC_INTR_MASK               (CY_IPC_INTR_MASK(CM33_NS_IPC_INTR_NUM))
#define CM33_NS_IPC_INTR_MUX                (CY_IPC_INTR_MUX(CM33_NS_IPC_INTR_NUM))
/******************************************************************************/

#define IPC_PRIORITY                        (1u)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* Setup the  IPC Interrupt */
const cy_stc_sysint_t ipcTestIntConfig =
{
          .intrSrc = (IRQn_Type)CM33_NS_IPC_INTR_MUX,
          .intrPriority = IPC_PRIORITY,
};

volatile bool notify_event, rel_event;
uint32_t writeMesg;

/*******************************************************************************
* Function Name: ipc_isr
********************************************************************************
* Summary:
* This is the ipc isr
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

void ipc_isr(void)
{
     uint32_t shadowIntr;
     IPC_STRUCT_Type *ipcPtr;
     IPC_INTR_STRUCT_Type *ipcIntrPtr;

     ipcIntrPtr = Cy_IPC_Drv_GetIntrBaseAddr(CM33_NS_IPC_INTR_NUM);
     shadowIntr = Cy_IPC_Drv_GetInterruptStatusMasked(ipcIntrPtr);

     /* Check to make sure the interrupt was a notify interrupt */
     if (0UL != Cy_IPC_Drv_ExtractAcquireMask(shadowIntr))
     {
          printf("Recv. notify event \r\n");

          ipcPtr = Cy_IPC_Drv_GetIpcBaseAddress(CM33_NS_IPC_CH_NUM);
          /* Clear the notify interrupt.  */
          Cy_IPC_Drv_ClearInterrupt(ipcIntrPtr, CY_IPC_NO_NOTIFICATION, Cy_IPC_Drv_ExtractAcquireMask(shadowIntr));

          if ( Cy_IPC_Drv_IsLockAcquired (ipcPtr))
          {
               /** process notify event */

               /* Must always release the IPC channel */
               (void)Cy_IPC_Drv_LockRelease (ipcPtr, CM33_S_IPC_INTR_MASK);
          }
          notify_event = true;
     }

     /* Check to make sure the interrupt was a release interrupt */
     if (0UL != Cy_IPC_Drv_ExtractReleaseMask(shadowIntr))  /* Check for a Release interrupt */
     {
          /* Clear the release interrupt  */
          Cy_IPC_Drv_ClearInterrupt(ipcIntrPtr, Cy_IPC_Drv_ExtractReleaseMask(shadowIntr), CY_IPC_NO_NOTIFICATION);

          printf("Received release event \r\n");

          /** process release event */

          rel_event = true;
     }

     (void)Cy_IPC_Drv_GetInterruptStatus(ipcIntrPtr);
}

/*******************************************************************************
* Function Name: send_msg
********************************************************************************
* Summary:
* This is the send message function
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

void send_msg(void)
{
     cy_en_ipcdrv_status_t status;

     printf("IPC[%d] Write Data\r\n", CM33_S_IPC_CH_NUM);

     status=Cy_IPC_Drv_SendMsgPtr(Cy_IPC_Drv_GetIpcBaseAddress(CM33_S_IPC_CH_NUM),
               CM33_S_IPC_INTR_MASK,
               &writeMesg);
     if (status == CY_IPC_DRV_SUCCESS)
     {
          printf("[CM33-NS] send success \r\n" );
     }
     else
     {
          printf("[CM33-NS] send failed \r\n" );
     }

}

/*******************************************************************************
* Function Name: usecase_cm33ns_cm33s
********************************************************************************
* Summary:
* This is the secure and non-secure use case function
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
void usecase_cm33ns_cm33s(void)
{
     printf("\n\n\n[cm33-ns] Send message to cm33-s \r\n");

     send_msg();

}

/*******************************************************************************
* Function Name: config_current_core
********************************************************************************
* Summary:
* This is the config_current_core function
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
void config_current_core(void)
{
     Cy_SysInt_Init(&ipcTestIntConfig, ipc_isr);
     NVIC_EnableIRQ((IRQn_Type)CM33_NS_IPC_INTR_MUX);

     /* Set IPC Interrupt mask to receive Interrupt */
     Cy_IPC_Drv_SetInterruptMask(Cy_IPC_Drv_GetIntrBaseAddr(CM33_NS_IPC_INTR_NUM),
               (CM33_S_IPC_CH_MASK | CM33_NS_IPC_CH_MASK),
               (CM33_S_IPC_CH_MASK | CM33_NS_IPC_CH_MASK));

     rel_event = false;
     notify_event = false;
     writeMesg = 1;
}

/*******************************************************************************
* Function Name: test_multi_core_ipc
********************************************************************************
* Summary:
* This is the test_multi_core_ipc function
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
void test_multi_core_ipc(void)
{
     config_current_core();
     while(1){
          usecase_cm33ns_cm33s();
          Cy_SysLib_Delay(1000);
     }
}

/* [] END OF FILE */

