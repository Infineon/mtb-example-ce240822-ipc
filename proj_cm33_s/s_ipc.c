/******************************************************************************
* File Name:   s_ipc.c
*
* Description: This is the source code for the CANFD example
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

#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_ms_ctl.h"
#include "cy_ipc_drv.h"
#include "cy_sysint.h"
#include "cy_ipc_sema.h"
#include "cy_ipc_pipe.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define CM33_NS_IPC_CH_NUM                 (CY_IPC_CHAN_USER)
#define CM33_NS_IPC_CH_MASK                (CY_IPC_CH_MASK(CM33_NS_IPC_CH_NUM))
#define CM33_NS_IPC_INTR_NUM               (CY_IPC_INTR_USER)
#define CM33_NS_IPC_INTR_MASK              (CY_IPC_INTR_MASK(CM33_NS_IPC_INTR_NUM))
#define CM33_NS_IPC_INTR_MUX               (CY_IPC_INTR_MUX(CM33_NS_IPC_INTR_NUM))
#define CM33_S_IPC_INTR_PRIORITY           (1U)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* Setup the  IPC Interrupt */
const cy_stc_sysint_t ipcIntConfig =
{
          .intrSrc = (IRQn_Type)CM33_S_IPC_INTR_MUX,
          .intrPriority = CM33_S_IPC_INTR_PRIORITY
};

uint32_t writeMesg;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void ipc_cm33_s_isr(void);
int config_ipc(void);

/*******************************************************************************
* Function Name: ipc_cm33_s_isr
********************************************************************************
* Summary:
* This is the ipc_cm33_s_isr function. 
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

void ipc_cm33_s_isr(void)
{
     uint32_t shadowIntr;
     IPC_STRUCT_Type *ipcPtr;
     IPC_INTR_STRUCT_Type *ipcIntrPtr;
     uint32_t notifyMask;

     ipcIntrPtr = Cy_IPC_Drv_GetIntrBaseAddr(CM33_S_IPC_INTR_NUM); /** all interrupt to cm33-s has to come from ipc-intr 0 */
     shadowIntr = Cy_IPC_Drv_GetInterruptStatusMasked(ipcIntrPtr);
     notifyMask = Cy_IPC_Drv_ExtractAcquireMask(shadowIntr);

     /* Notify Interrupt: Remote core sent message to cm33-s */
     if (0UL != notifyMask)
     {
          /* Clear the notify interrupt in IPC interrupt register */
          Cy_IPC_Drv_ClearInterrupt(ipcIntrPtr, CY_IPC_NO_NOTIFICATION, notifyMask);
          ipcPtr = Cy_IPC_Drv_GetIpcBaseAddress (CM33_S_IPC_CH_NUM);
          if (Cy_IPC_Drv_IsLockAcquired(ipcPtr))
          {

               Cy_GPIO_Inv(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN);
          }

          /* release the current channel and trigger release event to the remote core */
          Cy_IPC_Drv_LockRelease(ipcPtr, CY_IPC_INTR_MASK(CM33_NS_IPC_INTR_NUM));
     }
     /* set pc value to cm33-ns before returning from this isr */
     Cy_Ms_Ctl_SetActivePC(CPUSS_MS_ID_CM33_0, CM33_NS_PC_VALUE);
}

/*******************************************************************************
* Function Name: config_ipc
********************************************************************************
* Summary:
* This is the config_ipc function. 
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int config_ipc(void)
{
     cy_en_sysint_status_t intrStatus = CY_SYSINT_SUCCESS;

     intrStatus = Cy_SysInt_Init(&ipcIntConfig, ipc_cm33_s_isr);

     /* enable interrupts */
     __enable_irq();
     NVIC_EnableIRQ((IRQn_Type)CM33_S_IPC_INTR_MUX);

     /* Set IPC Interrupt mask to allow remote core to interrupt cm33-s core */
     Cy_IPC_Drv_SetInterruptMask(    Cy_IPC_Drv_GetIntrBaseAddr(CM33_S_IPC_INTR_NUM),
               (CM33_S_IPC_CH_MASK | CM33_NS_IPC_CH_MASK),
               (CM33_S_IPC_CH_MASK | CM33_NS_IPC_CH_MASK));

     return (int)intrStatus;
}
/* [] END OF FILE */
