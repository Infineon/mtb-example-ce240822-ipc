/*****************************************************************************
 * File Name        : main.c
 *
 * Description      : This source file contains the main routine for secure
 *                    application in the CM33 CPU
 *
 * Related Document : See README.md
 *
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

/******************************************************************************
 * Header Files
 *****************************************************************************/

#include "cybsp.h"
#include <arm_cmse.h>
#include "cy_pdl.h"
#include "partition_ARMCM33.h"
#include "cy_mpc.h"
#include "cy_ppc.h"
#include "cy_ms_ctl.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/* CM33 NS image stack pointer address */
#define CM33_SP_STORE                   NS_BOOT_START

/* CM33 NS image Reset handler address */
#define CM33_RESET_HANDLER_STORE        (NS_BOOT_START + 4)


/* Function pointer for NS function i.e. NonSecure_ResetHandler() */
typedef void (*funcptr_void) (void) __attribute__((cmse_nonsecure_call));

/******************************************************************************
 * Global Variables
 *****************************************************************************/


/******************************************************************************
 * Function Prototypes
 *****************************************************************************/

/**
 * @brief Configure PPC.
 *
 * This function configures the peripheral protection controller (PPC) to partition
 * the peripherals in to secure and non-secure worlds. The resources which are unused
 * by secure project are configured as non-secure. 
 * 
 * @return The status cy_en_ppc_status_t.
 */
cy_en_ppc_status_t config_ppc(void);

/**
 * @brief Configure M33 Protection Context.
 *
 * This function configures the M33 PC. The active and saved PC is set to PC3
 * for non-secure application. 
 * 
 * @return The status cy_en_ms_ctl_status_t.
 */
cy_en_ms_ctl_status_t config_set_cm33_ns_pc(void);

extern int config_ipc(void);


/******************************************************************************
 * Function Definitions
 *****************************************************************************/
/*******************************************************************************
* Function Name: config_ppc
********************************************************************************
* Summary:
* This is the config_ppc function.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
cy_en_ppc_status_t config_ppc(void)
{
     PPC_Type* ppcPtr = NULL;
     cy_en_ppc_status_t ppcStatus = CY_PPC_FAILURE;

     cy_stc_ppc_init_t ppcInit;
     cy_stc_ppc_attribute_t ppcAttribute;
     cy_stc_ppc_pc_mask_t pcMaskConfig;

     /* Configure PPC0 for CM33 access */
     ppcPtr = PPC;

     /* Initialize PPC */
     ppcInit.respConfig = CY_PPC_BUS_ERR;
     ppcStatus = Cy_Ppc_InitPpc(ppcPtr, &ppcInit);
     if (ppcStatus != CY_PPC_SUCCESS)
          return ppcStatus;

     ppcAttribute.secAttribute = CY_PPC_NON_SECURE;
     ppcAttribute.secPrivAttribute = CY_PPC_SEC_NONPRIV;
     ppcAttribute.nsPrivAttribute = CY_PPC_NON_SEC_NONPRIV;

     /* Address: 0x42000000 - 0x42004157 */
     ppcAttribute.startRegion = PROT_PERI0_MAIN;
     ppcAttribute.endRegion = PROT_PERI0_GR5_GROUP;
     ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     /* Address: 0x42040000 - 0x4204FFFF */
     ppcAttribute.startRegion = PROT_PERI_PCLK0_MAIN;
     ppcAttribute.endRegion = PROT_PERI_PCLK0_MAIN;
     ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     /* Address: 0x42100000 - 0x421102FF */
     ppcAttribute.startRegion = PROT_CPUSS;
     ppcAttribute.endRegion = PROT_RAMC0_RAM_PWR;
     ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     /* Address: 0x42160000 - 0x421600FF */
     ppcAttribute.startRegion = PROT_MXCM33_CM33;
     ppcAttribute.endRegion = PROT_MXCM33_CM33;
     ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     /* Address: 0x42168000 - 0x421c2007 */
     ppcAttribute.startRegion = PROT_MXCM33_CM33_INT;
     ppcAttribute.endRegion = PROT_CPUSS_BOOT;
     ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     /* Address: 0x421c8000 - 0x42BFFFFF */
     ppcAttribute.startRegion = PROT_CPUSS_SL_CTL_GROUP;
     ppcAttribute.endRegion = PROT_MCPASS;
     ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     /* Address: 0x42008000 - 0x4200FFFF */
     ppcAttribute.startRegion = PROT_PERI0_TR;
     ppcAttribute.endRegion = PROT_PERI0_TR;
     ppcStatus = Cy_Ppc_ConfigAttrib(ppcPtr, &ppcAttribute);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     /* Set PC Mask */
     pcMaskConfig.startRegion = PROT_PERI0_MAIN;
     pcMaskConfig.endRegion = PROT_MCPASS;
     pcMaskConfig.pcMask = 0xFFFFFFFF;
     ppcStatus = Cy_Ppc_SetPcMask(ppcPtr, &pcMaskConfig);
     if (ppcStatus != CY_PPC_SUCCESS)
     {
          return ppcStatus;
     }

     return CY_PPC_SUCCESS;
}
/*******************************************************************************
* Function Name: config_set_cm33_ns_pc
********************************************************************************
* Summary:
* This is the config_set_cm33_ns_pc function.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

cy_en_ms_ctl_status_t config_set_cm33_ns_pc(void)
{
     cy_en_ms_ctl_status_t mscStatus = CY_MS_CTL_FAILURE;

     mscStatus = Cy_Ms_Ctl_SetSavedPC(CPUSS_MS_ID_CM33_0, CM33_NS_PC_VALUE);
     if (mscStatus == CY_MS_CTL_SUCCESS)
     {
          mscStatus = Cy_Ms_Ctl_SetActivePC(CPUSS_MS_ID_CM33_0, CM33_NS_PC_VALUE);
     }
     return mscStatus;
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
     cy_rslt_t result = 0xFFFFFFFF;
     uint32_t ns_stack = 0;
     funcptr_void NonSecure_ResetHandler = NULL;
     cy_en_ms_ctl_status_t ms_ctl_status = CY_MS_CTL_FAILURE;
     cy_en_ppc_status_t ppc_status = CY_PPC_FAILURE;

     /* Initialize the device and board peripherals */
     result = cybsp_init();
     if (result != CY_RSLT_SUCCESS)
     {
          CY_ASSERT(0);
     }

     /* Configure SAU for SRAM, Flash and Peri MMIO */
     TZ_SAU_Setup();
     /* IPC */
     config_ipc();
     /* Configure PPC */
     ppc_status = config_ppc();
     if (CY_PPC_SUCCESS != ppc_status)
     {
          CY_ASSERT(0);
     }

     /* Enable global interrupts */
     __enable_irq();

     ns_stack = (uint32_t)(*(uint32_t*)CM33_SP_STORE);
     __TZ_set_MSP_NS(ns_stack);

     NonSecure_ResetHandler = (funcptr_void)(*((uint32_t*)CM33_RESET_HANDLER_STORE));

     /* change pc value for cm33-ns */
     ms_ctl_status = config_set_cm33_ns_pc();
     if (CY_MS_CTL_SUCCESS != ms_ctl_status)
     {
          CY_ASSERT(0);
     }

     /* Start non-secure application */
     NonSecure_ResetHandler();

     /* Program should not reach here */
     for (;;)
     {

     }
}
/* [] END OF FILE */

