/***************************************************************************//**
* \file usb_app.h
* \version 1.0
*
* \details Header file for Application Data structures and functions declaration..
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Macros */
#define RED                             "\033[0;31m"
#define CYAN                            "\033[0;36m"
#define COLOR_RESET                     "\033[0m"

#define ASSERT(condition, value)        Cy_CheckStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value) Cy_CheckStatus(__func__, __LINE__, condition, value, false);
#define ASSERT_AND_HANDLE(condition, value, failureHandler) Cy_CheckStatusHandleFailure(__func__, __LINE__, condition, value, false, Cy_FailHandler);

#define CY_USB_MSG_CTRL_XFER_SETUP          (0x0A)
#define CY_USB_ENDP0_READ_TIMEOUT           (0x0D)
#define CY_USB_VBUS_CHANGE_INTR             (0x0E)
#define CY_USB_VBUS_CHANGE_DEBOUNCED        (0x0F)

#define CY_USB_MSG_QUEUE_SIZE               (16)
#define CY_USB_MSG_SIZE                     (sizeof (cy_stc_usbd_app_msg_t))

#define VBUS_DETECT_GPIO_PORT                (P4_0_PORT) /* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PIN                 (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR                (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE                    (0u)

#define CY_APP_SPI_MAX_USB_TRANSFER_SIZE    (0x800)

#define USB_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32)))
#define HBDMA_BUF_ATTRIBUTES __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32)))

#define MS_VENDOR_CODE                    (0xF0) /* Vendor command code used to return WinUSB specific descriptors. */

extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* Structs */
/*
 * USB application data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains information about device functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    bool vbusChangeIntr;
    uint8_t firstInitDone;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    uint8_t currentAltSetting;
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    cy_en_usb_enum_method_t enumMethod;

    /* Next three are related to central DMA */
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;

    /* Global Task handles */
    TaskHandle_t taskHandle;
    QueueHandle_t xQueue;
    TimerHandle_t vbusDebounceTimer;

    /* VBus detect status */
    bool vbusPresent;
    bool usbConnected;    

    uint8_t *qspiWriteBuffer;
    uint8_t *qspiReadBuffer;
    uint8_t glpassiveSerialMode;
};

/**
 * \typedef cy_en_fx10_fpga_config_type_t
 * \brief Enum for FPGA Configuration
 */
typedef enum cy_en_fx10_fpga_config_type_t
{
    FPGA_PASSIVE_SERIAL_x4 = 1,
    FPGA_PASSIVE_SERIAL_x8,
    FPGA_CONFIG_INVALID = 0xFF,

}cy_en_fx10_fpga_config_type_t;

/* Function Prototypes */
/**
 * \name Cy_USB_TaskHandler
 * \brief This function handles events for device.
 * \param pTaskParam Task param
 * \retval None
 */
void Cy_USB_TaskHandler(void *pTaskParam);

/**
 * \name Cy_USB_Endp0ReadComplete
 * \brief Handler for DMA transfer completion on endpoint 0 OUT Transfer.
 * \param pApp application layer context pointer
 * \retval None
 */
void Cy_USB_Endp0ReadComplete(void *pApp);

/**
 * \name Cy_USB_AppRegisterCallback
 * \brief This function will register all calback with USBD layer.
 * \param pAppCtxt application layer context pointer
 * \return None
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_AppBusResetCallback
 * \brief This Function will be called by USBD when bus detects RESET.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer.
 * \retval None
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSetupCallback
 * \brief Callback function will be invoked by USBD when SETUP packet is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetupCallback(void *pAppCtxt, 
                             cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_ConnectionEnable
 * \brief Function used to enable USB connection.
 * \param pAppCtxt Application context structure.
 * \retval None
 */
bool Cy_USB_ConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_ConnectionDisable
 * \brief Function which disables the USB connection.
 * \param pAppCtxt Application context structure pointer.
 * \retval None
 */
void Cy_USB_ConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_AppInit
 * \details This function Initializes application related data structures,
 *          register callback and creates queue and task for device
 *          function. Common function for FS/HS.
 * \param pAppCtxt application layer conetxt pointer
 * \param pUsbdCtxt USBD layer conetxt pointer
 * \param pCpuDmacBase DMA controller base address pointer
 * \param pCpuDw0Base Datawire 0 base address pointer
 * \param pCpuDw1Base Datawire 1 base address pointer
 * \retval None
 */
void Cy_USB_AppInit (cy_stc_usb_app_ctxt_t *pAppCtxt,
                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
                DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base);

/**
 * \name Cy_CheckStatus
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line Line number where error is seen
 * \param condition condition of failure
 * \param value error code
 * \param isBlocking blocking function
 * \return None
 */
void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, 
                    uint32_t value, uint8_t isBlocking);

/**
 * \name Cy_CheckStatusHandleFailure
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line LineNumber where error is seen
 * \param condition Line number where error is seen
 * \param value error code
 * \param isBlocking blocking function
 * \param failureHandler failure handler function
 * \return None
 */
void Cy_CheckStatusHandleFailure(const char *function, uint32_t line, uint8_t condition, 
                        uint32_t value, uint8_t isBlocking, void (*failureHandler)());

/**
 * \name Cy_FailHandler
 * \brief Error Handler
 * \retval None
 */
void Cy_FailHandler(void);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */
