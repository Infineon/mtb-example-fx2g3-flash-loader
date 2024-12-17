/***************************************************************************//**
* \file usb_app.c
* \version 1.0
*
* C source file implementing USB Handling for flash loader application logic.
*
*******************************************************************************
* \copyright
* (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
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

#include "usb_app.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "spi.h"

uint32_t Ep0TestBuffer[1024U] __attribute__ ((aligned (32)));
extern uint32_t Ep0TestBuffer[1024U];
extern cy_en_smif_slave_select_t glSlaveSelect;

/*
 * Function: Cy_USB_HandleCtrlSetup()
 * Description: This function handles control command given to application.
 * Parameter: pApp, pMsg
 * return: void
 */
void
Cy_USB_HandleCtrlSetup (void *pApp, cy_stc_usbd_app_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    cy_en_usb_endp_dir_t endpDir = CY_USB_ENDP_DIR_INVALID;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    bool isReqHandled = false;
    uint8_t   reqType;
    uint8_t bmRequest, bRequest, bTarget;
    uint8_t fxFlashProg[] = {'F','X','1','0','P','R','O','G'};
    uint8_t busyStat = 0;
    uint16_t wValue, wIndex, wLength;
    uint16_t loopCnt = 250u;
    uint32_t  setupData0;
    uint32_t  setupData1;
    uint32_t spiAddress = 0;
    uint32_t sector = 0;
    uint32_t numPages = (CY_APP_SPI_MAX_USB_TRANSFER_SIZE) / (CY_SPI_FLASH_PAGE_SIZE);
    cy_en_flash_index_t glFlashMode = SPI_FLASH_0;
    cy_en_smif_status_t smifStatus = CY_SMIF_SUCCESS;

    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pApp;

    setupData0 = pMsg->data[0];
    setupData1 = pMsg->data[1];

    DBG_APP_TRACE("Cy_USB_HandleCtrlSetup\r\n");
    /* Decode the fields from the setup request. */
    bmRequest = (uint8_t)((setupData0 & CY_USB_BMREQUEST_SETUP0_MASK) >>
                           CY_USB_BMREQUEST_SETUP0_POS);
    bRequest =  (uint8_t)((setupData0 & CY_USB_BREQUEST_SETUP0_MASK) >>
                           CY_USB_BREQUEST_SETUP0_POS);
    wValue = (uint16_t)((setupData0 & CY_USB_WVALUE_SETUP0_MASK) >>
                         CY_USB_WVALUE_SETUP0_POS);
    wIndex = (uint16_t)((setupData1 & CY_USB_WINDEX_SETUP1_MASK) >>
                         CY_USB_WINDEX_SETUP1_POS);
    wLength = (uint16_t)((setupData1 & CY_USB_WLENGTH_SETUP1_MASK) >>
                          CY_USB_WLENGTH_SETUP1_POS);

    reqType = ((bmRequest & CY_USB_CTRL_REQ_TYPE_MASK) >>
                                                CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bmRequest & CY_USB_CTRL_REQ_RECIPENT_MASK);

    switch (reqType) {

        case CY_USB_CTRL_REQ_STD:
            DBG_APP_INFO("StdReq\r\n");
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)) {
                DBG_APP_INFO("SetFeatureReq: EndpHalt\r\n");
                endpDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                         (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt,
                                              ((uint32_t)wIndex & 0x7FUL),
                                               endpDir, true);
                Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }
            
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE)) {
                switch (wValue) {
                    case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
                        DBG_APP_INFO("SetFeature:CY_USB_FEATURE_DEVICE_REMOTE_WAKE\r\n");
                        Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    case CY_USB_FEATURE_U1_ENABLE:
                        DBG_APP_INFO("SetFeature:CY_USB_FEATURE_U1_ENABLE\r\n");
                        Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U2_ENABLE:
                        DBG_APP_INFO("SetFeature:CY_USB_FEATURE_U2_ENABLE\r\n");
                        Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    
                    default:
                    /* Unknown feature selector: Request will be stalled below. */
                    break;
                }
            }

            /* Handle FUNCTION_SUSPEND here */
            if ((bRequest == CY_USB_SC_SET_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) &&
                (wValue == 0x00)) {

                /* TODO: Send a queue Msg to set the link to U2 */
                Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }

            if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)) {
                DBG_APP_INFO("ClearFeatureReq\r\n");

                endpDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) :
                         (CY_USB_ENDP_DIR_OUT));
                Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt,
                                  ((uint32_t)wIndex & 0x7FUL), endpDir);
                Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt,
                                  ((uint32_t)wIndex & 0x7FUL), endpDir, false);
                Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt,
                                              ((uint32_t)wIndex & 0x7FUL),
                                              endpDir, false);
                Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                isReqHandled = true;
            }
            
            if ((bRequest == CY_USB_SC_CLEAR_FEATURE) &&
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE)) {
                switch (wValue) {
                    case CY_USB_FEATURE_DEVICE_REMOTE_WAKE:
                        DBG_APP_INFO("ClrFeature:CY_USB_FEATURE_DEVICE_REMOTE_WAKE\r\n");
                        Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        /* TBD NT Enablng LPM only for CV test */
                        DBG_APP_INFO("Enabling LPM\r\n");
                        Cy_USBD_LpmEnable(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U1_ENABLE:
                        DBG_APP_INFO("ClearFeature:CY_USB_FEATURE_U1_ENABLE\r\n");
                        Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;

                    case CY_USB_FEATURE_U2_ENABLE:
                        DBG_APP_INFO("ClearFeature:CY_USB_FEATURE_U2_ENABLE\r\n");
                        Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        isReqHandled = true;
                        break;
                    
                    default:
                        /*
                         * Unknown feature selector so dont handle here.
                         * just send stall.
                         */
                        DBG_APP_INFO("default of wValue\r\n");
                        isReqHandled = false;
                    break;
                }
            }

            /* Handle Microsoft OS String Descriptor request. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

                /* Make sure we do not send more data than requested. */
                if (wLength > glOsString[0]) {
                    wLength = glOsString[0];
                }

                DBG_APP_INFO("OSString\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                      glOsString, wLength);
                if (retStatus != CY_USBD_STATUS_SUCCESS) {
                    DBG_APP_INFO("SendEp0Fail\r\n");
                }
                isReqHandled = true;
            }

            break;
        
        case CY_USB_CTRL_REQ_VENDOR:

            /* Handle OS Compatibility and OS Feature requests */
            if (bRequest == MS_VENDOR_CODE) {
                /*
                 * this one is VENDOR request. As off now class and vendor
                 * request under fallback case statement.
                 */
                if (wIndex == 0x04) {
                    if (wLength > *((uint16_t *)glOsCompatibilityId)) {
                        wLength = *((uint16_t *)glOsCompatibilityId);
                    }
                    DBG_APP_INFO("OSCompat\r\n");
                    retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                 glOsCompatibilityId, wLength);
                    if (retStatus != CY_USBD_STATUS_SUCCESS) {
                        DBG_APP_INFO("SendEp0Fail\r\n");
                    }
                    isReqHandled = true;

                } else if (wIndex == 0x05) {

                    if (wLength > *((uint16_t *)glOsFeature)) {
                        wLength = *((uint16_t *)glOsFeature);
                    }
                    DBG_APP_INFO("OSFeature\r\n");
                    retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt,
                                                          glOsFeature, wLength);
                    if (retStatus != CY_USBD_STATUS_SUCCESS) {
                        DBG_APP_INFO("SendEp0Fail\r\n");
                    }
                    isReqHandled = true;
                }
            }

            DBG_APP_TRACE("bRequest = 0x%x\r\n",bRequest);
                  
            /* Handle SPI vendor commands */
            if (bRequest == FLASH_CMD_CHECK_SPI_SUPPORT)
            {
                DBG_APP_INFO("FLASH_CMD_CHECK_SPI_SUPPORT\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt, fxFlashProg, sizeof(fxFlashProg));
                if(retStatus == CY_USBD_STATUS_SUCCESS)
                {
                    DBG_APP_TRACE("Check flash prog done\r\n");
                    isReqHandled = true;
                }
            }

            /* Check memory busy status */
            if (bRequest == FLASH_CMD_CHECK_STATUS)
            {
                busyStat = Cy_SPI_IsMemBusy(SPI_FLASH_0);
                retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt, &busyStat, 1);

                ASSERT_NON_BLOCK(CY_USBD_STATUS_SUCCESS == retStatus, retStatus);
                if(retStatus == CY_USBD_STATUS_SUCCESS)
                {
                    DBG_APP_INFO("Check status done (%d)\r\n",busyStat);
                    isReqHandled = true;
                }
            }

            /* Write to SPI flash */
            if ((bRequest == FLASH_CMD_FLASH_WRITE) && (wLength == CY_APP_SPI_MAX_USB_TRANSFER_SIZE))
            {
                DBG_APP_INFO("FLASH_CMD_FLASH_WRITE\r\n");
                spiAddress = wIndex * (CY_APP_SPI_MAX_USB_TRANSFER_SIZE);
                glFlashMode = SPI_FLASH_0;
                DBG_APP_TRACE("SPI Write..Wlength=%d spiAddress = %d, wIndex=%d %d\r\n", wLength,spiAddress, wIndex, wValue);

                retStatus = Cy_USB_USBD_RecvEndp0Data(pAppCtxt->pUsbdCtxt, (uint8_t*)pAppCtxt->qspiWriteBuffer, wLength);
                if (retStatus == CY_USBD_STATUS_SUCCESS)
                {
                    /* Wait until receive DMA transfer has been completed. */
                    while ((!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) && (loopCnt--)) {
                        Cy_SysLib_DelayUs(10);
                    }
                    if (!Cy_USBD_IsEp0ReceiveDone(pAppCtxt->pUsbdCtxt)) {
                        DBG_APP_ERR("Receive timed out\r\n");
                        Cy_USB_USBD_RetireRecvEndp0Data(pAppCtxt->pUsbdCtxt);
                        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
                        return;
                    }
                }
                
                if(retStatus == CY_USBD_STATUS_SUCCESS)
                {
                    smifStatus = Cy_SPI_WriteOperation(spiAddress, (uint8_t*)pAppCtxt->qspiWriteBuffer, wLength,numPages,
                                                    glFlashMode);
                    ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == smifStatus,smifStatus);
                    DBG_APP_TRACE("numPages = %d.Program %d->%d done\r\n",numPages,spiAddress,spiAddress+(CY_APP_SPI_MAX_USB_TRANSFER_SIZE-1));
                    isReqHandled = true;
                }
            }
            /* Read from SPI flash */
            if ((bRequest == FLASH_CMD_FLASH_READ) && (wLength == CY_APP_SPI_MAX_USB_TRANSFER_SIZE))
            {               
                spiAddress = wIndex * (CY_APP_SPI_MAX_USB_TRANSFER_SIZE);
                glFlashMode = SPI_FLASH_0;
                
                DBG_APP_TRACE("SPI read..Windex=%d Wlength=%d spiAddress = %d\r\n",wIndex, wLength, spiAddress);
                ASSERT_NON_BLOCK(wLength <= MAX_BUFFER_SIZE, wLength);

                smifStatus = Cy_SPI_ReadOperation(spiAddress, pAppCtxt->qspiReadBuffer, wLength,
                                glFlashMode);
                ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == smifStatus,smifStatus);
                retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt, pAppCtxt->qspiReadBuffer, wLength);
                if (retStatus == CY_USBD_STATUS_SUCCESS)
                {
                    isReqHandled = true;
                }
            }

            /* Sector Erase to SPI flash */
            if (bRequest == FLASH_CMD_FLASH_SECTOR_ERASE)
            {
                DBG_APP_INFO("FLASH_CMD_FLASH_SECTOR_ERASE\r\n");
                sector = wIndex & 0xFF;
                spiAddress = sector * CY_APP_SPI_FLASH_ERASE_SIZE;
                {
                    if((Cy_SPI_SectorErase(SPI_FLASH_0, spiAddress) == CY_SMIF_SUCCESS) )
                    {
                        Cy_USBD_SendACkSetupDataStatusStage(pAppCtxt->pUsbdCtxt);
                        DBG_APP_TRACE("SPI Erase Done\r\n");
                        isReqHandled = true;
                    }
                    else
                    {
                        DBG_APP_ERR("SPI Erase failed\r\n");
                    }
                }
            }

            /* Read SPI ID */
            if (bRequest == FLASH_CMD_FLASH_GET_ID)
            {
                if(wIndex == 0)
                {
                    Cy_SPI_ReadID((uint8_t*)pAppCtxt->qspiReadBuffer,glFlashMode);
                }
				retStatus = Cy_USB_USBD_SendEndp0Data(pAppCtxt->pUsbdCtxt, (uint8_t*)pAppCtxt->qspiReadBuffer,CY_FLASH_ID_LENGTH);
                if (retStatus == CY_USBD_STATUS_SUCCESS)
                {
                    isReqHandled = true;
                }
            }

        break;

        default:
            DBG_APP_INFO("HandleCtrlSetup:Default\r\n");
            break;
            
    }
    if(!isReqHandled) {
        Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
    return;

}   /* end of function() */

/*
 * Function: Cy_USB_TaskHandler()
 * Description: This function handles events for device.
 * Parameter: pTaskParam
 * return: void
 */
#if FREERTOS_ENABLE
void 
Cy_USB_TaskHandler (void *pTaskParam)
#else
void 
Cy_USB_TaskHandler (void *pTaskParam, void* qMsg)
#endif /* FREERTOS_ENABLE */
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    cy_stc_usbd_app_msg_t queueMsg;
    pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;

    /* Enable USB-2 connection and wait until it is stable. */
    vTaskDelay(250);

#if FREERTOS_ENABLE
    BaseType_t xStatus;
    uint32_t idleLoopCnt = 0;

    Cy_SPI_Start(pAppCtxt,SPI_FLASH_0);
    DBG_APP_INFO("Flash Init \n\r:");

#if !FLASH_AT45D
    Cy_SPI_FlashInit(SPI_FLASH_0, false, false);
#endif /* !FLASH_AT45D */

    /* Enable USB-2 connection and wait until it is stable. */
    vTaskDelay(100);

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent =
    (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

    if (pAppCtxt->vbusPresent) {
        Cy_USB_ConnectionEnable(pAppCtxt);
    }

    DBG_APP_INFO("ThreadActive\r\n");

    do {
#if WATCHDOG_RESET_EN
        /* Kick The WDT to prevent RESET */
        KickWDT();
#endif /* WATCHDOG_RESET_EN */

#if FREERTOS_ENABLE

        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->xQueue, &queueMsg, 1);
        if (xStatus != pdPASS) {
            idleLoopCnt++;
            if (idleLoopCnt >= 10000UL) {
                idleLoopCnt = 0;
                DBG_APP_INFO("TaskIdle\r\n");
            }

            continue;
        }
        idleLoopCnt = 0;
        #endif
#else /* !FREERTOS_ENABLE */

        memcpy((uint8_t *)&queueMsg, (uint8_t *)qMsg,
                sizeof(cy_stc_usbd_app_msg_t));

         
#endif /* FREERTOS_ENABLE */

        /*
         * Make sure that the USB link is brought into active state
         * periodically to avoid stuck data transfers.
         */
        if ((Cy_USBD_GetTimerTick() & 0x1FF) == 0x1FF) {
            Cy_USBD_GetUSBLinkActive(pAppCtxt->pUsbdCtxt);
        }


        switch (queueMsg.type) {

            case CY_USB_MSG_CTRL_XFER_SETUP:
                DBG_APP_TRACE("CY_USB_MSG_CTRL_XFER_SETUP\r\n");
                Cy_USB_HandleCtrlSetup((void *)pAppCtxt, &queueMsg);
                break;


            case CY_USB_ENDP0_READ_TIMEOUT:
                DBG_APP_INFO("Endp0ReadTimeout\r\n");
                /*
                 * When application layer wants to recieve data from
                 * host through endpoint 0 then device initiate RcvEndp0
                 * function call and start timer. When timer ends and still
                 * data is not recieved then TIMER interrupt will send
                 * CY_USB_ENDP0_READ_TIMEOUT message. If data is recieved then
                 * case which handles data should stop timer.
                 */
                Cy_USB_USBD_RetireRecvEndp0Data(pAppCtxt->pUsbdCtxt);
                break;


            default:
                DBG_APP_ERR("Default %d\r\n", queueMsg.type);
                break;
        }   /* end of switch() */
    
#if FREERTOS_ENABLE
    } while (1);
#endif /* FREERTOS_ENABLE */
}   /* End of function  */





/*
 * Function: Cy_USB_Endp0ReadComplete()
 * Description:  Handler for DMA transfer completion on endpoint 0 OUT Transfer.
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: void
 */
void
Cy_USB_Endp0ReadComplete (void *pApp)
{
    BaseType_t status;
    DBG_APP_TRACE("Sent CY_USB_ENDP0_READ_COMPLETE\r\n");

    (void)status;
    return;
}   /* end of function */


/*
 * Function: Cy_USB_AppInit()
 * Description: This function Initializes application related data structures,
 *              register callback and creates queue and task for device
 *              function. Common function for FS/HS.
 * Parameter: cy_stc_usb_app_ctxt_t, cy_stc_usb_usbd_ctxt_t, DMAC_Type
 *            DW_Type, DW_Type, cy_stc_hbdma_mgr_context_t*
 * return: None.
 * Note: This function should be called after USBD_Init()
 */
void
Cy_USB_AppInit (cy_stc_usb_app_ctxt_t *pAppCtxt,
                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
                DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base)
{
   
#if FREERTOS_ENABLE
    BaseType_t status = pdFALSE;
#endif /* FREERTOS_ENABLE */
    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;

    DBG_APP_INFO("Cy_USB_AppInit\r\n");
    /*
     * Initially application sees device speed as USBFS and during set
     * configuration application will update actual device speed.
     */
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->currentAltSetting = 0x00;
    pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone)) {
       

#if FREERTOS_ENABLE
        /* create queue and register it to kernel. */
        pAppCtxt->xQueue = xQueueCreate(CY_USB_MSG_QUEUE_SIZE,
                                        CY_USB_MSG_SIZE);
        DBG_APP_INFO("CreatedQueue\r\n");
        vQueueAddToRegistry(pAppCtxt->xQueue, "MsgQueue");

        /* Create task and check status to confirm task created properly. */
        status = xTaskCreate(Cy_USB_TaskHandler, "Task", 2048,
                        (void *)pAppCtxt, 5, &(pAppCtxt->taskHandle));
        if (status != pdPASS) {
            DBG_APP_ERR("TaskcreateFail\r\n");
            return;
        }
#else
        Cy_USB_ConnectionEnable(pAppCtxt);
#endif /* FREERTOS_ENABLE */
        pAppCtxt->firstInitDone = 0x01;
        
    }


    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppRegisterCallback()
 * Description: This function will register all calback with USBD layer. 
 * Parameter: cy_stc_usb_app_ctxt_t.
 * return: None.
 */
void
Cy_USB_AppRegisterCallback (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET, 
                                               Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP, 
                                                  Cy_USB_AppSetupCallback);
   
    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppBusResetCallback()
 * Description: This Function will be called by USBD when bus detects RESET.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void 
Cy_USB_AppBusResetCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("ResetCallback >>\r\n");
    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase,
                   pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;

    DBG_APP_INFO("ResetCallback <<\r\n\r\n");
    return;
}   /* end of function. */



/*
 * Function: Cy_USB_AppSetupCallback()
 * Description: This Function will be called by USBD  layer when 
 *              set configuration command successful. This function 
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
volatile uint16_t pktType = 0;
volatile uint16_t pktLength = 1024;

void 
Cy_USB_AppSetupCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
#if FREERTOS_ENABLE
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t status;

    xMsg.type = CY_USB_MSG_CTRL_XFER_SETUP;
    xMsg.data[0] = pMsg->data[0];
    xMsg.data[1] = pMsg->data[1];

#if FREERTOS_ENABLE
    status = xQueueSendFromISR(pUsbApp->xQueue, &(xMsg),
                               &(xHigherPriorityTaskWoken));
#else
    Cy_USB_TaskHandler(pAppCtxt, &xMsg);
#endif /* FREERTOS_ENABLE */
    (void)status;

}   /* end of function. */

void checkStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

void checkStatusAndHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)(void))
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);

        if(failureHandler != NULL)
        {
            (*failureHandler)();
        }


        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

