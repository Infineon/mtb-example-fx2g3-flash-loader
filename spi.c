/***************************************************************************//**
* \file spi.c
* \version 1.0
*
* C source file with SMIF handlers.
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

#include "spi.h"
#include "usb_app.h"

/* SMIF context*/
cy_stc_smif_context_t spiContext;
cy_en_smif_txfr_width_t   glCommandWidth[NUM_SPI_FLASH]     = {CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_QUAD};
cy_en_smif_txfr_width_t   glReadWriteWidth[NUM_SPI_FLASH]   = {CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_OCTAL};
uint8_t glSlaveSelectIndex[NUM_SPI_FLASH] = {CY_SMIF_SLAVE_SELECT_0, CY_SMIF_SLAVE_SELECT_1, (CY_SMIF_SLAVE_SELECT_0 | CY_SMIF_SLAVE_SELECT_1)};
cy_en_flash_index_t glFlashMode = SPI_FLASH_0;
#if !FLASH_AT45D
static cy_stc_cfi_flash_map_t glCfiFlashMap[NUM_SPI_FLASH];
#endif /* !FLASH_AT45D */

HBDMA_BUF_ATTRIBUTES uint8_t readBuffer[MAX_BUFFER_SIZE];
HBDMA_BUF_ATTRIBUTES uint8_t writeBuffer[MAX_BUFFER_SIZE];

static const cy_stc_smif_config_t spiConfig =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,                       /* Normal mode operation */
    .deselectDelay = 0u,                                    /* Minimum de-select time */
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INV_INTERNAL_CLK,   /* Source selection for receiver clock. MISO is sampled on rising edge of this clock */ 
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR
};

/* Convert the provided 32-bit value to an array */
static cy_en_smif_status_t Cy_SPI_AddressToArray(uint32_t value, uint8_t *byteArray, uint8_t size)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    if(byteArray == NULL)
    {
      return CY_SMIF_BAD_PARAM;
    }
    do
    {
      size--;
      byteArray[size] = (uint8_t)(value & 0x000000FF);
      value >>= 8U; /* Shift to get the next byte */
    } while (size > 0U);

    return status;
}

/* SPI Write enable*/
static cy_en_smif_status_t Cy_SPI_WriteEnable(cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
#if !FLASH_AT45D
    uint8_t statusVal = 0;
#endif /* !FLASH_AT45D */
    
    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_WRITE_ENABLE_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_LAST_BYTE,
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

#if !FLASH_AT45D
    /* Check if WRITE_ENABLE LATCH is set */
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_STATUS_READ_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW,
            &statusVal,
            1u,
            glReadWriteWidth[flashIndex],
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if(statusVal & CY_SPI_WRITE_ENABLE_LATCH_MASK)
    {
        status = CY_SMIF_SUCCESS;
        DBG_APP_TRACE("Write Enable Passed\r\n");
    }
    else
    {
        status = CY_SMIF_BUSY;
        DBG_APP_ERR("Write Enable failed\r\n");
    }
#endif /* !FLASH_AT45D */

    return status;
}

/* Configure SMIF pins*/
static cy_en_gpio_status_t Cy_SPI_ConfigureSMIFPins(bool init)
{
    cy_en_gpio_status_t status = CY_GPIO_SUCCESS;
    cy_stc_gpio_pin_config_t pinCfg;

    memset((void *)&pinCfg, 0, sizeof(pinCfg));

    if(init)
    {
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        /* Configure P6.0 as SPI Clock */
        pinCfg.hsiom = SMIF_CLK_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P6.0 as floating GPIO */
        pinCfg.hsiom = P6_0_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_CLK_PORT, SMIF_CLK_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        /* Configure P6.1 as SPI Select 0 */
        pinCfg.outVal = 1;
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom = SMIF_SELECT0_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P6.1 as floating GPIO */
        pinCfg.hsiom = P6_1_GPIO;
    }

    status = Cy_GPIO_Pin_Init(SMIF_SELECT0_PORT, SMIF_SELECT0_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        /* Configure P6.2 as QSPI Select 1 */
        pinCfg.outVal = 1;
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom = SMIF_SELECT1_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P6.2 as floating GPIO */
        pinCfg.hsiom = P6_2_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_SELECT1_PORT, SMIF_SELECT1_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.0 as SPI Data 0 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA0_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.0 as floating GPIO */
        pinCfg.hsiom = P7_0_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA0_PORT, SMIF_DATA0_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.1 as SPI Data 1 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA1_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.1 as floating GPIO */
        pinCfg.hsiom = P7_1_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA1_PORT, SMIF_DATA1_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.2 as SPI Data 2*/
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA2_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.2 as floating GPIO */
        pinCfg.hsiom = P7_2_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA2_PORT, SMIF_DATA2_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.3 as SPI Data 3*/
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA3_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.3 as floating GPIO */
        pinCfg.hsiom = P7_3_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA3_PORT, SMIF_DATA3_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.4 as QSPI Data 4 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA4_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.4 as floating GPIO */
        pinCfg.hsiom = P7_4_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA4_PORT, SMIF_DATA4_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.5 as QSPI Data 5 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA5_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.5 as floating GPIO */
        pinCfg.hsiom = P7_5_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA5_PORT, SMIF_DATA5_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.6 as QSPI Data 6 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA6_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.6 as floating GPIO */
        pinCfg.hsiom = P7_6_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA6_PORT, SMIF_DATA6_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.7 as QSPI Data 7 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA7_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.7 as floating GPIO */
        pinCfg.hsiom = P7_7_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA7_PORT, SMIF_DATA7_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);
    return status;
}

/* Function to read device ID */
cy_en_smif_status_t Cy_SPI_ReadID(uint8_t *rxBuffer, cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_READ_ID_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW, rxBuffer, CY_FLASH_ID_LENGTH, glReadWriteWidth[flashIndex], &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

/* Function to check busy status of flash */
bool Cy_SPI_IsMemBusy(cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t statusVal;

    /* Send status register read command */
    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_STATUS_READ_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &spiContext);

    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW,
            &statusVal,
            1u,
            glReadWriteWidth[flashIndex],
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    DBG_APP_TRACE("Cy_SPI_IsMemBusy %x\r\n",statusVal);

    return ((statusVal & CY_SPI_WIP_MASK) == CY_SPI_WIP_STATUS);
}


/* Function to enable SPI block */
cy_en_smif_status_t Cy_SPI_Start(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint32_t programWait = 0;

    pAppCtxt->qspiWriteBuffer = writeBuffer;
    pAppCtxt->qspiReadBuffer = readBuffer;

    memset(pAppCtxt->qspiWriteBuffer, 0, MAX_BUFFER_SIZE);
    memset(pAppCtxt->qspiReadBuffer, 0, MAX_BUFFER_SIZE);
 
    /* SPI is connected to CLK_HF1. As per current clock configuration in Cy_Fx2g3_clk_init, CLK_HF1 is connected Clock path #1 (PLL#0) at 150 MHz  */
    Cy_SysClk_ClkHfDisable(CY_SYSCLK_SPI_CLK_HF1);
    Cy_SysClk_ClkHfSetSource(CY_SYSCLK_SPI_CLK_HF1, CY_SYSCLK_CLKHF_IN_CLKPATH1);

    /* Selected SPI Clock = 150M/DIVIDER */
    Cy_SysClk_ClkHfSetDivider(CY_SYSCLK_SPI_CLK_HF1, CY_SYSCLK_CLKHF_DIVIDE_BY_4);
    Cy_SysClk_ClkHfEnable(CY_SYSCLK_SPI_CLK_HF1);
    Cy_SPI_ConfigureSMIFPins(true);

    status = Cy_SMIF_Init(SMIF_HW, &spiConfig, 10000u, &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_1, CY_SMIF_DATA_SEL2);
    
    Cy_SMIF_Enable(SMIF_HW, &spiContext);

    while(Cy_SPI_IsMemBusy(flashIndex)) { 
        if(programWait++ >= CY_SPI_PROGRAM_TIMEOUT_US) 
        { 
            status = CY_SMIF_EXCEED_TIMEOUT; 
            DBG_APP_ERR("Error: Program Timeout\r\n"); 
            break; 
        } 
    }

    DBG_APP_INFO("SPI Initialization Done\r\n");
    DBG_APP_INFO("SPI Clock = %d\r\n",Cy_SysClk_ClkHfGetFrequency(CY_SYSCLK_SPI_CLK_HF1));
    return status;
}

/* Function to disable SPI block */
cy_en_smif_status_t Cy_SPI_Stop(void)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    Cy_SysClk_ClkHfDisable(CY_SYSCLK_SPI_CLK_HF1);
    Cy_SPI_ConfigureSMIFPins(false);
    Cy_SMIF_Disable(SMIF_HW);
    return status;
}

#if !FLASH_AT45D

/*
Function     : Cy_SPI_FlashReset ()
Description  : Send reset command to selected flash.   
Parameters  :  cy_en_flash_index_t flashIndex 
Return      :  cy_en_smif_status_t  

*/
static cy_en_smif_status_t Cy_SPI_FlashReset(cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_RESET_ENABLE_CMD,
            CY_SMIF_WIDTH_QUAD,
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_LAST_BYTE,
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_SW_RESET_CMD,
            CY_SMIF_WIDTH_QUAD,
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_LAST_BYTE,
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    /*tRPH delay SFS256 Flash*/
    Cy_SysLib_DelayUs(50);

    return status;
}

/*
 * Function     :   Cy_SPI_ReadCFIMap  
 * Description  :   Read Common Flash Interface (CFI) Table
 * Parameters   :   cy_stc_cfi_flash_map_t *cfiFlashMap, cy_en_flash_index_t flashIndex
 * Return       :   cy_en_smif_status_t  
 */

static cy_en_smif_status_t Cy_SPI_ReadCFIMap (cy_stc_cfi_flash_map_t *cfiFlashMap, cy_en_flash_index_t flashIndex)
{
    uint8_t sectorIndex = 0;
    uint8_t eraseRegionIndex = 0;
    uint8_t rxBuffer[CY_CFI_TABLE_LENGTH] = {0};
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_READ_ID_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW, rxBuffer, CY_CFI_TABLE_LENGTH, glReadWriteWidth[flashIndex], &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    cfiFlashMap->deviceSizeFactor = rxBuffer[CY_CFI_DEVICE_SIZE_OFFSET];
    cfiFlashMap->deviceSize = (uint32_t)(1u << cfiFlashMap->deviceSizeFactor);
    DBG_APP_INFO("DeviceSize = 0x%x[%d]\r\n", (cfiFlashMap->deviceSize), (cfiFlashMap->deviceSize));

    /* Parse the CFI buffer and understand possible memory array layouts */
    cfiFlashMap->numEraseRegions = rxBuffer[CY_CFI_NUM_ERASE_REGION_OFFSET];
    DBG_APP_INFO("Number of erase regions = %d\r\n", cfiFlashMap->numEraseRegions);

    if(cfiFlashMap->numEraseRegions < CY_CFI_TABLE_LENGTH)
    {
        //The part has multiple erase layouts, possibly because it supports hybrid layout
        for(eraseRegionIndex = 0 , sectorIndex = 0;
                eraseRegionIndex < (cfiFlashMap->numEraseRegions);
                eraseRegionIndex++, sectorIndex += CY_CFI_ERASE_REGION_SIZE_INFO_SIZE)
        {
            cfiFlashMap->memoryLayout[eraseRegionIndex].numSectors = 1 + (rxBuffer[sectorIndex + CY_CFI_ERASE_NUM_SECTORS_OFFSET] |
                    (rxBuffer[sectorIndex + CY_CFI_ERASE_NUM_SECTORS_OFFSET + 1] << 8));

            cfiFlashMap->memoryLayout[eraseRegionIndex].sectorSize = 256 * (rxBuffer[sectorIndex + CY_CFI_ERASE_SECTOR_SIZE_OFFSET] |
                    (rxBuffer[sectorIndex + CY_CFI_ERASE_SECTOR_SIZE_OFFSET + 1] << 8));
            if(eraseRegionIndex)
            {
                cfiFlashMap->memoryLayout[eraseRegionIndex].startingAddress = (cfiFlashMap->memoryLayout[eraseRegionIndex - 1].startingAddress +
                        cfiFlashMap->memoryLayout[eraseRegionIndex - 1].sectorSize *
                        cfiFlashMap->memoryLayout[eraseRegionIndex - 1].numSectors);
            }
            else
            {
                cfiFlashMap->memoryLayout[eraseRegionIndex].startingAddress = 0;
            }

            cfiFlashMap->memoryLayout[eraseRegionIndex].lastAddress = cfiFlashMap->memoryLayout[eraseRegionIndex].startingAddress +
                (cfiFlashMap->memoryLayout[eraseRegionIndex].numSectors * cfiFlashMap->memoryLayout[eraseRegionIndex].sectorSize) - 1;

            if(cfiFlashMap->memoryLayout[eraseRegionIndex].sectorSize == 0x1000)
            {
                cfiFlashMap->num4KBParameterRegions++;
            }

            DBG_APP_INFO("Erase region:%d, numSectors=%d, sectorSize=0x%x, startingAddress=0x%x\r\n",eraseRegionIndex,
                    cfiFlashMap->memoryLayout[eraseRegionIndex].numSectors,
                    cfiFlashMap->memoryLayout[eraseRegionIndex].sectorSize,
                    cfiFlashMap->memoryLayout[eraseRegionIndex].startingAddress);
        }
    }
    return status;
}

/* Initialize the SPI Flash - 
 * Quad Mode - Data in x4 mode, Command in x1 mode
 * QPI Mode - Data in x4 mode, Command in x4 mode
 *
 * QPI enabled implies Quad enable.
 *
 * Enable only Quad mode when writes to flash can be in x1 mode and only reads need to be in x4 mode (eg: Passive x4 mode with one x4 flash memory)
 * Enable QPI mode when writes and reads should be in x4 mode (eg: Passive x8 mode with two x4 flash memories)
 * */
cy_en_smif_status_t Cy_SPI_FlashInit (cy_en_flash_index_t flashIndex, bool quadEnable, bool qpiEnable)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t flashID[CY_FLASH_ID_LENGTH]={0};

    status = Cy_SPI_FlashReset(flashIndex);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SPI_ReadCFIMap(&glCfiFlashMap[flashIndex], flashIndex);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    Cy_SPI_ReadID(flashID, flashIndex);
    
    return status;
}

/*
Function     : Cy_SPI_UniformSectorErase ()
Description :  Send the uniform sector erase command for all non-4KB sectors. 
               Note that this command has no effect on 4KB-sized sectors.
Parameters  :  cy_en_flash_index_t flashIndex, uint32_t address 
Return      :  cy_en_smif_status_t  

*/

static cy_en_smif_status_t Cy_SPI_UniformSectorErase(cy_en_flash_index_t flashIndex, uint32_t address)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    uint8_t addrArray[SPI_ADDRESS_BYTE_COUNT];
    Cy_SPI_AddressToArray(address, addrArray, SPI_ADDRESS_BYTE_COUNT);

    status = Cy_SPI_WriteEnable(flashIndex);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if (status == CY_SMIF_SUCCESS)
    {
        status =  Cy_SMIF_TransmitCommand(SMIF_HW,
                CY_SPI_SECTOR_ERASE_CMD,
                glCommandWidth[flashIndex],
                addrArray,
                SPI_ADDRESS_BYTE_COUNT,
                glCommandWidth[flashIndex],
                (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
                CY_SMIF_TX_LAST_BYTE,
                &spiContext);

        DBG_APP_INFO("Uniform sector erase from 0x%x\r\n", address);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    }
    return status;
}

/*
Function     : Cy_SPI_HybridSectorErase ()
Description :  Send hybrid sector erase command for the 4KB sectors. 
Parameters  :  cy_en_flash_index_t flashIndex, uint32_t address 
Return      :  cy_en_smif_status_t  

*/

static cy_en_smif_status_t Cy_SPI_HybridSectorErase(cy_en_flash_index_t flashIndex, uint32_t address)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t addrArray[SPI_ADDRESS_BYTE_COUNT];
    
    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }
    Cy_SPI_AddressToArray(address, addrArray, SPI_ADDRESS_BYTE_COUNT);
    
    status = Cy_SPI_WriteEnable(flashIndex);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    
    if (status == CY_SMIF_SUCCESS)
    {
        status =  Cy_SMIF_TransmitCommand(SMIF_HW,
                CY_SPI_HYBRID_SECTOR_ERASE_CMD,
                glCommandWidth[flashIndex],
                addrArray,
                SPI_ADDRESS_BYTE_COUNT,
                glCommandWidth[flashIndex],
                (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
                CY_SMIF_TX_LAST_BYTE,
                &spiContext);
        DBG_APP_TRACE("4KB region erase from 0x%x\r\n", address);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    }
    return status;
}
#endif

/* Function to erase flash sector */
cy_en_smif_status_t Cy_SPI_SectorErase(cy_en_flash_index_t flashIndex, uint32_t address)
{
#if FLASH_AT45D
    /* Skip erase and return. Erase is performed by write command */
    return CY_SMIF_SUCCESS;
#else
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint32_t index = 0;
    uint32_t hybridIndex = 0;
    uint32_t numEraseRegions = glCfiFlashMap[flashIndex].numEraseRegions;
    uint32_t programWait = 0;

    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    if(address >= glCfiFlashMap[flashIndex].deviceSize)
    {
        DBG_APP_ERR("[%s]Invalid Address.\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    /* Check if memory has any 4KB sector regions */
    if(glCfiFlashMap[flashIndex].num4KBParameterRegions)
    {
        for(index = 0; index < numEraseRegions; index++)
        {

            /* Check if the address to be erased is in a 4KB Hybrid region */
            if((address >= glCfiFlashMap[flashIndex].memoryLayout[index].startingAddress) && (address <= glCfiFlashMap[flashIndex].memoryLayout[index].lastAddress)
                    && (glCfiFlashMap[flashIndex].memoryLayout[index].sectorSize == 0x1000))
            {
                /* The address is present in a 4 KB sector region. Erase the entire 4KB region. */
                for(hybridIndex = 0; hybridIndex < glCfiFlashMap[flashIndex].memoryLayout[index].numSectors; hybridIndex++)
                {
                    status = Cy_SPI_HybridSectorErase(flashIndex, address + (hybridIndex * 0x1000));
                    if(status == CY_SMIF_SUCCESS)
                    {
                        programWait = 0;
                        while(Cy_SPI_IsMemBusy(flashIndex))
                        {
                            if(programWait++ >= CY_SPI_PROGRAM_TIMEOUT_US)
                            {
                                status =  CY_SMIF_EXCEED_TIMEOUT;
                                DBG_APP_ERR(" Cy_SPI_HybridSectorErase TIMEOUT!! %x\r\n",status);
                                break;
                            }
                            else
                            {
                                Cy_SysLib_DelayUs(1);
                            }
                        }
                    }
                    else
                    {
                        DBG_APP_ERR(" Cy_SPI_HybridSectorErase failed %x\r\n",status);
                    }
                }
            }
        }
    }

    /* Do a uniform sector erase command to erase all non-4KB sector areas. This command has no effect on the 4KB regions. */
    status = Cy_SPI_UniformSectorErase(flashIndex, address);
    programWait = 0;
    while(Cy_SPI_IsMemBusy(flashIndex))
    {
        if(programWait++ >= CY_SPI_PROGRAM_TIMEOUT_US)
        {
            status =  CY_SMIF_EXCEED_TIMEOUT;
            DBG_APP_ERR(" Cy_SPI_HybridSectorErase TIMEOUT!! %x\r\n",status);
            break;
        }
        else
        {
            Cy_SysLib_DelayUs(1);
        }
    }
    return status;
#endif
}

/* Write to Flash page*/
cy_en_smif_status_t Cy_SPI_WritePage(uint32_t address, uint8_t *txBuffer, cy_en_flash_index_t flashIndex)
{
    uint8_t addrArray[SPI_ADDRESS_BYTE_COUNT]; 
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint32_t programWait = 0;

#if FLASH_AT45D
    Cy_SPI_AddressToArray(address, addrArray, SPI_ADDRESS_BYTE_COUNT-1);  /* Convert address to 3 bytes*/ 
    status = Cy_SPI_WriteEnable(flashIndex); 
    if(status == CY_SMIF_SUCCESS)
    {
        /* Step 1: Write to Buffer 1 using command 0x84 */
        status = Cy_SMIF_TransmitCommand(
            SMIF_HW,                                                                    /* SMIF instance */
            CY_SPI_PROGRAM_CMD,                                                         /* Buffer 1 Write command */
            glCommandWidth[flashIndex],                                                 /* Command transfer width */
            addrArray,                                                                  /* Address array */
            SPI_ADDRESS_BYTE_COUNT-1,                                                   /* Address array size (3 bytes) */
            glCommandWidth[flashIndex],                                                 /* Address transfer width */
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],                  /* Slave select */
            CY_SMIF_TX_NOT_LAST_BYTE,                                                   /* Not the last byte yet */
            &spiContext                                                                 /* Context */
        );

        if (status != CY_SMIF_SUCCESS)
        {
            DBG_APP_ERR("Error: Buffer1 WR cmd status:0x%x\r\n", status);
            return status;
        }

        status = Cy_SMIF_TransmitDataBlocking(
            SMIF_HW, 
            txBuffer, 
            CY_SPI_FLASH_PAGE_SIZE, 
            glReadWriteWidth[flashIndex], 
            &spiContext
        );

        if (status != CY_SMIF_SUCCESS)
        {
            DBG_APP_ERR("Error: Buffer1 WR data cmd status:0x%x\r\n", status);
            return status;
        }
        
        Cy_SysLib_DelayUs(10);

        /* Step 2: Transfer Buffer 1 to Main Memory */
        status = Cy_SMIF_TransmitCommand(
            SMIF_HW,                                                                    /* SMIF instance */
            CY_SPI_PROGRAM_CMD_1,                                                       /* Buffer 1 to Main Memory Page Program With Built-In Erase command */
            glCommandWidth[flashIndex],                                                 /* Command transfer width */
            addrArray,                                                                  /* Address array */
            SPI_ADDRESS_BYTE_COUNT-1,                                                   /* Address array size (3 bytes) */
            glCommandWidth[flashIndex],                                                 /* Address transfer width */
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],                  /* Slave select */ 
            CY_SMIF_TX_LAST_BYTE,                                                       /* Last byte */
            &spiContext                                                                 /* Context */
        );
        if (status != CY_SMIF_SUCCESS)
        {
            DBG_APP_ERR("Error: Buffer 1 to Main Memory cmd status:0x%x\r\n", status);
            return status;
        }

        while(Cy_SPI_IsMemBusy(flashIndex))
        {
            if(programWait++ >= CY_SPI_PROGRAM_TIMEOUT_US)
            {
                status = CY_SMIF_EXCEED_TIMEOUT;
                DBG_APP_ERR("Error: Program Timeout\r\n");
                break;
            }
        }
    }
#else
    Cy_SPI_AddressToArray(address, addrArray, SPI_ADDRESS_BYTE_COUNT);                    /* Convert address to 3 bytes*/ 
    status = Cy_SPI_WriteEnable(flashIndex); 

    if(status == CY_SMIF_SUCCESS)
    {
        status = Cy_SMIF_TransmitCommand(SMIF_HW,
                CY_SPI_PROGRAM_CMD,
                glCommandWidth[flashIndex],
                addrArray,
                SPI_ADDRESS_BYTE_COUNT,
                glCommandWidth[flashIndex],
                (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
                CY_SMIF_TX_NOT_LAST_BYTE,
                &spiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
        if(status == CY_SMIF_SUCCESS)
        {
            status = Cy_SMIF_TransmitDataBlocking(SMIF_HW, txBuffer, CY_SPI_FLASH_PAGE_SIZE,
                    glReadWriteWidth[flashIndex], &spiContext);
            if (status != CY_SMIF_SUCCESS)
            {
                DBG_APP_ERR("Error: Cy_SMIF_TransmitDataBlocking failed :0x%x\r\n", status);
                return status;
            }
            else if(status == CY_SMIF_SUCCESS)
            {
                Cy_SysLib_DelayUs(100);
                while(Cy_SPI_IsMemBusy(flashIndex))
                {
                    if(programWait++ >= CY_SPI_PROGRAM_TIMEOUT_US)
                    {
                        status =  CY_SMIF_EXCEED_TIMEOUT;
                        break;
                    }
                    else
                    {
                        Cy_SysLib_DelayUs(1);
                    }
                }
            }
            
        }
    }
#endif /* FLASH_AT45D */
    return status;
}

/* Flash Write*/
cy_en_smif_status_t Cy_SPI_WriteOperation(uint32_t address, uint8_t *txBuffer, uint32_t length, uint32_t numPages, cy_en_flash_index_t flashIndex)
{
    uint32_t pageIndex = 0;
    uint32_t spiAddress = address;
    uint32_t pageOffset = 0;
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    for(pageIndex = 0; pageIndex < numPages; pageIndex++)
    {
        pageOffset = CY_SPI_FLASH_PAGE_SIZE * pageIndex;
        status = Cy_SPI_WritePage(spiAddress, txBuffer + pageOffset, flashIndex);
        if(status != CY_SMIF_SUCCESS)
        {
            DBG_APP_ERR("Error: Write page failed at address 0x%x\r\n", spiAddress);
            return status;
        }

        spiAddress += CY_SPI_FLASH_PAGE_SIZE;
    }

    return status;
}

/* Read Flash*/
cy_en_smif_status_t Cy_SPI_ReadOperation(uint32_t address, uint8_t *rxBuffer, uint32_t length, cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t addrArray[SPI_ADDRESS_BYTE_COUNT];                                     

#if FLASH_AT45D
    /* Convert address and add dummy byte */ 
    Cy_SPI_AddressToArray(address, addrArray, SPI_ADDRESS_BYTE_COUNT-1);
    addrArray[SPI_ADDRESS_BYTE_COUNT-1] = 0x00;                                            /* Dummy byte for the read command */

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
                                     CY_SPI_READ_CMD,                                      /* Read Data Bytes Low Frequency command*/
                                     CY_SMIF_WIDTH_SINGLE,
                                     addrArray,
                                     SPI_ADDRESS_BYTE_COUNT,                                /* 3 address bytes + 1 dummy byte*/
                                     glReadWriteWidth[flashIndex],
                                     (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
                                     CY_SMIF_TX_NOT_LAST_BYTE,
                                     &spiContext);
    
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    /* Receive the data into the rxBuffer*/ 
    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW, rxBuffer, length, glReadWriteWidth[flashIndex], &spiContext);
    if (status != CY_SMIF_SUCCESS)
    {
        DBG_APP_ERR("Error: Cy_SPI_ReadOperation read data 0x%x\r\n", status);
        return status;
    }

#else

    Cy_SPI_AddressToArray(address, addrArray, SPI_ADDRESS_BYTE_COUNT);
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_READ_CMD,
            glCommandWidth[flashIndex],
            addrArray,
            SPI_ADDRESS_BYTE_COUNT,
            glReadWriteWidth[flashIndex],
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    /* Receive the data into the rxBuffer*/ 
    if(status == CY_SMIF_SUCCESS)
    {
        status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW, rxBuffer, length, glReadWriteWidth[flashIndex], &spiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    }

#endif /* FLASH_AT45D */ 
    return status;
}

/* Check status */
void Cy_App_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
        DBG_APP_ERR("Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

