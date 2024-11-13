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

/* SMIF context*/
cy_stc_smif_context_t spiContext;
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
static cy_en_smif_status_t Cy_SPI_WriteEnable(cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
                                     CY_SPI_WRITE_ENABLE_CMD,
                                     CY_SMIF_WIDTH_SINGLE,
                                     NULL,
                                     CY_SMIF_CMD_WITHOUT_PARAM,
                                     CY_SMIF_WIDTH_NA,
                                     (cy_en_smif_slave_select_t)slaveSelect,
                                     CY_SMIF_TX_LAST_BYTE,
                                     &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
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

/* Function for Chip erase */
cy_en_smif_status_t Cy_SPI_ChipErase(cy_en_smif_slave_select_t slaveSelect) 
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    /* Commands for chip erase */
    uint8_t chipEraseCmd[] = {0xC7, 0x94, 0x80, 0x9A}; 

    /* Send the erase command sequence */ 
    status = Cy_SMIF_TransmitCommand(
        SMIF_HW,                         /* SMIF instance */ 
        chipEraseCmd[0],                 /* First command byte */ 
        CY_SMIF_WIDTH_SINGLE,            /* Command transfer width */ 
        &chipEraseCmd[1],                /* Remaining command bytes */ 
        sizeof(chipEraseCmd) - 1,        /* Command parameter size */ 
        CY_SMIF_WIDTH_SINGLE,            /* Command parameter transfer width */ 
        slaveSelect,                     /* Slave select */ 
        CY_SMIF_TX_LAST_BYTE,            /* Complete transfer */ 
        &spiContext                     /* Context */ 
    );

    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

/* Function to read device ID */
cy_en_smif_status_t Cy_SPI_ReadID(uint8_t *rxBuffer, cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
                            CY_SPI_READ_ID_CMD,
                            CY_SMIF_WIDTH_SINGLE,
                            NULL,
                            CY_SMIF_CMD_WITHOUT_PARAM,
                            CY_SMIF_WIDTH_NA,
                            (cy_en_smif_slave_select_t)slaveSelect,
                            CY_SMIF_TX_NOT_LAST_BYTE,
                            &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW, rxBuffer, CY_FLASH_ID_LENGTH, CY_SMIF_WIDTH_SINGLE, &spiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    return status;
}

/* Function to disable sector protection */
cy_en_smif_status_t Cy_SPI_DisableSectorProtection(cy_en_smif_slave_select_t slaveSelect) 
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    /* Commands to disable sector protection */
    uint8_t disableProtectionCmd[] = {0x3D, 0x2A, 0x7F, 0xCF}; 

    /* Send the disable protection command sequence */
    status = Cy_SMIF_TransmitCommand(
        SMIF_HW,                         
        disableProtectionCmd[0],         
        CY_SMIF_WIDTH_SINGLE,            
        &disableProtectionCmd[1],        
        sizeof(disableProtectionCmd) - 1,
        CY_SMIF_WIDTH_SINGLE,            
        slaveSelect,                     
        CY_SMIF_TX_LAST_BYTE,            
        &spiContext                     
    );

    if (status != CY_SMIF_SUCCESS) {
        DBG_APP_ERR("Error: TransmitCommand\r\n");
        return status;
    }

    /* Deselect and reselect the device to apply changes */
    Cy_SMIF_Disable(SMIF_HW);
    Cy_SysLib_DelayUs(10); 
    Cy_SMIF_Enable(SMIF_HW, &spiContext);

    return CY_SMIF_SUCCESS;
}

/* Function to check busy status of flash */
bool Cy_SPI_IsMemBusy(cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t statusReg = 0;
    bool isBusy = true;
    

    /* Send status register read command */
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
                                     CY_SPI_STATUS_READ_CMD, 
                                     CY_SMIF_WIDTH_SINGLE,
                                     NULL,
                                     CY_SMIF_CMD_WITHOUT_PARAM, 
                                     CY_SMIF_WIDTH_NA,
                                     slaveSelect,
                                     CY_SMIF_TX_NOT_LAST_BYTE,
                                     &spiContext);

    if (status != CY_SMIF_SUCCESS) {
        DBG_APP_ERR("Error: Read Command status=0x%x\r\n", status);
        return true; /* Return true to indicate an error occurred */
    }

    if (CY_SMIF_SUCCESS == status)
    {
        status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW,
                                             &statusReg,
                                             1u,
                                             CY_SMIF_WIDTH_SINGLE,
                                             &spiContext);
        if (status != CY_SMIF_SUCCESS) {
            DBG_APP_ERR("Error: Read Data cmd status=0x%x, statusReg=0x%x\r\n", statusReg, status);
            return true; /* Return true to indicate an error occurred */
        }
        else
        {
            Cy_SysLib_DelayUs(10);
        }    
    }

    /* b7: Ready/busy status (1 = ready, 0 = busy) */
    isBusy = !(statusReg & 0x80);
    return isBusy;
}

/* Function to enable SPI block */
cy_en_smif_status_t Cy_SPI_Start(cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint32_t programWait = 0;
    
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

    while(Cy_SPI_IsMemBusy(slaveSelect)) { 
        if(programWait++ >= CY_APP_SPI_PROGRAM_TIMEOUT_US) 
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

/* Function to erase flash sector */
cy_en_smif_status_t Cy_SPI_SectorErase(cy_en_smif_slave_select_t slaveSelect, uint32_t address)
{
#if FLASH_AT45D
    /* Skip erase and return. Erase is performed by write command */
    return CY_SMIF_SUCCESS;
#else

    DBG_APP_TRACE("SectorEraseAddr: 0x%x,",address);
    cy_en_smif_status_t status;

    unsigned int sectorNumber = address >> 16;  /*Shifting right by 16 bits to extract the sector number */
    DBG_APP_TRACE("sectorNo: 0x%x,",sectorNumber);

    uint32_t flashAddr = sectorNumber << 10;    /* Masking the lower 16 bits and left shifting by 10 */
    DBG_APP_TRACE("flashAddr: 0x%x\r\n",flashAddr);

    /* Step 2: Send Sector Erase Command with Address */
    Cy_SysLib_DelayUs(10);

    uint8_t addrArray[SPI_ADDRESS_BYTE_COUNT];
    Cy_SPI_AddressToArray(flashAddr, addrArray, SPI_ADDRESS_BYTE_COUNT);
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
                            CY_SPI_SECTOR_ERASE_CMD,
                            CY_SMIF_WIDTH_SINGLE,
                            addrArray,
                            SPI_ADDRESS_BYTE_COUNT,
                            CY_SMIF_WIDTH_SINGLE,
                            slaveSelect,
                            CY_SMIF_TX_NOT_LAST_BYTE,
                            &spiContext);

    if (status != CY_SMIF_SUCCESS)
    {
        DBG_APP_ERR("Error: Erase cmd status: 0x%x\r\n", status);
        return status;
    }

    Cy_SysLib_DelayUs(100);
    return CY_SMIF_SUCCESS;
#endif
}

/* Write to Flash page*/
cy_en_smif_status_t Cy_SPI_WritePage(uint32_t address, uint8_t *txBuffer, cy_en_smif_slave_select_t slaveSelect)
{
    uint8_t addrArray[3]; 
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint32_t programWait = 0;

    Cy_SPI_AddressToArray(address, addrArray, 3);  /* Convert address to 3 bytes*/ 
    status = Cy_SPI_WriteEnable(slaveSelect); 
    Cy_SysLib_DelayUs(10);

    if(status == CY_SMIF_SUCCESS)
    {
        /* Step 1: Write to Buffer 1 using command 0x84 */
        status = Cy_SMIF_TransmitCommand(
            SMIF_HW,                        /* SMIF instance */
            0x84,                           /* Buffer 1 Write command */
            CY_SMIF_WIDTH_SINGLE,           /* Command transfer width */
            addrArray,                      /* Address array */
            3,                              /* Address array size (3 bytes) */
            CY_SMIF_WIDTH_SINGLE,           /* Address transfer width */
            slaveSelect,                    /* Slave select */
            CY_SMIF_TX_NOT_LAST_BYTE,       /* Not the last byte yet */
            &spiContext                     /* Context */
        );

        if (status != CY_SMIF_SUCCESS)
        {
            DBG_APP_ERR("Error: Buffer1 WR cmd status:0x%x\r\n", status);
            return status;
        }

        status = Cy_SMIF_TransmitDataBlocking(
            SMIF_HW, 
            txBuffer, 
            CY_APP_SPI_FLASH_PAGE_SIZE, 
            CY_SMIF_WIDTH_SINGLE, 
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
            SMIF_HW,                        /* SMIF instance */
            0x83,                           /* Buffer 1 to Main Memory Page Program With Built-In Erase command */
            CY_SMIF_WIDTH_SINGLE,           /* Command transfer width */
            addrArray,                      /* Address array */
            3,                              /* Address array size (3 bytes) */
            CY_SMIF_WIDTH_SINGLE,           /* Address transfer width */
            slaveSelect,                    /* Slave select */ 
            CY_SMIF_TX_LAST_BYTE,           /* Last byte */
            &spiContext                     /* Context */
        );
        if (status != CY_SMIF_SUCCESS)
        {
            DBG_APP_ERR("Error: Buffer 1 to Main Memory cmd status:0x%x\r\n", status);
            return status;
        }

        while(Cy_SPI_IsMemBusy(slaveSelect))
        {
            if(programWait++ >= CY_APP_SPI_PROGRAM_TIMEOUT_US)
            {
                status = CY_SMIF_EXCEED_TIMEOUT;
                DBG_APP_ERR("Error: Program Timeout\r\n");
                break;
            }
        }
    }
    return status;
}

/* Flash Write*/
cy_en_smif_status_t Cy_SPI_WriteOperation(uint32_t address, uint8_t *txBuffer, uint32_t length, uint32_t numPages, cy_en_smif_slave_select_t slaveSelect)
{
    uint32_t pageIndex = 0;
    uint32_t spiAddress = address;
    uint32_t pageOffset = 0;
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    for(pageIndex = 0; pageIndex < numPages; pageIndex++)
    {
        pageOffset = CY_APP_SPI_FLASH_PAGE_SIZE * pageIndex;
        status = Cy_SPI_WritePage(spiAddress, txBuffer + pageOffset, slaveSelect);
        if(status != CY_SMIF_SUCCESS)
        {
            DBG_APP_ERR("Error: Write page failed at address 0x%x\r\n", spiAddress);
            return status;
        }

        spiAddress += CY_APP_SPI_FLASH_PAGE_SIZE;
    }

    return status;
}

/* Read Flash*/
cy_en_smif_status_t Cy_SPI_ReadOperation(uint32_t address, uint8_t *rxBuffer, uint32_t length, cy_en_smif_slave_select_t slaveSelect)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t addrArray[4];                                      /*3 address bytes + 1 dummy byte */ 

    /* Convert address and add dummy byte */ 
    Cy_SPI_AddressToArray(address, addrArray, 3);
    addrArray[3] = 0x00;                                       /* Dummy byte for the read command */

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
                                     0x0B,                     /* Read Data Bytes Low Frequency command*/
                                     CY_SMIF_WIDTH_SINGLE,
                                     addrArray,
                                     4,                        /* 3 address bytes + 1 dummy byte*/
                                     CY_SMIF_WIDTH_SINGLE,
                                     slaveSelect,
                                     CY_SMIF_TX_NOT_LAST_BYTE,
                                     &spiContext);
    
    if (status != CY_SMIF_SUCCESS)
    {
        DBG_APP_ERR("Error: Cy_SPI_ReadOperation read cmd 0x%x\r\n", status);
        return status;
    }

    /* Receive the data into the rxBuffer*/ 
    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW, rxBuffer, length, CY_SMIF_WIDTH_SINGLE, &spiContext);
    if (status != CY_SMIF_SUCCESS)
    {
        DBG_APP_ERR("Error: Cy_SPI_ReadOperation read data 0x%x\r\n", status);
        return status;
    }
    
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

