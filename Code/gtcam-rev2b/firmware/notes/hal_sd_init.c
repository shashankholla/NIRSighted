/**
 * Code copied from stm32l4xx_hal_sd.c and stripped down so I can get a better feel for what it does.
 *
 * A lot of sanity checks are taken out and a lot of comments are added.
 */

/**
  * @brief  Initializes the SD according to the specified parameters in the
            SD_HandleTypeDef and create the associated handle.
  * @param  hsd Pointer to the SD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *hsd)
{
    HAL_SD_CardStatusTypeDef CardStatus;
    uint32_t speedgrade, unitsize;
    uint32_t tickstart;

    if(hsd->State == HAL_SD_STATE_RESET)
    {
        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        // This function is declared as __weak in stm32l4xx_hal_sd.c and is overridden in
        // auto-generated STM32CubeMX code.
        // It enables the SDMMC and GPIO clocks in the power manager and configures the appropriate
        // GPIO pins.
        HAL_SD_MspInit(hsd);
    }

    hsd->State = HAL_SD_STATE_BUSY;

    /* Initialize the Card parameters */
    // HAL_SD_InitCard does quite a bit of lifting.
    //
    if (HAL_SD_InitCard(hsd) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Not sure what HAL_SD_GetCardStatus does....
    if( HAL_SD_GetCardStatus(hsd, &CardStatus) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Get Initial Card Speed from Card Status*/
    speedgrade = CardStatus.UhsSpeedGrade;
    unitsize = CardStatus.UhsAllocationUnitSize;
    if ((hsd->SdCard.CardType == CARD_SDHC_SDXC) && ((speedgrade != 0U) || (unitsize != 0U)))
    {
        hsd->SdCard.CardSpeed = CARD_ULTRA_HIGH_SPEED;
    }
    else
    {
        if (hsd->SdCard.CardType == CARD_SDHC_SDXC)
        {
            hsd->SdCard.CardSpeed  = CARD_HIGH_SPEED;
        }
        else
        {
            hsd->SdCard.CardSpeed  = CARD_NORMAL_SPEED;
        }

    }

    /* Configure the bus wide */
    if(HAL_SD_ConfigWideBusOperation(hsd, hsd->Init.BusWide) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Verify that SD card is ready to use after Initialization */
    tickstart = HAL_GetTick();
    while((HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER))
    {
        if((HAL_GetTick()-tickstart) >=  SDMMC_DATATIMEOUT)
        {
            hsd->ErrorCode = HAL_SD_ERROR_TIMEOUT;
            hsd->State= HAL_SD_STATE_READY;
            return HAL_TIMEOUT;
        }
    }
#endif /* STM32L4P5xx || STM32L4Q5xx || STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */

    /* Initialize the error code */
    hsd->ErrorCode = HAL_SD_ERROR_NONE;

    /* Initialize the SD operation */
    hsd->Context = SD_CONTEXT_NONE;

    /* Initialize the SD state */
    hsd->State = HAL_SD_STATE_READY;

    return HAL_OK;
}

/**
  * @brief  Initializes the SD Card.
  * @param  hsd Pointer to SD handle
  * @note   This function initializes the SD card. It could be used when a card
            re-initialization is needed.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_SD_InitCard(SD_HandleTypeDef *hsd)
{
  uint32_t errorstate;
  HAL_StatusTypeDef status;
  SD_InitTypeDef Init;
  uint32_t sdmmc_clk;

  /* Default SDMMC peripheral configuration for SD card initialization */
  Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  Init.BusWide             = SDMMC_BUS_WIDE_1B;
  Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;

  /* Init Clock should be less or equal to 400Khz*/
  sdmmc_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC1);
  Init.ClockDiv = sdmmc_clk/(2U*400000U);

  /* Set Transceiver polarity */
  hsd->Instance->POWER |= SDMMC_POWER_DIRPOL;

  /* Initialize SDMMC peripheral interface with default configuration */
  // This just initializes the CLKCR register of the SDMMC peripheral
  status = SDMMC_Init(hsd->Instance, Init);

  /* Set Power State to ON */
  // Set the bottom 2 bits of the SDMMC_POWER register, which turns on the SDMMC module
  status = SDMMC_PowerState_ON(hsd->Instance);

  /* wait 74 Cycles: required power up waiting time before starting
     the SD initialization sequence */
  // I guess that this automatically happens as soon as the SDMMC hardware is turned on.
  sdmmc_clk = sdmmc_clk/(2U*Init.ClockDiv);
  HAL_Delay(1U+ (74U*1000U/(sdmmc_clk)));

  /* Identify card operating voltage */
  // Issue CMD0, CMD8, and then issue ACMD41 over and over until bit 31 of the card's
  // OCR ("operation conditions register") is set, indicating that "the card power up procedure has
  // been finished".
  errorstate = SD_PowerON(hsd);
  if(errorstate != HAL_SD_ERROR_NONE)
  {
    hsd->State = HAL_SD_STATE_READY;
    hsd->ErrorCode |= errorstate;
    return HAL_ERROR;
  }

  /* Card initialization */
  // Issue CMD2 and CMD3 to put the card in "data transfer mode".
  errorstate = SD_InitCard(hsd);
  if(errorstate != HAL_SD_ERROR_NONE)
  {
    hsd->State = HAL_SD_STATE_READY;
    hsd->ErrorCode |= errorstate;
    return HAL_ERROR;
  }

  /* Set Block Size for Card */
  // Use CMD16 to set block size to 512, which is the same as the default.
  errorstate = SDMMC_CmdBlockLength(hsd->Instance, BLOCKSIZE);
  if(errorstate != HAL_SD_ERROR_NONE)
  {
    /* Clear all the static flags */
    __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
    hsd->ErrorCode |= errorstate;
    hsd->State = HAL_SD_STATE_READY;
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Enquires cards about their operating voltage and configures clock
  *         controls and stores SD information that will be needed in future
  *         in the SD handle.
  * @param  hsd Pointer to SD handle
  * @retval error state
  *
  * John - there's quite a bit of stuff here. I think that it basically implements the flow chart
  * in section 3.6 of the SD Host Controller Simplified Spec: "Card Initialization and Identification".
  *
  * Command descriptions can be found in "Physical Layer Simplified Specification" 4.7.4 "Detailed
  * Command Description"
  *
  * These are the commands that SD_PowerON() appears to send:
  *     CMD0: Resets all cards to idle state
  *     CMD8: Sends interface info to the SD card, telling the card what operating voltage is intended.
  *           It seems like only one option is supported: 2.7 - 3.3v with reserved options for future
  *           compatibility with lower voltages.
  *     ACMD41: Not entirely sure what this command does... The simplified SD spec is clear as mud.
  *             It appears to do a few things:
  *               * starts an initialization process that takes ~1 second
  *               * sends some operating information to the SD card (XPC (max power), VDD window)
  *               * recieves some operating info back from the SD card.
  *             SD_PowerON() repeats this command up to 0xffff times, waiting for a '1' in the
  *             "card power-up status bit" of the OCR (operation conditions register).
  *             More info about this command can be found in section 3.6 of the "host controller
  *             simplified spec".
  *
  * Note that it DOESN'T try to switch the card into 1.8V mode; the STM32L4S requires an external
  * voltage translator (called a transceiver in the HAL code), so we can't use 1.8V mode. This
  * option to not switch the card over to 1.8V mode is specified in
  * SD_HandleTypeDef.init.Transceiver.
  */
static uint32_t SD_PowerON(SD_HandleTypeDef *hsd)
{
    __IO uint32_t count = 0U;
    uint32_t response = 0U, validvoltage = 0U;
    uint32_t errorstate;
    uint32_t tickstart = HAL_GetTick();

    /* CMD0: GO_IDLE_STATE */
    // SDMMC_CmdGoIdleState first calls SDMMC_SendCommand() which is nonblocking and just "launches" a
    // new command by writing to SDMMCx->ARG and SDMMCx->CMD.
    // It then calls SDMMC_GetCmdError(), which polls SDMMCx->STAR's CMDSENT bit.
    if ((errorstate = SDMMC_CmdGoIdleState(hsd->Instance)) != HAL_SD_ERROR_NONE) return errorstate;

    /* CMD8: SEND_IF_COND: Command available only on V2.0 cards */
    // If the card doesn't respond to this command, it's a legacy card or not really an SD card.
    errorstate = SDMMC_CmdOperCond(hsd->Instance);
    if(errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->SdCard.CardVersion = CARD_V1_X;
        /* CMD0: GO_IDLE_STATE */
        errorstate = SDMMC_CmdGoIdleState(hsd->Instance);
        if(errorstate != HAL_SD_ERROR_NONE)
        {
            return errorstate;
        }

    }
    else
    {
        hsd->SdCard.CardVersion = CARD_V2_X;
    }

    //
    if( hsd->SdCard.CardVersion == CARD_V2_X)
    {
        /* SEND CMD55 APP_CMD with RCA as 0 */
        // Note: RCA is the 16-bit "relative card address" register; used when multiple SD cards are
        // on the same bus. 0x0000 is the default address.
        // Not really sure what this is supposed to achieve... I guess it just checks to make sure
        // that the SD card will respond to "app command"s?
        // Worth pointing out that multiple CMD55s in a row just resolve to one CMD55, so the fact
        // that this function sends another CMD55 right after this one is fine; "ACMD55" is
        // eqivalent to CMD55.
        errorstate = SDMMC_CmdAppCommand(hsd->Instance, 0);
        if(errorstate != HAL_SD_ERROR_NONE)
        {
            return HAL_SD_ERROR_UNSUPPORTED_FEATURE;
        }
    }

    /* SD CARD */
    /* Send ACMD41 SD_APP_OP_COND with Argument 0x80 10 00 00 */
    while((count < SDMMC_MAX_VOLT_TRIAL) && (validvoltage == 0U))
    {
        /* SEND CMD55 APP_CMD with RCA as 0 */
        errorstate = SDMMC_CmdAppCommand(hsd->Instance, 0);
        if(errorstate != HAL_SD_ERROR_NONE)
        {
            return errorstate;
        }

        /* Send CMD41 */
        errorstate = SDMMC_CmdAppOperCommand(hsd->Instance, SDMMC_VOLTAGE_WINDOW_SD | SDMMC_HIGH_CAPACITY | SD_SWITCH_1_8V_CAPACITY);
        if(errorstate != HAL_SD_ERROR_NONE)
        {
            return HAL_SD_ERROR_UNSUPPORTED_FEATURE;
        }

        /* Get command response */
        response = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);

        /* Get operating voltage*/
        validvoltage = (((response >> 31U) == 1U) ? 1U : 0U);

        count++;
    }

    //
    if(count >= SDMMC_MAX_VOLT_TRIAL)
    {
        return HAL_SD_ERROR_INVALID_VOLTRANGE;
    }

    if((response & SDMMC_HIGH_CAPACITY) == SDMMC_HIGH_CAPACITY) /* (response &= SD_HIGH_CAPACITY) */
    {
        hsd->SdCard.CardType = CARD_SDHC_SDXC;

        if(hsd->Init.Transceiver == SDMMC_TRANSCEIVER_ENABLE)
        {
            if((response & SD_SWITCH_1_8V_CAPACITY) == SD_SWITCH_1_8V_CAPACITY)
            {
                hsd->SdCard.CardSpeed = CARD_ULTRA_HIGH_SPEED;

                /* Start switching procedue */
                hsd->Instance->POWER |= SDMMC_POWER_VSWITCHEN;

                /* Send CMD11 to switch 1.8V mode */
                errorstate = SDMMC_CmdVoltageSwitch(hsd->Instance);
                if(errorstate != HAL_SD_ERROR_NONE)
                {
                    return errorstate;
                }

                /* Check to CKSTOP */
                while(( hsd->Instance->STA & SDMMC_FLAG_CKSTOP) != SDMMC_FLAG_CKSTOP)
                {
                    if((HAL_GetTick() - tickstart) >=  SDMMC_DATATIMEOUT)
                    {
                        return HAL_SD_ERROR_TIMEOUT;
                    }
                }

                /* Clear CKSTOP Flag */
                hsd->Instance->ICR = SDMMC_FLAG_CKSTOP;

                /* Check to BusyD0 */
                if(( hsd->Instance->STA & SDMMC_FLAG_BUSYD0) != SDMMC_FLAG_BUSYD0)
                {
                    /* Error when activate Voltage Switch in SDMMC Peripheral */
                    return SDMMC_ERROR_UNSUPPORTED_FEATURE;
                }
                else
                {
                    /* Enable Transceiver Switch PIN */
                    HAL_SDEx_DriveTransceiver_1_8V_Callback(SET);

                    /* Switch ready */
                    hsd->Instance->POWER |= SDMMC_POWER_VSWITCH;

                    /* Check VSWEND Flag */
                    while(( hsd->Instance->STA & SDMMC_FLAG_VSWEND) != SDMMC_FLAG_VSWEND)
                    {
                        if((HAL_GetTick() - tickstart) >=  SDMMC_DATATIMEOUT)
                        {
                            return HAL_SD_ERROR_TIMEOUT;
                        }
                    }

                    /* Clear VSWEND Flag */
                    hsd->Instance->ICR = SDMMC_FLAG_VSWEND;

                    /* Check BusyD0 status */
                    if(( hsd->Instance->STA & SDMMC_FLAG_BUSYD0) == SDMMC_FLAG_BUSYD0)
                    {
                        /* Error when enabling 1.8V mode */
                        return HAL_SD_ERROR_INVALID_VOLTRANGE;
                    }
                    /* Switch to 1.8V OK */

                    /* Disable VSWITCH FLAG from SDMMC Peripheral */
                    hsd->Instance->POWER = 0x13U;

                    /* Clean Status flags */
                    hsd->Instance->ICR = 0xFFFFFFFFU;
                }
            }
        }
#endif /* STM32L4P5xx || STM32L4Q5xx || STM32L4R5xx || STM32L4R7xx || STM32L4R9xx || STM32L4S5xx || STM32L4S7xx || STM32L4S9xx */
    }
    else
    {
        hsd->SdCard.CardType = CARD_SDSC;
    }


    return HAL_SD_ERROR_NONE;
}


/**
  * @brief  Initializes the sd card.
  * @param  hsd Pointer to SD handle
  * @retval SD Card error state
  *
  * CMD2: This command requests the CID (Card ID register); when it answers a request for the CID,
  *       the card moves from "ready" state to "identification" state.
  * CMD3: This command requests the card's RCA (Relative Card Address), a peripheral address that's
  *       "dynamically suggested" by the cards. After they first publish their RCA, cards enter
  *       "data transfer mode".
  **********
  * CMD9: Addressed with the RCA, the respondant card sends its CSD (card-specific data). The CSD
  *       is 128 bits long and contains a bunch of assorted info like timing, block size, etc. Info
  *       about the CSD can be found in the SD "Physical Layer" Simplified Specification.
  *       CMD9 has no side effects, after answering CMD9,
  */
static uint32_t SD_InitCard(SD_HandleTypeDef *hsd)
{
    HAL_SD_CardCSDTypeDef CSD;
    uint32_t errorstate;
    uint16_t sd_rca = 1U;

    // Send CMD2 to get the CID (card ID register), containing information about the card, like
    // its manufacturer and serial number.
    if(hsd->SdCard.CardType != CARD_SECURED)
    {
        /* Send CMD2 ALL_SEND_CID */
        errorstate = SDMMC_CmdSendCID(hsd->Instance);
        if(errorstate != HAL_SD_ERROR_NONE)
        {
            return errorstate;
        }
        else
        {
            /* Get Card identification number data */
            hsd->CID[0U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);
            hsd->CID[1U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP2);
            hsd->CID[2U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP3);
            hsd->CID[3U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP4);
        }
    }

    if(hsd->SdCard.CardType != CARD_SECURED)
    {
        /* Send CMD3 SET_REL_ADDR with argument 0 */
        /* SD Card publishes its RCA. */
        // The cards dynamically "suggest" RCAs which are approved by the host. Cards enter "data
        // transfer mode" after they first publish their RCA.
        errorstate = SDMMC_CmdSetRelAdd(hsd->Instance, &sd_rca);
        if(errorstate != HAL_SD_ERROR_NONE)
        {
            return errorstate;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // CARD IS NOW IN DATA TRANSFER MODE

    if(hsd->SdCard.CardType != CARD_SECURED)
    {
        /* Get the SD card RCA */
        hsd->SdCard.RelCardAdd = sd_rca;

        /* Send CMD9 SEND_CSD with argument as card's RCA */
        errorstate = SDMMC_CmdSendCSD(hsd->Instance, (uint32_t)(hsd->SdCard.RelCardAdd << 16U));
        if(errorstate != HAL_SD_ERROR_NONE)
        {
            return errorstate;
        }
        else
        {
            /* Get Card Specific Data */
            hsd->CSD[0U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);
            hsd->CSD[1U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP2);
            hsd->CSD[2U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP3);
            hsd->CSD[3U] = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP4);
        }
    }

    /* Get the Card Class */
    hsd->SdCard.Class = (SDMMC_GetResponse(hsd->Instance, SDMMC_RESP2) >> 20U);

    /* Get CSD parameters */
    // don't send a command, just check the CSD parameters that we've already recieved.
    if (HAL_SD_GetCardCSD(hsd, &CSD) != HAL_OK)
    {
        return HAL_SD_ERROR_UNSUPPORTED_FEATURE;
    }

    /* Select the Card */
    // CMD7: move the card from standby state into transfer state.
    errorstate = SDMMC_CmdSelDesel(hsd->Instance, (uint32_t)(((uint32_t)hsd->SdCard.RelCardAdd) << 16U));
    if(errorstate != HAL_SD_ERROR_NONE)
    {
        return errorstate;
    }

#if !defined(STM32L4P5xx) && !defined(STM32L4Q5xx) && !defined(STM32L4R5xx) && !defined(STM32L4R7xx) && !defined(STM32L4R9xx) && !defined(STM32L4S5xx) && !defined(STM32L4S7xx) && !defined(STM32L4S9xx)
    /* Configure SDMMC peripheral interface */
    (void)SDMMC_Init(hsd->Instance, hsd->Init);
#endif /* !STM32L4P5xx && !STM32L4Q5xx && !STM32L4R5xx && !STM32L4R7xx && !STM32L4R9xx && !STM32L4S5xx && !STM32L4S7xx && !STM32L4S9xx */

    /* All cards are initialized */
    return HAL_SD_ERROR_NONE;
}



/**
 * @brief  Send Status info command.
 * @param  hsd pointer to SD handle
 * @param  pSDstatus Pointer to the buffer that will contain the SD card status
 *         SD Status register)
 * @retval error state
 *
 * Side effect: block size is changed to 64 at the end of this function.
 */
static uint32_t SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus)
{
    SDMMC_DataInitTypeDef config;
    uint32_t errorstate;
    uint32_t tickstart = HAL_GetTick();
    uint32_t count;
    uint32_t *pData = pSDstatus;

    /* Check SD response */
    // SD_SendSDStatus assumes that there's a valid command response already in RESP1, which it
    // uses to check and see whether the card is locked or not
    if((SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1) & SDMMC_CARD_LOCKED) == SDMMC_CARD_LOCKED)
    {
        return HAL_SD_ERROR_LOCK_UNLOCK_FAILED;
    }

    /* Set block size for card if it is not equal to current block size for card */
    // ??? dude what ???
    // The SD status - described in section 4.10.2 - takes up 64 Bytes and is transferred in
    // response to ACMD13 the same way that data being read from the card's flash memory is.
    errorstate = SDMMC_CmdBlockLength(hsd->Instance, 64U);
    if(errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_NONE;
        return errorstate;
    }

    /* Send CMD55 */
    errorstate = SDMMC_CmdAppCommand(hsd->Instance, (uint32_t)(hsd->SdCard.RelCardAdd << 16U));
    if(errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_NONE;
        return errorstate;
    }

    /* Configure the SD DPSM (Data Path State Machine) */
    // It looks like this function is probably pretty important for my purposes. It sets up an SD
    // transfer.
    config.DataTimeOut   = SDMMC_DATATIMEOUT;
    config.DataLength    = 64U;
    config.DataBlockSize = SDMMC_DATABLOCK_SIZE_64B;
    config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
    config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
    config.DPSM          = SDMMC_DPSM_ENABLE;

    // Writes to DTIMER, DLEN, and DCTRL to set up a new transfer
    (void)SDMMC_ConfigData(hsd->Instance, &config);

    /* Send ACMD13 (SD_APP_STAUS)  with argument as card's RCA */
    errorstate = SDMMC_CmdStatusRegister(hsd->Instance);
    if(errorstate != HAL_SD_ERROR_NONE)
    {
        hsd->ErrorCode |= HAL_SD_ERROR_NONE;
        return errorstate;
    }

    /* Get status data */
    // Looks like this loop and the next one are a kind of messy hodgepodge that poll until the end
    // of the transfer. This first loop waits
    while(!__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DATAEND))
    {
        if(__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXFIFOHF))
        {
            for(count = 0U; count < 8U; count++)
            {
                // Just reads one byte from the FIFO register.
                *pData = SDMMC_ReadFIFO(hsd->Instance);
                pData++;
            }
        }

        if((HAL_GetTick() - tickstart) >=  SDMMC_DATATIMEOUT)
        {
            return HAL_SD_ERROR_TIMEOUT;
        }
    }

    // Check for error flags.
    if(__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DTIMEOUT)) return HAL_SD_ERROR_DATA_TIMEOUT;
    else if(__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DCRCFAIL)) return HAL_SD_ERROR_DATA_CRC_FAIL;
    else if(__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_RXOVERR)) return HAL_SD_ERROR_RX_OVERRUN;

    // ??? I think this loop should never be entered. The first loop won't transfer unless there's
    // an error OR DATAEND is set. DATAEND will only be set if the FIFO is empty, and if the FIFO is
    // empty, then
    // Which then kinda begs the question: what's the difference between DPSMACT and DATAEND?
    while ((__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DPSMACT))) {
        *pData = SDMMC_ReadFIFO(hsd->Instance);
        pData++;

        if ((HAL_GetTick() - tickstart) >=  SDMMC_DATATIMEOUT) {
            return HAL_SD_ERROR_TIMEOUT;
        }
    }

    /* Clear all the static status flags*/
    __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_DATA_FLAGS);

    return HAL_SD_ERROR_NONE;
}
