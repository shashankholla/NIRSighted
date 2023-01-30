What interrupts / status bits are available in the SDMMC peripheral?

Here are the ones that we care about for sure:
    Relating to CPSM (Command Path State Machine):
     ** CMDREND: Command response received (CRC passed / no CRC)
                 Looks like this will mark the end of successful TX / RX of a command.
     ** CCRCFAIL: Command response received (but CRC check failed)
     ** CTIMEOUT: Command response timed out
                  No response receieved after 64 SDMMC_CK cycles.
     ** CMDSENT: Command sent
                 Set when a command that doesn't have a response finishes sending.

    Relating to DPSM (Data Path State Machine):
     ** DATAEND: Data transfer ended correctly
    Set when DATACOUNT reaches 0. Looks like this is only set after the SD bus goes not busy; the
    datasheet seems to imply that a DPSM transition from Wait_S to Idle is required for this bit to
    be set. I also think
     ** DTIMEOUT: Data timeout
     ** DCRCFAIL: Data transfer finished, but CRC

 ** BUSYD0END: end of D0 busy following a CMD response detected
    Note: this bit does not signal busy due to data xfer.
    This command is only relevant for commands with an R1b-type response. It might take many cycles
    for the card to "fulfill" the command. DAT0 will be held low by the card until the card is done
    handling the command.
    There are some commands we need to service (CMD7 & CMD12) with an R1b response, so we'll
    probably want to handle this interrupt.

 ** BUSYD0: Inverted value of SDMMC_D0 line, sampled at the end of a CMD response and a second time
            2 SDMMC_CK cycles after the CMD response.
    See section 54.6.6; should be used in conjunction with CMDREND and BUSYD0END to properly handle
    R1b responses that are terminated right away.

Here are the ones that we MAYBE care about:


  ? DABORT: Data transfer aborted by CMD12?
    This flag signals that the data path SM has become inactive because a transfer was aborted by
    a CMD12.

  ? IDMATE: IDMA transfer error
    Might want to handle this one, but it's a lower priority.

  ? DBCKEND: Data block sent / received
    I can barely figure out what the significance of this flag is, but it's related to some very
    specific state transfers in the DPSM.

  ? DHOLD: Data Transfer Hold
    Not entirely sure what this does.... I think this flag gets triggered when the DPSM is ready to
    begin a transfer but DTHOLD is set, preventing the transfer?

    IDMABTC: IDMA buffer xfer complete
    Not using IDMA double-buffer feature, but if we do, we'll need to use it.



Here are some that we for sure don't care about:
    CKSTOP: SDMMC_CK stopped in voltage switch procedure
    Don't care. No voltage switch.

    VSWEND: Voltage Switch critical timing section completion
    Don't care. No voltage switch.

    ACKTIMEOUT: Boot ack timeout.
    Don't care. Not using "boot" mode (and tbh i don't even know what boot mode is).

    ACKFAIL: Boot ack received.
    Don't care. Not using "boot" mode (and tbh i don't even know what boot mode is).

    SDIOIT: SDIO interrupt received.
    Don't care. I don't think that we have any SDIO interrupts that we're worried about.

    {T,R}XFIFO{E,F,HF}: Is {T,R}X FIFO {empty, fully, half-full}?
    Don't care. Not hooked up to an interrupt, just a status bit.

    {CP,DP}SMACT: {Command Path, Data Path} State Machine Active?
    Don't care. Just status bits.
