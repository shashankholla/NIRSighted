To be able to understand the SDMMC peripheral on the STM32, it's helpful to have at least a high-level understanding of the SD protocol, because a lot of the design choices of the SD protocol are baked directly into the SDMMC hardware. The best place to read about the SD protocol is in the [SD Physical Layer Simplified Specification](https://www.sdcard.org/downloads/pls/). The most relevant parts of the document are chapters 3.6 (4 pages), 4.1 (1 page), 4.2.0 - 4.2.3 (5 pages), 4.3.0 - 4.3.4 (5 pages), 4.7.1 - 4.7.3 (1 page) (but 4.7.4 - a table of all the commands - is a good reference), and 4.9 (3 pages).

Many of the sections can be skimmed. Only sections 3.6 and 4.3.0 - 4.3.4 need to be closely read, a total of about 10 pages.



The big takeaway for me is that the SD "physical layer" consists of 2 seperate half-duplex busses: a 1-bit wide command bus and a 4-bit wide data bus. Both busses are bidirectional. Both busses are clocked by the same unidirectional (no peripheral device clock-stretching allowed like in i2c) clock line. Each bus has it's own basic sequencing that it needs to follow, but the busses also mutually determine what states each one will be in. Commands on the CMD bus dictate what, when, and which direction data on the data bus will go. The data bus has a peripheral-device controlled busy signal (shared with D[0]) that determines when new commands can be sent.

This is reflected in the STM32L4 SDMMC's design choices. There are 2 seperate state machines, one of which controls the commands flowing in & out of the command line and the other controlling data flowing in and out of the data bus. These state machines are largely independant, but sometimes interlock in a few ways. For instance, the SDMMC peripheral can be programmed to automatically start a data transmission only after CMD tx/rx has finished (done via the CMDTRANS bit in SDMMC_CMDR.



Hang during SD_PowerON bug:
Places within HAL_SD_Init() where HAL_Delay() is called...
  *  HAL_SD_Init -> HAL_SD_InitCard -> SDMMC_PowerState_ON
     Delay after turning on power to the on-chip SD card peripheral
  *  HAL_SD_Init -> HAL_SD_InitCard
     The big 74 cycle wait.

And that's it!!



Ok.... sometimes it's useful to figure out where an ISR will return to; maybe you don't want to break IN the ISR, but you care about the context that it came from.

    break IRQHandler
    commands
    silent
    printf "breaking at return from ISR: %p\n", ((uint32_t*)$psp)[6]
    break *((uint32_t*)$psp)[6]
    cont
    end
