## STM32Cube example project
This is an example, minimal bare-metal application for the STM32 that doesn't rely on any STM code generation tools and uses headers and startup files located elsewhere.

It's for the "Ground Truth Camera" board, rev 2b, and just sends a character repeatedly over UART.

## Programming Instructions
I typically use `JLinkGDBServerCLExe` and `gdb-multiarch` together to program and debug the STM32. There are other good options, but these are the ones I know how to use.

To program the microcontroller, you need a programmer (like [this one](https://www.adafruit.com/product/3571) plugged into the programming port of the microcontroller.

#### `JLinkGDBServer`
`JLinkGDBServerCLExe` acts as a bridge between `gdb` and the microcontroller. It can be downloaded [here](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack) from the section titled "J-Link Software and Documentation Pack".

In one window run:
```
JLinkGDBServerCLExe -if SWD -device <your microcontroller name here>
```
(`-if SWD` tells `JLinkGDBServerCLExe to use the "serial-wire-debug" interface instead of JTAG, which is the default)

Because this project uses the STM32L4S9ZI microcontroller, the command for this project would be:
```
JLinkGDBServerCLExe -if SWD -device STM32L4S9ZI
```

Just so you know, a "terms of use" pop-up might show up if your programmer is an "educational grade" one.

If the programmer connected to your microcontroller successfully, a message like this one will pop up:
```
Connecting to J-Link...
J-Link is connected.
Firmware: J-Link EDU Mini V1 compiled Jan  7 2020 16:53:19
Hardware: V1.00
S/N: 801010392
Feature(s): FlashBP, GDB
Checking target voltage...
Target voltage: 2.79 V
Listening on TCP/IP port 2331
Connecting to target...
Connected to target
Waiting for GDB connection...
```

Which means that it's ready to connect to `gdb`

#### `gdb`
This section assumes that you're familiar with `gdb`.

"normal" `gdb` won't work here. you need to use a variant called `gdb-multiarch`.

in another window run
```
gdb-multiarch
```

connect it to `JLinkGDBServerCLExe` by doing
```
(gdb) target remote localhost:2331
```

tell it which file to work with by doing
```
(gdb) file <your .elf filename>
```

now type
```
(gdb) load
```
to program the microcontroller

and then
```
(gdb) monitor reset
(gdb) continue
```
to start the program.

You can press Ctrl+C at any time to halt the microcontroller letting you set breakpoints and look at variables.
