# Black Pill (STM32F401CC) as a Black Magic Probe

Black Magic Debug v2.0.0, `blackpill-f401cc` probe, built with `-Dbmd_bootloader=false`
so it sits at 0x08000000 and flashes straight through the F401's ROM USB DFU.

Flash: hold BOOT0, tap NRST, release BOOT0, then in STM32CubeProgrammer pick USB,
Connect, open `bmp_f401cc.hex` (or the .bin at 0x08000000) and Start Programming.

Wiring to the motor board: PB8 = SWCLK, PB9 = SWDIO, PA5 = NRST, GND, (3V3).
PlatformIO: `upload_protocol = blackmagic`, `upload_port = COMx` (GDB server port).
