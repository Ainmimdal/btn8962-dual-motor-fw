# Notes for Claude working on this repo

Firmware, GUI and tools for Imad's STM32F103C8T6 + 4x BTN8962 dual brushed motor board. README.md has the pin map, protocol and settings; this file holds what isn't written there.

## Working with Imad
- Commits are authored as Imad only. No Co-Authored-By, session links or "Generated with Claude Code" lines.
- Act directly and keep replies short. Don't ask him to run commands; he launches the GUI by double-clicking `Motor GUI.lnk` on the Desktop (`tools/gui/Motor GUI.pyw`).

## Bench setup (his Windows PC)
- Programmer: Black Pill (STM32F401CC) running Black Magic Probe, firmware in `tools/blackpill-bmp/`. Its GDB server is on the lower-numbered of its two COM ports (was COM12, UART on COM11). SWD wiring: Black Pill PB8 = SWCLK, PB9 = SWDIO, PA5 = NRST, GND.
- To flash with PlatformIO, use `upload_protocol = blackmagic` and `upload_port = <BMP GDB COM port>` (platformio.ini still defaults to stlink). The board runs env `enc1_tim3_3v3`.
- USB-UART adapter (CH340) to board connector U4: U4-1 GND, U4-2 = PB11 (board RX) <- adapter TXD, U4-3 = PB10 (board TX) -> adapter RXD. 115200 baud. Its COM port moves with the USB socket (seen as COM13 and COM14).
- ELEGOO/UltiMaker Cura grabs COM ports (USB printing plugin): close it, or disable "USB printing" in its Marketplace, before flashing or connecting.
- Python 3.10 with pyserial and python-can installed.

## Board facts that bit us
- Motor numbering is fixed as drawn: motor 1 = OUT1/OUT2 (U6/U8) with encoder 1 on CN1, motor 2 = OUT3/OUT4 (U2/U3) with encoder 2 on CN2.
- CN1 pin 4 is hard-wired +5V while CN1-2/3 go to PA6/PA7, which aren't 5V tolerant. His CHP-36GP-555 hall encoder is open-collector with ~10k pull-ups, which keeps the injected current within spec, so it is OK on CN1. Treat any push-pull 5V encoder on CN1 as unsafe.
- Current sense: the IS zero offset must be measured with the bridges enabled at 0 % duty (done at boot, `ISCAL` re-measures). The old INH-off calibration left a false ~2.7 A. The missing IS zener/series resistor is fault protection only, not the cause.
- IS scaling: R11/R14/R17/R20 = 3.3k, kILIS 19500 typical, so readings are within the chip's ~±20 % spread unless calibrated against a meter.
- Status/error LEDs are active low; the power LED is very bright (needs a resistor change).

## Checking changes without hardware
- `sh tools/board_sim/run.sh --selftest` runs the real control loop and protocol on the PC (Linux/macOS, needs ptys).
- `sh tools/board_sim/run.sh --odrive` plus `python3 tools/board_sim/test_can_odrive.py` checks CAN/ODrive coexistence.
