# ADXL362 Sampling-Rate Investigation

Status: the first low-rate capture and a corrected-INT2 repeat are complete.
ADXL362_MEASUREMENTS.md records evidence, timing and limitations. High-rate
capture B confirms one complete 906-byte drain at approximately 4 MHz.
Capture C, obtained by manual hold-reset/arm/release after an unsuccessful
ST-Link attempt, verifies startup IDs and all fourteen configuration
write/readback pairs. ODR remains unresolved; no acceptance gate is waived.
The user deferred
base-station tests because the base station is not on site. Windows enumerates
the analyzer as USB-based DSL Instrument v2 (VID 2A0E / PID 002A); DSView 1.3.2
is installed. USB enumeration is not evidence that probes are connected.

Keep the existing normal-STOP Diagnostics image for the first measurements.
Do not change the classifier's 25 samples/output or sensor settings to hide
the observed approximately 19.75 samples/s rate. See VERIFICATION.md.

## Connections

Power down the board before attaching probes; check for shorts before power-up.
Use short signal leads and nearby grounds. These are passive digital inputs:
do not connect an analyzer signal/trigger output to the board's supply.
Leave CK, TI and TO disconnected and use the analyzer's internal sampling clock.
The analyzer's USB ground is common with the computer and must only connect
to board GND, never a supply rail. Keep all existing supplies compatible.

| Analyzer | Board net | STM32 pin |
| --- | --- | --- |
| GND | GND | Board GND |
| D0 | ADXL362 CS | PB12 |
| D1 | ADXL362 SCLK | PB13 |
| D2 | ADXL362 MOSI | PB15 |
| D3 | ADXL362 MISO | PB14 |
| D4 | ADXL362 INT2 | PB1 |

If only INT2 and GND are accessible, start with capture A. Do not guess tiny
package-pad positions from the logical pin names; consult the PCB/netlist.
Set the digital threshold to 1.5 V for the measured approximately 3 V board.
Disable input glitch filtering for initial evidence. No probe firmware or
Windows driver replacement is required for the detected device.

## Capture A: FIFO Interrupt Timing

- Enable D4; optionally also D0 to mark additional FIFO/register transactions.
- Stream mode, internal clock, 1 MS/s, single capture, at least 30 seconds.
  Choose the next longer supported duration if the exact value is unavailable.
- Start with Instant capture, without a trigger prerequisite. Do not reset the
  board for this run; save the continuous waveform after capture completes.
- Measure several D4 rising-edge intervals. INT2 currently reports FIFO
  watermark, not individual data-ready samples. Do not label its frequency ODR.
- A 453-word read represents 151 XYZ samples. If the previous FIFO drain was
  complete and no intermediate drain occurred, roughly 151/25 = 6.04 seconds
  is expected; previous UART evidence instead showed about 7.65 seconds.
- Checkpoints drain partial FIFOs approximately every 60 seconds. Exclude
  intervals spanning these extra drains or account for their sample counts;
  one interrupt interval by itself does not prove the internal sensor clock.
- If no edges appear, preserve that trace, confirm channel/ground/power and
  increase the sample rate if needed before attributing the result to firmware.

Capture UART concurrently when possible using Tools/capture-uart.ps1. It is
receive-only at 115200 8N1; align FIFO messages to the bus activity, allowing
for USB/UART buffering rather than treating host timestamps as exact IRQ time.

## Capture B: A Complete FIFO Drain

- Enable D0-D4, buffer mode, 50 MS/s, at least 50 ms (next longer option is OK).
- Simple trigger on D4 rising only, about 10 percent pre-trigger. Do not also
  require CS falling: multiple simple conditions are ANDed and need not coincide.
- Start a single capture. If it does not trigger within 20 seconds, stop and
  investigate capture A instead of leaving an unbounded wait.
- Recheck the top-bar rate/duration after closing Device Options: the installed
  DSView build restored old values during this session. Do not reopen that
  dialog after final verification unless a change is actually necessary.
- Add SPI decoding: CLK=D1, MOSI=D2, MISO=D3, CS=D0, active-low,
  CPOL=0, CPHA=0, MSB first, 8 bits per word. This sample rate provides more
  than ten samples per nominal 4 MHz SPI clock cycle.
- Inspect the measured SCLK period, CS boundaries, FIFO count and word tags.
  Verify that each transaction begins with its command, and that CS is high
  between chunks. FIFO command is 0x0D, with no register-address byte.
- A normal 906-byte read should be 510+396 data bytes in two transactions;
  each transaction adds its own one-byte command. The separate 900/1024-byte
  stress cases are 510+390 and 510+510+4, but are not exercised by this capture.
- FIFO count uses register read 0x0B, address 0x0C, then two result bytes.
  A count of 453 is 0x01C5, little-endian C5 01. Count words, not transactions
  or command bytes, when comparing the trace with UART.

## Capture C: Startup Configuration, Only After A/B

Use the same SPI settings, but trigger on D0 falling and select at least
100 ms (500 ms selected for this session). Arm the analyzer before an agreed
hardware NRST pulse. This restarts
the application and increments its reset counter; it is not a passive read.
No Flash download, EEPROM restoration or option-byte change is needed.
Ordinary FIFO reads also trigger D0 falling: if one arrives before reset, the
capture is not startup evidence. Check for the reset command/configuration
sequence and a matching UART boot log. Do not automatically repeat resets on
ST-Link connection errors; first verify NRST and a reliable reset procedure.
For the verified manual method, hold the board's SW2 continuously, arm the
analyzer while held, then release SW2 only once acquisition is waiting.
Prepare a bounded UART capture that covers the release, not just setup time;
if it expires first, document the missing synchronized boot log rather than
claiming alignment. The SPI trace itself can establish the initialization
sequence. Do not keep restarting solely to recover a missing UART log.

Confirm writes and readbacks on the actual wires, especially:

| Register | Address | Value |
| --- | --- | --- |
| ACT_INACT_CTL | 0x27 | 0x3F |
| FIFO_CTL | 0x28 | 0x0A |
| FIFO_SAMPLES | 0x29 | 0xC2 |
| INTMAP1 | 0x2A | 0x10 |
| INTMAP2 | 0x2B | 0x04 |
| FILTER_CTL | 0x2C | 0x51 |
| POWER_CTL | 0x2D | 0x02 |

A one-byte register write has MOSI 0x0A, address, value. A one-byte register
read has MOSI 0x0B, address, dummy; the third MISO byte is the result.
IDs at 0x00/0x01/0x02 should be AD/1D/F2. Keep the whole startup trace, not
only a screenshot of the last successful register read.

## Evidence and Next Decisions

In DSView use File > Save for the raw .dsl recording, and File > Export > VCD
for an independently parseable digital waveform. For SPI, also export both
MOSI and MISO columns from the decoder's list viewer to CSV/TXT. Session
settings alone are not captured data. Retain original files and record sample
rate, threshold, enabled channels, firmware hash, trigger and any resets.
Keep recordings under ignored .local/captures or provide their actual paths.
If VCD is not offered by the installed build, keep the raw .dsl file; CSV
export is also acceptable. Do not require a conversion merely to inspect
this build's packed per-channel data. The first measurements use byte-bin
timing directly from DSL and do not decode undersampled SPI traffic.

First compare IRQ timing, SPI clock/configuration, FIFO word count and UART.
If the same low rate persists with correct transactions, a separate temporary
data-ready experiment or reference sensor comparison may be warranted. Such
an experiment must be explicitly marked non-production and restored afterward.
Do not declare a defective/counterfeit sensor from timing evidence alone.
Supply-voltage/ripple measurements require an analog instrument, not this
digital trace. Base-station receipt and current acceptance remain pending.

## Sources

- [DSLogic U3Pro16 data sheet](https://www.dreamsourcelab.com/doc/DSLogic_U3Pro16_Datasheet.pdf)
- [DSView user guide](https://www.dreamsourcelab.com/doc/DSView_User_Guide.pdf),
  installed ug31.pdf sections 2.2-2.5 and 2.8-2.9 reviewed for this procedure.
- [ADXL362 data sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl362.pdf)
- Project pin review: HARDWARE.md; actual configuration: Platform/Src/sensor.c.
