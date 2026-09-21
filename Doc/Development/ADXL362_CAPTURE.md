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

## Temporary DATA_READY Test

`RFID_SENSOR_ODR_TEST` is OFF in all normal presets. It requires Diagnostics
and rejects simultaneous HOLD_AWAKE. Build into a separate directory:

```powershell
cmake --preset Diagnostics-local -B build/odr-test -DRFID_SENSOR_ODR_TEST=ON
cmake --build build/odr-test --parallel 1
```

This image is NOT usable production firmware. entry.c bypasses app_init and
app_poll entirely: no classification, RF reporting, checkpoints or EEPROM
writes. It polls PB1 with STOP disabled, services the watchdog and stops the
sensor after at most 240 seconds. The final idle loop is not normal operation;
restore the normal image explicitly. It does not automatically reflash itself.

Initialization verifies the ordinary sensor configuration, then enters standby,
disables FIFO, maps only DATA_READY to INT2, and resumes measurement. Readbacks
must confirm FIFO_CTL=00, INTMAP2=01, FILTER_CTL=51, POWER_CTL=02. A read of
XDATA_L clears DATA_READY. Only a low-to-high observation is counted, with
bounded stuck-high/no-ready detection and SPI errors propagated. UART reports
cumulative counts and both SysTick and RTC spans every ten seconds. Count rate
is (n-1)/first-to-last span, not n/span. MCU polling quantizes timestamps;
the analyzer must independently check the physical INT2 rising-edge periods.

Keep D0..D4 wired as above. Use Stream, 1 MHz / 50 s, single, internal clock,
1.5 V, no filter; click Instant only after UART confirms test configuration.
One MHz is sufficient for millisecond IRQ/CS timing, NOT 4 MHz SPI decoding.
Keep a new raw DSL for each attempt. An all-low D4 is missing IRQ evidence,
not zero sensor ODR. Check probe contact before repeating.

Before each download, back up EEPROM and option bytes under reset. Program
only application Flash, verify, and reset. After acquisition, read back the
complete EEPROM and options before restoring normal firmware; require exact
equality with that attempt's before-test snapshots. Restore the previously
hashed normal image, verify, and confirm ordinary FIFO/checkpoint UART logs.
Never restore EEPROM or change protection to work around a connection issue.
Elapsed time during this bench experiment is not classified or persisted.

Offline edge analysis (does not decode SPI or assume that CS is DATA_READY):

```powershell
python -B Tools/analyze_adxl362_timing.py capture.dsl --output capture.json
python -B -m unittest discover -s Tools -p "test_analyze_adxl362*.py" -v
```

## Temporary ODR Sweep

Add the default-OFF RFID_SENSOR_ODR_SWEEP switch only to the bench build:

```powershell
cmake --preset Diagnostics-local -B build/odr-sweep -DRFID_SENSOR_ODR_TEST=ON -DRFID_SENSOR_ODR_SWEEP=ON
cmake --build build/odr-sweep --parallel 1
```

It requires ODR_TEST and thus Diagnostics. The normal presets are unchanged.
Six 25/50/100 Hz cycles run for ten seconds per phase, with two seconds in
standby between phases. Each phase initializes the sensor, configures it in
standby and verifies FIFO_CTL=00, INTMAP2=01, FILTER_CTL=51/52/53, POWER_CTL=02.
The range, HALF_BW bit and SPI speed do not change. No register outside the
documented user range is accessed. This test does not use external clock/sample
inputs. All application/EEPROM/RF work remains bypassed.

After logging the configuration, read XDATA_L to clear any pending readiness,
then measure without any UART printing until the sensor is stopped. The
SWEEP phase marker plus DRDY config/END reports provide nominal ODR, register
readback, count and MCU/RTC spans. TEST_ONLY messages are emitted every phase;
they are not in themselves MCU reset evidence. SWEEP BEGIN is emitted once.
The eighteen phases take about 217 seconds normally and finish in standby;
errors stop the sweep early. The final idle loop still requires normal-image
restoration. Host tests simulate every phase and failure/wrap cases; the field
test is restored once enough waveforms are captured, without waiting for all
eighteen phases. Watchdog service continues during the standby gaps.

Use the same 1 MHz / 50 s Stream/Instant acquisition and per-attempt backup /
restore process above. Capture while the sweep is already running; the repeated
sequence permits complete examples of all three phases within fifty seconds.
Identify phases using UART markers/timestamps and sequence, not measured-rate
clustering alone. Do not operate reset, power, probes or the Start button during
acquisition. Keep raw files under distinct names.

```powershell
python -B Tools/analyze_adxl362_timing.py sweep.dsl --segments --output sweep.json
```

Segments split at IRQ gaps over 500 ms. Raw results are retained. Report steady
rates after applying the SAME 500 ms head / 100 ms tail trim to every phase;
ignore phases whose complete bounds are not captured or that lack enough
remaining edges. Sensor reinitialization and mode switching can cause initial
short intervals. The settling exclusion exceeds the datasheet's 4/ODR startup
time even for the observed slow 25 Hz setting; it is not selected by whether
an interval makes the desired frequency. Do not include standby gaps in a
global rate. Inspect individual CS acknowledgements inside steady IRQ pulses.

## Temporary External Clock A/B Test

RFID_SENSOR_EXTCLK_TEST is default-OFF, requires ODR_TEST/Diagnostics, and is
mutually exclusive with ODR_SWEEP. The normal firmware and protocol do not
change. Build into a separate directory:

```powershell
cmake --preset Diagnostics-local -B build/extclk-test -DRFID_SENSOR_ODR_TEST=ON -DRFID_SENSOR_EXTCLK_TEST=ON
cmake --build build/extclk-test --parallel 1
```

V3.8 already connects MCU PB0 to ADXL362 INT1 / TP18. Add analyzer D5 there
with the board unpowered, retaining D0-D4 and common ground. Never connect a
second clock source. INT1 is an INPUT in external-clock mode, not a clock
output from the ADXL362. PB0 has no timer-output alternate function (DS10184
Table 17). RM0377 Figure 1 places GPIO on the CPU IOPORT path, outside the DMA
bus. The bench therefore uses TIM2 update interrupts, NOT PWM or GPIO DMA.

HSE/PLL-derived 32 MHz timer / 500 generates 64000 interrupts/s. The short
priority-0 handler alternates PB0 set/reset for a nominal 32 kHz clock.
SysTick is lower priority; RTC/USART/external GPIO interrupts are disabled in
this bench. Record total edges and maximum observed timer count at the GPIO
write; a value over 200 timer ticks aborts. This counter is only a latency
guard: it cannot prove no whole timer periods were missed. Physical D5 timing
is mandatory. No STOP, application, RF, classification or EEPROM writes run.

For every phase, first park PB0 high-Z and initialize the sensor. Enter
standby, disable INTMAP1 and read it back, disable FIFO, map DATA_READY to INT2
and confirm FILTER_CTL=51. Only after these readbacks may PB0 start driving.
Start the external clock BEFORE selecting POWER_CTL=42; internal phases use
POWER_CTL=02 and keep PB0 high-Z. After 600 ms settling, read XDATA_L and
verify INT2 low after 1 ms, allowing at most three clearing attempts. A new
sample may arrive during the read; do not start an unarmed measurement and
misdiagnose its latched ready level as a stuck pin. Count ready edges for ten
seconds without logging inside that measurement. On failure record the pin,
arming state, STATUS and INTMAP2 before cleanup. Stop the
sensor into internal-clock standby before stopping/releasing PB0. Error paths
also release PB0 and terminate the test; do not reinitialize while driving it.

Twenty-four alternating internal/external phases have two-second standby
gaps; normal completion takes about 304 seconds and leaves standby/high-Z.
The ordinary sensor_init briefly restores the usual INT1 mapping before each
phase, while PB0 is high-Z. Do not mistake isolated activity transitions on D5
or initial long DATA_READY pulses during settling for clock/steady data.

At FILTER_CTL=51, the ADI formula gives ODR = reference_clock / 2048:
32000 Hz predicts 15.625 Hz and 64 ms, NOT 25 Hz. Use the independently
measured D5 frequency in this formula. Internal phases provide the same-board,
same-image control. Compare each interval's reference-edge count as well as
mean rates, and verify timely SPI acknowledgement of every retained IRQ.

Acquire six channels in Stream/Instant at 1 MHz / 50 s, single, 1.5 V,
unfiltered/internal analyzer clock. This observes reference-clock continuity
and DATA_READY, not 4 MHz SPI bytes or sub-microsecond jitter. Follow with a
50 MHz / 100 ms Buffer capture during an external phase to inspect individual
reference high/low widths and SPI activity. Inspect D5 frequency, missing or
extra edges and duty cycle before using the A/B result. A reference waveform
failure makes the experiment inconclusive, not evidence against the sensor.

```powershell
python -B Tools/analyze_adxl362_extclock.py extclk.dsl --output extclk.json
```

The analysis retains short startup groups, reference bursts and partial
phases. The fixed 500 ms/100 ms trims are additional to firmware settling;
only complete steady phases with adequate data qualify. Match them to UART
EXTCLK phase/config/END markers; absence of D5 alone cannot distinguish an
internal phase from a disconnected probe. The 32 kHz digital continuity gate
requires frequency within 1%, periods within 25%, and both pulse levels between
25% and 75% of the expected period. These are diagnostic rejection thresholds,
not ADI electrical specifications or analog signal-integrity acceptance.

Use the same fresh full backup, per-download EEPROM/options hashes and exact
normal-image restoration process above. No hardware acceptance is claimed by
building this image. Do not apply this high-CPU test clock to production or
compensate behavior counts based on the experiment.

## Sources

- [DSLogic U3Pro16 data sheet](https://www.dreamsourcelab.com/doc/DSLogic_U3Pro16_Datasheet.pdf)
- [DSView user guide](https://www.dreamsourcelab.com/doc/DSView_User_Guide.pdf),
  installed ug31.pdf sections 2.2-2.5 and 2.8-2.9 reviewed for this procedure.
- [ADXL362 data sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl362.pdf)
- Project pin review: HARDWARE.md; actual configuration: Platform/Src/sensor.c.
- [STM32L051 DS10184](https://www.st.com/resource/en/datasheet/stm32l051t6.pdf),
  Table 17 PB0 alternate functions.
- [STM32L0x1 RM0377](https://www.st.com/resource/en/reference_manual/rm0377-ultralowpower-stm32l0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf),
  Rev 10 Figure 1 / page 49, CPU IOPORT versus DMA bus architecture.
