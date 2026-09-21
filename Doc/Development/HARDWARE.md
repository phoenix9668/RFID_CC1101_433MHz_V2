# V3.8 Hardware Review

Basis: the six user-supplied V3.8 exports in PCB/RFID_CC1101_433MHz_V3.8.
Schematic pages were visually reviewed with BOM, netlist, placement and PCB.
Some sheet title blocks retain older V3.7/V3.2 labels; exported source files
are retained unchanged, not relabeled to imply a different hardware revision.

Main parts: STM32L051C8T6, ADXL362BCCZ-RL, E07-433M20S, LP5907-3.0, TP4056.
HSE is 12 MHz; LSE is 32768 Hz; system clock is 32 MHz.
SPI1 and SPI2 use /8 = 4 MHz.

| Function | Pins / Details |
| --- | --- |
| ADXL362 SPI2 | PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS |
| ADXL362 FIFO interrupt | PB1 / INT2 |
| E07 SPI1 | PA5 SCK, PA6 MISO/SO, PA7 MOSI, PA2 CS |
| E07 GDO | PA3 GDO0 active-low packet signal, PA4 GDO2 FIFO threshold |
| Radio power | PB5 controls Q4/Q3 power switching |
| Radio front end | PA1 TX enable, PB2 RX enable |
| Battery | PA0, 10M/10M divider, C12 100 nF |
| LED | PA12, active high |
| USART1 | PA9 TX, PA10 RX, 115200 8N1 in Diagnostics |
| Other UART | PB10/PB11 LPUART unused in production |
| SWD CN1 | 1=3V, 2=NRST, 3=NC, 4=SWCLK, 5=SWDIO, 6=GND |

UART adapter RX connects to PA9, TX to PA10, common ground; use compatible
logic levels and do not back-power the board. NRST is necessary to connect
under reset while the firmware is in STOP.

## Power Ownership

ADXL362 remains powered while collecting. Its CS stays high/output even while
SPI2 clocks and bus pins are parked. FIFO streaming is 450 words, with 4g,
25 Hz, half-bandwidth and the original activity/inactivity configuration:
ACT_INACT_CTL=3F, INTMAP1=10, INTMAP2=04, FIFO_CTL=0A,
FIFO_SAMPLES=C2, FILTER_CTL=51, POWER_CTL=02.

Radio shutdown first disables PA/LNA, masks/clears GDO interrupts, disables SPI,
parks CS/bus/GDO pins without a driven high level, then drops PB5. This avoids
a powered MCU driving the unpowered radio through its I/O. Register programming
is repeated on every power-up. Production does not keep the radio in RX.

ADC calibration, a discarded first sample and ten long-sampling conversions
retain the original raw-value wire unit. The 10M divider and 100 nF capacitor
still require physical settling/accuracy tests; software changes are not proof
of battery-voltage accuracy. MCO, UART and unnecessary LEDs are disabled in
production. STOP entry atomically checks pending work; interrupts do not perform
SPI transfers, storage or algorithm processing.

## Physical Acceptance Checklist

Basic ID/configuration, regular FIFO reads, one resumed-window TX and reset
recovery now have board evidence in VERIFICATION.md. Sampling rate is still
unresolved at about 19.75 Hz; the complete checklist below is not all passed.

- Check ADXL362 AD/1D/F2 IDs and all configuration readbacks.
- Confirm FIFO pace and no overruns at real ODR; test 900 and 1024 byte paths.
- Confirm CC1101 PARTNUM, VERSION and critical registers on the powered module.
- Observe short and long packets, GDO polarity, timeout recovery and RX failures.
- Compare exact packets with the unchanged original base station.
- Run at least 4h20 real time, including a checkpoint-boundary reset.
- Measure sleep current, acquisition charge, TX charge and 20-minute total charge.
- Compare with a functionally equivalent production baseline, with SWD/UART
  disconnected and no watchdog/low-power debug freeze.

These are acceptance gates; recorded results are in VERIFICATION.md. No current reduction or radio range
improvement is claimed before measurements.
