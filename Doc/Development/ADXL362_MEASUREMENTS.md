# ADXL362 Logic-Analyzer Measurements

Date: 2026-09-21. Sampling-rate acceptance remains NOT PASSED.
No firmware change, download or agent-issued board reset was made for captures
A/B. The user corrected the INT2 probe connection between the low-rate
recordings. The later startup-reset attempt is documented separately below.

## Evidence

DSLogic U3Pro16 / DSView 1.3.2, Stream / single / immediate capture,
1 MHz internal sampling clock, 1.5 V threshold, no filter, channels D0-D4:
CS, SCK, MOSI, MISO, INT2. The session stores 50000896 samples per channel,
or 50.000896 seconds. This rate is NOT suitable for decoding 4 MHz SPI.

Corrected recording supplied as DSLogic U3Pro16-la-260921-152326.dsl:

- SHA256: 74DD03753694FD1B7238F5C1B7CC0012988146958D13A14C85A4861E0F15253A
- Ignored evidence copy: .local/captures/20260921-152326-adxl362-timing.dsl
- Matching receive-only UART: .local/captures/20260921-152316-COM27.txt
  and its timestamped .txt.jsonl companion.
- Header trigger time: 1789975406277 Unix milliseconds, 15:23:26.277 UTC+8.

The earlier 150510 recording had all 6250112 bytes of D4 equal to zero.
It is not valid INT2 timing evidence. Its SHA256 was
9E091606DCC332FF611990450C89AF8479815E4E84B6CFBECE86551BF582FA9A.
The user subsequently confirmed a wrong/unreliable connection and corrected it.
Its CS activity and UART agreed on approximately 7.65-second read spacing,
but no sensor-interrupt conclusion is drawn from that flat D4 channel.

## Method

Read the DSL ZIP's header and session JSON, then each channel's L-n/0..2
blocks in numeric order. Each channel has 6250112 packed bytes for 50000896
samples. For the timing below, locate nonzero bytes on INT2 and non-FF bytes
on CS, grouping activity separated by more than 100 ms. Byte-bin times have
8-microsecond resolution; no within-byte bit ordering is needed for these
reported intervals. This is coarse timing analysis, not an SPI decoder or
an independent validation of every DSView archive-format variant.

## Corrected Trace

| Event | INT2 activity begins (s) | First CS activity (s) | UART FIFO words |
| --- | ---: | ---: | ---: |
| FIFO watermark | 4.308296 | 4.310952 | 453 |
| Checkpoint partial drain | None | 10.876344 | 387 |
| FIFO watermark | 18.469408 | 18.472112 | 453 |
| FIFO watermark | 26.107624 | 26.110272 | 453 |
| FIFO watermark | 33.750688 | 33.753240 | 453 |
| FIFO watermark | 41.395808 | 41.398496 | 453 |
| FIFO watermark | 49.038536 | 49.041200 | 453 |

The four uninterrupted INT2 intervals are 7.638216, 7.643064, 7.645120 and
7.642728 seconds, mean 7.642282 seconds. The first 14.161112-second INT2 gap
contains the separate partial drain and must not be treated as one FIFO fill.

INT2 remains high for approximately 3.30-3.46 ms. First CS activity follows
INT2 by approximately 2.55-2.70 ms. Therefore the observed multi-second
difference from the nominal six-second pace is not explained by a long
delay between this interrupt and the first host SPI transaction.

The partial drain matches UART t=180004, stage=2, elapsed=660, reset=34.
Subsequent normal drains match t=187598, 195238, 202883, 210527 and 218172.
The recorded checkpoint reported sensor/radio/storage errors 0/0/0.

## Interpretation and Limits

The analyzer independently confirms a roughly 7.64-second hardware-interrupt
pace, consistent with MCU and host timing. Given the reported 453 words
(151 XYZ samples) and complete intervening FIFO drains, the inferred rate is
151 / 7.642282 = approximately 19.76 XYZ samples/s, not the requested 25 Hz.
This was initially an inference using UART word counts. Capture B below
independently confirms a later 453-word drain on the SPI wires. The earlier
low-rate recording is still unsuitable for SPI decoding; do not claim that
each of its individual drains has been byte-verified.

The low-rate measurement alone does not identify the cause or establish a
sensor defect. Do not resample, change classifier thresholds or shorten the
reporting window to conceal it. Base-station and current tests remain pending.

## Capture B: SPI FIFO Drain

Original: DSLogic U3Pro16-la-260921-153828.dsl.

- SHA256: 616603F5C0E21C350A7571041C0A48655C9B0BF1AC295A121E2708DE1D461A44.
- Evidence copy: .local/captures/20260921-153828-adxl362-spi.dsl.
- Decoded data: the same basename with .json instead of .dsl.
- Concurrent receive-only UART: .local/captures/20260921-153812-COM27.txt
  and .txt.jsonl. Regular reads report 453; the later checkpoint reads 384
  and reports stage=3, elapsed=360, reset=34, errors=0/0/0.
- Buffer, single, D4 rising, trigger position 10 percent; 50 MHz internal
  clock, 1.5 V threshold, no filter, RLE disabled, D0-D4.
- 5000192 samples = 100.00384 ms. INT2 rises at sample 500002, exactly the
  archive's trigger position; it falls at sample 673088.

Mode-0, active-low CS, MSB-first byte decoding gives:

| Transaction | Start / end (ms from record start) | MOSI prefix | MISO payload |
| --- | --- | --- | --- |
| Status | 12.71724 / 12.73664 | 0B 0B 00 | 07 |
| FIFO entries | 12.85198 / 12.87588 | 0B 0C 00 00 | C5 01 = 453 words |
| FIFO chunk 1 | 13.43500 / 15.74332 | 0D, then 510 dummy bytes | 510 data bytes |
| FIFO chunk 2 | 16.20926 / 18.00454 | 0D, then 396 dummy bytes | 396 data bytes |

There are 7320 rising clock edges: (3 + 4 + 511 + 397) * 8. All transactions
contain complete bytes. The concatenated FIFO payload is 906 bytes, 453 words,
151 each of X/Y/Z and zero temperature entries. It begins with X; every tag
continues X/Y/Z, including the chunk boundary, and sign extension is valid.
The first four XYZ sets are (22,87,561), (23,86,563), (23,87,563), (19,88,563).
These are raw counts, not calibrated acceleration measurements.

Within-byte clock periods are 12 or 13 analyzer samples (240/260 ns).
The 6405 within-byte periods average 250.208 ns, approximately 3.997 MHz.
This is consistent with configured 4 MHz; do not imply calibrated frequency
accuracy from the number of displayed decimal places. Clock high is 120-140 ns.
Neither data channel changes within one analyzer sample of a rising clock edge.
This digital check is not analog signal-integrity, voltage or ripple validation.

INT2-to-first-CS latency is 2.71720 ms, IRQ high time 3.46172 ms, and the full
status/count/drain sequence lasts 5.28730 ms. CS is high for 465.94 us between
FIFO chunks. Status 07 does not report FIFO overrun or user-register error.
This one capture supports correct regular FIFO transport, not the separate
900/1024-byte stress cases, runtime stack safety or all future transactions.

### Reproduction

Tools/analyze_adxl362_dsl.py reads the ZIP directly; NumPy is required.
It rejects undersampled, truncated, ambiguous-edge and odd FIFO-byte data.
It supports only explicit v3 L-n/block channel data, not all DSView formats.
Ten synthetic tests cover byte/sign/tag decoding, incomplete transfers,
ambiguous data edges, archive block/bit order and unsupported input.

```powershell
python -B Tools/analyze_adxl362_dsl.py .local/captures/20260921-153828-adxl362-spi.dsl
python -B -m unittest discover -s Tools -p test_analyze_adxl362_dsl.py -v
```

The byte packing was cross-checked against DSView v1.3.2
[LogicSnapshot::get_sample_self](https://raw.githubusercontent.com/DreamSourceLab/DSView/v1.3.2/DSView/pv/data/logicsnapshot.cpp).
LSB-first archive unpacking also reproduces the exact trigger sample and
complete expected SPI commands. This is distinct from MSB-first SPI bytes.
Command format and FIFO word encoding follow the
[ADXL362 data sheet, Serial Communications](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL362.pdf).

## Startup Capture Attempt

After user approval, DSView was armed at 50 MHz / 500 ms, D0 falling only,
10 percent trigger position. STM32CubeProgrammer 2.19.0 was called once with
`-c port=SWD sn=<local-probe> freq=100 mode=HWRSTPULSE -run`.
It identified the probe and 2.93 V target supply, then exited with code 1:
`Unable to get core ID` / `ST-LINK error (DEV_TARGET_CMD_ERR)`.
No download, erase, memory write or option-byte command was issued.

The analyzer display showed another normal FIFO drain, not initialization.
UART .local/captures/20260921-154547-COM27.txt recorded continuous FIFO times
1515262 through 1553535 with no boot log or time reset. Therefore a successful
board restart and startup-register capture are NOT established. No automatic
reset retry was performed; NRST connectivity/manual-reset availability was
requested. The current DSView trace should not be labelled a startup capture.

The user then confirmed an NRST connection and a manual reset button. A
hold-reset/arm/release procedure was proposed. After the user reported the
button held, receive-only UART 20260921-155236-COM27.txt / .txt.jsonl recorded
boot reset=38 at 15:52:38, a FIFO read at 15:52:45, boot reset=39 at 15:53:06
and boot reset=40 at 15:53:29 (UTC+8 host receipt times). Each boot recovered
stage=3 / elapsed=1080 and reported errors=0/0/0. No new analyzer acquisition
or ST-Link command was issued in this attempt. The user was asked to release
the button and clarify continuous holding/button identity, since execution
is inconsistent with NRST continuously asserted. The supplied V3.8 netlist
connects SW2.1 to GND and SW2.2 to NRST/CN1.2/U1.7. This does not establish
the physical contact condition or the cause of the repeated restarts.
The user subsequently clarified that the button had been released briefly;
the earlier observation is therefore not evidence of a faulty reset button.

## Capture C: Startup Configuration Verified

The user continuously held SW2 while the analyzer was armed, then released
it on request. No further ST-Link command, download or option change was used.
Original: DSLogic U3Pro16-la-260921-155625.dsl.

- SHA256: E6969BA8AF2D52CCC37D53B351E049F880D862E7AC6E59B7D8E2D586D466843B.
- Evidence copy: .local/captures/20260921-155625-adxl362-startup.dsl.
- Decoded data: same basename with .json instead of .dsl.
- 50 MHz, 25000960 samples (500.0192 ms), Buffer/single, 1.5 V, no filter,
  D0 falling only, 10 percent trigger position. First CS falling edge is
  sample 2500106, exactly the recorded trigger position.

All 33 transactions decode as complete mode-0, approximately 4 MHz transfers:

1. Official driver initialization reads PARTID (02), returning F2.
2. Soft reset writes 0A 1F 52; the next register transaction begins
   10.79508 ms after reset CS rises.
3. Reads at 00/01/02 return AD/1D/F2 respectively.
4. Fourteen configuration writes each have an immediately matching readback,
   in the following firmware order. All values are hexadecimal.

| Register address | Written and read back |
| --- | --- |
| 20 / 21 / 22 | 64 / 00 / 06 |
| 23 / 24 / 25 / 26 | 64 / 00 / 06 / 00 |
| 27 | 3F |
| 2A / 2B | 10 / 04 |
| 28 / 29 | 0A / C2 |
| 2C / 2D | 51 / 02 |

The final POWER_CTL read ends at 65.02954 ms from record start. The trace has
no FIFO transactions, as expected within the short startup window. The
complete decoded order and readback bytes were asserted against sensor.c's
configuration table, not just inspected in a screenshot.

The 90-second UART preparation log 20260921-155503-COM27.txt ended before
the user released reset; it does NOT contain this boot. The later receive-only
20260921-155751-COM27.txt log confirms reset=41, stage=4, elapsed=60 and
errors=0/0/0 at its checkpoint. Subsequent 453-word reads at t=67598, 75230,
82875, 90512 still have 7.632/7.645/7.637-second intervals. No claim of
synchronized UART boot timing is made for this capture.

## Remaining Sampling-Rate Investigation

Regular SPI transport and startup configuration now have independent bus
evidence. The approximately 19.76 XYZ sets/s inference remains inconsistent
with the requested 25 Hz; its cause is still unresolved. Next evidence should
include analog VS/VDDIO supply/ripple measurements and, if needed, a clearly
temporary data-ready measurement or reference-board comparison. Do not change
the production classifier or reporting protocol to compensate silently.
