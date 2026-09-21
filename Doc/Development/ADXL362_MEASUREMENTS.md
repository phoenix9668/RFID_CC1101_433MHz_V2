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

## Temporary DATA_READY Experiment

The user reported collars with about 2800-3000 total classifications per hour,
while others approach 3600. This is consistent with the observed rate deficit,
but it does not establish the same cause for all collars.

A new, default-OFF RFID_SENSOR_ODR_TEST entry bypasses all application work.
The experiment keeps FILTER_CTL=51, POWER_CTL=02, disables FIFO (28=00), and
maps DATA_READY alone to INT2 (2B=01). GPIO PB1 is polled; a read of XDATA_L
acknowledges each event. STOP, classification, RF and EEPROM writes are absent.
The linked bench ELF has no app_init/app_poll, storage or EEPROM programming
symbols. SPI and pin waits are bounded; a run ends in standby within 240 s.
The actual runs below were interrupted after acquisition to restore normal
firmware, so the 240-second termination path was host-tested, not timed on board.

- Test HEX SHA256: 25CF36C42BE91AEEE3D9123666497D4752055075CBCCAEDFE4916610C2FBE52F.
- Test ELF SHA256: 180C06023A08A21D2138C1CE3DCF12A871B597B1FB113EB1F5664B4FB2BDCBF0.
- Restored normal HEX SHA256: 6A823BA02515CF614E70F79850F2E62D5BDE56AD750DAEA67C706877982D6866.
- Restored normal ELF SHA256: 2E597FAB4C147A9BE897C6BDAD7A4D209D969B9CF81CCAAD322D7CC415E5A68E.

Read-only UR backup 20260921-161051 includes complete 64 KB Flash, double-read
2 KB EEPROM and 20 option bytes. Its Flash prefix exactly matched the existing
26216-byte normal image. Each subsequent experiment took another EEPROM/options
snapshot immediately before download. Logs and binary backups remain private
under .local/backups; no mass erase, EEPROM restoration, identity write or
option-byte modification was performed. Normal operation between experiments
continues to update the journal, so compare before/after within each attempt.

### Attempt D1: CS Evidence, Missing D4

Original DSLogic U3Pro16-la-260921-162447.dsl; evidence copy and JSON:
.local/captures/20260921-162447-adxl362-drdy.*.
SHA256 F77903E4E2570B91E508EF6483904F2CED4E9F0F7D926A4867671A59418F5CB7.
Stream/Instant, 1 MHz, 50000896 samples (50.000896 s), 1.5 V, no filter/RLE,
internal clock, explicit L-0 through L-4 blocks present and complete.

- D0 contains 987 complete CS pulses, 19-20 us wide. First/last falling edges
  are samples 44327 and 49986866. Across 986 intervals, mean period is
  50651.662 us and frequency is 19.742689 Hz. Period range: 49995-52001 us.
- D4 contains ZERO high samples. It provides no independent DATA_READY period;
  do not report its rate as zero or substitute CS edges for sensor IRQ evidence.
- UART 20260921-162326-COM27.txt confirms IDs AD/1D/F2, revision 03, test
  readbacks and status=0. At n=1975, SysTick span=99978 ms, RTC span=99980 ms:
  rates (n-1)/span are about 19.744 Hz. No restart appeared in this test log.
- Test-before/test-after/pre-normal-release EEPROM SHA256 all equal
  C99E16DE7CED8292BE22A468B7BA55487713EB213B1750A10A407E05991E3EAA.
  Corresponding options SHA256 all equal
  7FC6E8272A2DE908091CDC536B180CE557CAE4D13B4120250A50D564507821D6.
- Normal image restoration verified; later UART 20260921-162742-COM27.txt
  has regular 453-word drains and checkpoint reset=43, errors=0/0/0.

The analyzer's independent CS timing agrees with the MCU clock spans. This
supports a deficit before classification even without FIFO/STOP/application
work. Missing D4 prevents treating D1 alone as physical DATA_READY proof.

### Attempt D2: Restart-Contaminated Acquisition

After the user reported D4 reconnected, the same test image was used again.
UART 20260921-163218-COM27.txt records normal test counts up to n=1777 over
89956 SysTick / 89961 RTC ms, followed by two new TEST_ONLY boot messages at
about 16:34:16 and 16:34:18 host local receipt time. No agent ST-Link command
was issued during that interval. Cause is unconfirmed; user requested a new
acquisition rather than confirming a reset/power action. Do not infer a hardware
fault or treat this attempt as a continuous timing run. No raw D2 archive was
received before reacquisition; only its UI and UART observations are retained.

Full EEPROM/options matched before/after and before normal release again
(before-odr2/after-odr2/restore2 files). The exact normal image was restored and
verified; UART 20260921-163553-COM27.txt shows 453-word reads at 22848, 30492,
38137 and 45785 ms. A third attempt was requested to obtain a continuous trace.

### Attempt D3: Physical DATA_READY Rate Verified

Analyzed original: DSLogic U3Pro16-la-260921-163735.dsl, saved at 16:38:46.
The archive trigger/record-start timestamp is 16:37:35.950 host local time.
Evidence copy/JSON: .local/captures/20260921-163735-adxl362-drdy.*.
SHA256: 00676EF8ADC037EA569122703581E7238B3F898D588D759DFC202FA13F2F6D30.
Stream, Instant, 1 MHz, 50000896 samples (50.000896 s), threshold 1.5 V,
no RLE/filter/external clock. There is no start/end-truncated CS or IRQ pulse.

| Measurement | Result |
| --- | --- |
| Complete INT2 rising/falling pairs | 988 |
| First / last INT2 rise sample | 10692 / 49984408 |
| INT2 period min / mean / max | 50516 / 50631.931 / 50717 us |
| INT2 rising-edge frequency over 987 intervals | 19.750382 Hz |
| Complete CS transactions | 988 |
| CS duration | 19-20 us |
| IRQ rise to CS assertion min / mean / max | 105 / 1096.845 / 3371 us |
| CS end to IRQ low | 18-43 us |
| IRQ high duration | 146-3426 us |

All 988 pairs satisfy IRQ rise < CS fall < CS rise < IRQ fall. Each observed
DATA_READY pulse is acknowledged well before the next sample; there are no
double-period gaps in this trace. Within consecutive ten-second subwindows,
mean periods are 50648.75, 50641.92, 50626.40, 50644.17, 50598.85 us. Timing
uses the analyzer clock, not MCU polling timestamps. SPI bytes are NOT decoded
from this 1 MHz trace. Configuration/REVID=03 is corroborated by the test UART
readback and the previously verified high-rate production startup trace.

UART 20260921-163644-COM27.txt shows the sole test boot at 16:37:05.817 and
uninterrupted counts through n=2371 / 119982 SysTick ms; no boot occurs across
the analyzed 50-second capture. Follow-on 20260921-163935-COM27.txt has n=3754
over 189964/189972 ms. The test reboot at 16:40:19 follows the agent's post-test
UR snapshot, not spontaneous reset. Normal restoration at about 16:40:31
reports stage=5, elapsed=720, reset=45, errors=0/0/0; first normal drain has
453 words. D3 EEPROM before/after/pre-release SHA256 all equal
DE8E19381F0890652B76DB4B7AE1FDD296B3F1BBB45A384C8278F3B402E4E4BA;
options match the original 7FC6E827...821D6 hash. Exact normal image restored.

DSView also showed a subsequent acquisition starting at 16:38:51 during the
same test boot. After the user reported saving, the latest received file was
still 163735.dsl. The table above deliberately uses that actual archive, not
the later UI trace. The second D3 waveform is not needed for this conclusion.

The sensor's physical DATA_READY output is about 21.00 percent below nominal
25 Hz on this board under this configuration. With the unchanged one-output-
per-25-XYZ-sets rule, the steady-state prediction is
19.750382 / 25 * 3600 = 2844.06 classifications/hour, before startup/history
latency or other losses. This is quantitatively consistent with the user's
2800-3000 field totals, not a direct one-hour end-to-end acceptance test.

The deficit persists with FIFO, STOP, classification, EEPROM and radio work
removed. Those paths therefore are not necessary to reproduce this board's
slow sample-generation cadence. This does not prove all field collars share
one cause, nor certify the sensor defective/counterfeit. Internal sensor clock,
analog supply conditions and part/board differences remain to be separated.
The ADI datasheet states ODR scales with its internal time base; the typical
clock distribution is not a guarantee that this observed 21 percent deficit
is acceptable. Next useful test: explicit temporary 25/50/100 Hz selections,
checking whether measured rates all scale by about 0.79, then compare a known
good board and measure VS/VDDIO at the sensor. Do not change the production
algorithm, invent samples or normalize counts to 3600 without an agreed design.

## Capture E: 25/50/100 Hz Sweep

The user requested the next diagnostic step. A default-OFF ODR_SWEEP option
selects repeated ten-second 25/50/100 Hz phases with two-second standby gaps.
Each phase resets/initializes the sensor, changes only the ODR selection while
in standby, verifies configuration, clears stale readiness after the initial
log, and measures with NO UART printing in the measurement loop. Range remains
4g, HALF_BW=1, FIFO disabled, INT2=DATA_READY, POWER_CTL=02, SPI=4 MHz.
No production algorithms, protocol, storage layout or peripheral timing change.

### Software and Backup Evidence

- Sweep HEX SHA256: 543E0BBC5A1A66460CAAC91FF8E58C5F068FBF2FE2C53DB699086C0EE7364B83.
- Sweep ELF SHA256: 98BF0439003450E1052AE6E7446CB50E02E0A6AFCF926DB210822F2410A486BD.
- Sweep image: Flash 14784 bytes, RAM 2576 bytes including reserved stack.
- Clean build/odr-sweep-verify Debug/Release/Diagnostics builds passed; all
  three BIN files are byte-identical to the previous build/odr-verify outputs.
- Six CTests and all three CSV comparisons pass (15738 identical outputs).
  Tests cover all eighteen phases, nominal/slower mock clocks, tick wrap,
  standby ODR writes, readback mismatch, no in-phase log and early error stop.
  Nineteen Python decoder/timing tests pass, including raw/steady separation.
- The first host runs exposed errors in the new test doubles (disabled fault
  injection and pending-ready advancement) and an incorrect coarse expected
  count for a slow mock clock. These were corrected before any board download.
- Complete private UR backup: .local/backups/20260921-165201-<probe>/.
  Flash prefix again matches the hashed original normal image. A new EEPROM /
  options snapshot was taken in the download connection immediately before
  programming the sweep. Only application Flash was programmed and verified.

### Physical Waveform

Original: DSLogic U3Pro16-la-260921-165326.dsl, saved 16:54:24.
SHA256: B13F1FF0D4E1D0CC344A821D38B0F2773C5681E3A42328E22BCD96DD30D9F4AB.
Copy: .local/captures/20260921-165326-adxl362-sweep.dsl.
JSON: same basename with -steady.json suffix (raw and trimmed results retained).
Stream/Instant, internal 1 MHz, 50000896 samples (50.000896 s), 1.5 V,
no filter/RLE. Capture start is 16:53:26.435 local host time. SPI data bytes
are NOT decoded at this sample rate; nominal selection comes from firmware
readback and independently matched UART phase markers, not rate inference.

UART .local/captures/20260921-165242-COM27.txt / .txt.jsonl shows phase IDs
2,3,4,5 at about 16:53:28.55, 16:53:40.58, 16:53:52.73, 16:54:04.80.
These align with the waveform's four segment starts below. Each phase has
the expected FILTER_CTL readback 53,51,52,53 and status=0. No unexpected
SWEEP BEGIN appears during capture. The per-phase TEST_ONLY lines are not
reboots. Each ordinary sensor_init temporarily selects the baseline FIFO
configuration before the DATA_READY configuration; its transition is excluded.

| Phase / nominal | Raw first-last IRQ (s) | Raw edges | Steady edges | Steady Hz | Actual/nominal |
| --- | --- | ---: | ---: | ---: | ---: |
| 2 / 100 Hz | 2.039838-12.041935 | 795 | 746 | 79.269014 | 0.792690 |
| 3 / 25 Hz | 14.105485-24.089517 | 199 | 186 | 19.750956 | 0.790038 |
| 4 / 50 Hz | 26.171132-36.180816 | 398 | 372 | 39.502107 | 0.790042 |
| 5 / 100 Hz | 38.237779-48.243078 | 795 | 746 | 79.233530 | 0.792335 |

Raw phase means are 79.383353, 19.831667, 39.661592 and 79.357948 Hz. They
include initial short intervals (first intervals 992, 12924, 2108, 991 us), so
they must not be treated as settled ODR. For ALL four phases, exclude the first
500 ms and last 100 ms, then compute (retained edges - 1)/(last - first).
This fixed time exclusion is larger than the documented 4/ODR settling time
at the slowest tested setting. All four phase boundaries are present in the
capture. Initial waveform evidence is retained rather than hidden as outliers.

Steady period min/mean/max in microseconds:

- Phase 2: 12586 / 12615.269799 / 12657.
- Phase 3: 50548 / 50630.459459 / 50710.
- Phase 4: 25261 / 25315.105121 / 25356.
- Phase 5: 12592 / 12620.919463 / 12648.

For the 2050 retained IRQ pulses, pair each rise with its following IRQ fall,
find CS assertions in that high interval and the matching CS deassertion.
Each is acknowledged before its IRQ falls. Maximum high time is 2164 us;
CS deassertion to IRQ low is 17-43 us. No doubled steady-period gaps occur.
This validates timely service of the captured ready events, not all possible
errors or a long-duration production acceptance run.

### Recovery and Interpretation

Before/after/pre-normal-release EEPROM SHA256 is identical:
4AB3761E603D5F9AC83ED7E7DA51A14668C962DF108D535142068A37010620DE.
All option snapshots equal 7FC6E8272A2DE908091CDC536B180CE557CAE4D13B4120250A50D564507821D6.
The exact normal HEX (6A823BA0...982D6866) was restored and verified. EEPROM,
identity and options were not programmed. The bounded original UART session
ended before restoration, so no normal boot log is claimed from that session.
Follow-on .local/captures/20260921-165618-COM27.txt shows normal 453-word drains
at 38156,45797,53445 ms, then a 387-word checkpoint drain, stage=6, elapsed=240,
reset=47, errors=0/0/0. Full eighteen-phase auto-termination is host-tested,
not a completed on-board run; the board was restored after sufficient capture.

All selected rates are approximately 79.0-79.3 percent of nominal. The result
supports a common sensor time-base scale error rather than an isolated 25 Hz
selection error or an application processing bottleneck. This is an INFERENCE:
the internal oscillator itself was not measured. The ~0.3 percentage-point
spread is reported, not forced into one exact calibration constant.

The ADI datasheet's External Clock section states ODR/bandwidth scale with the
time base; its typical population clock distribution is not an acceptance
limit that makes this ~21 percent deficit normal. Continue with a known-good
same-version collar under the same diagnostic image and measure VS/VDDIO at
the sensor, including ripple. Battery operation or the ST-Link supply reading
does not establish those analog conditions. Do not declare a failed/counterfeit
part, use undocumented trim registers, or silently normalize class counts.
Base-station receipt, real four-hour history and current acceptance stay open.
