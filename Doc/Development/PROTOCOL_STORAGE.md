# Protocol and Persistence Contracts

## Radio Payload

The application payload is exactly 191 bytes, unchanged from the base
CC1101SendHandler. The radio FIFO prefixes it with length 192 and address EF.
Hardware packet CRC remains enabled in addition to the application CRC.

| Offset | Bytes | Field |
| --- | --- | --- |
| 0 | 6 | Device identifier, original byte order |
| 6 | 32 | Non-security random padding |
| 38 | 144 | Six class-major arrays of twelve uint16 counters, big endian |
| 182 | 1 | Next/current history slot after sealing the previous slot |
| 183 | 2 | Raw battery ADC value, big endian; not millivolts |
| 185 | 2 | Reset counter, big endian |
| 187 | 4 | CRC of bytes 0..186, big endian |

CRC is reflected IEEE CRC32, polynomial EDB88320, initial FFFFFFFF, final XOR
FFFFFFFF. Tests use both the 123456789 vector and the mechanically extracted
base encoder with an independent model of the original STM32 CRC peripheral.
Class order and field offsets are not inferred from a rewritten decoder.

The radio tests check every base RF table entry in order, address mode,
sync bytes, 60-byte refill boundaries and the complete 191-byte payload.
No ACK, packet-version field, compressed counter or raw-data diagnostic packet
has been added.

## Internal EEPROM

The MCU has 2048 bytes of internal data EEPROM; there is no external EEPROM IC.
Offsets 000..23F remain untouched, including identifier words 000/004 and
legacy stage/reset/time/history. New code never dual-writes the old layout.

The journal uses 0240..07FF: 16 history slots followed by 30 checkpoint slots,
32 bytes each. The physical base address is 08080000.

| Slot byte | Bytes | Encoding |
| --- | --- | --- |
| 0 | 4 | Sequence, little endian, wrap-aware comparison |
| 4 | 12 | Six counters, little endian |
| 16 | 2 | Elapsed seconds, little endian |
| 18 | 2 | Reset count, little endian |
| 20 | 1 | History stage, 0..11 |
| 21 | 1 | Reserved zero |
| 22 | 1 | Format version 1 |
| 23 | 1 | History 1 / checkpoint 2 |
| 24 | 4 | CRC32 of first 24 bytes, little endian |
| 28 | 4 | Commit marker 38524649, little endian |

A reused slot's commit word is invalidated first. Seven body words are written,
then the commit word last, followed by readback/CRC verification. This is nine
word-program calls including invalidation. The previous complete slot stays
valid on any intermediate failure. An EEPROM timeout/error disables subsequent
writes until reset rather than risking additional stale commits.

Startup scans all slots and chooses the newest valid checkpoint/history.
A history completed after the latest checkpoint starts the next window at zero.
Checkpoints are approximately one minute apart; reset count is checkpointed
at boot. Algorithm filter history and unread FIFO samples are not persisted.
The loss target is approximate and excludes storage faults and oscillator drift.

Legacy migration imports history then publishes a checkpoint. Without any
valid checkpoint, interrupted first imports are retried from the unchanged old
area. Legal stages are 0..11, legal old progress is 0..119 ten-second ticks.
A legacy window with any class over 1500 or total over 1500 is entirely unknown
and is represented as zeros. The 1500 bound permits nominal ODR tolerance but
rejects clearly incompatible old windows; it is not a new report duration.
Invalid stage/progress/current statistics open a new empty current window.
The user approved this policy after the connected board showed progress 252
and window counts around 2832. No private device identifier is recorded here.

## Endurance and Limitations

At one checkpoint/minute, each of 30 slots is reused about every 30 minutes.
The commit word may undergo two erase/program cycles per reuse: approximately
96 cycles/day before boot/checkpoint overhead. Against a 100k-cycle rating this
is roughly 2.85 years at the rated temperature range, not a lifetime guarantee.
Temperature, frequent resets and actual programming physics matter. Consult the
[STM32L051 datasheet](https://www.st.com/resource/en/datasheet/stm32l051c8.pdf);
the high-temperature endurance rating is lower.

This design follows the approved 16+30 slot layout and is not claimed to be
globally wear-optimal. Longer deployment lifetimes require a separately reviewed
record/commit strategy, fewer checkpoints or different nonvolatile storage.
Do not silently change checkpoint interval or legacy data to hide this tradeoff.
