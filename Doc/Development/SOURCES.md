# Source Provenance

## Analog Devices

The two files in Drivers/ADI are unchanged ADXL362 no-OS source at
[commit 0c4ef5cd278555ba4d426b35f87f30102460361f](https://github.com/analogdevicesinc/no-OS/tree/0c4ef5cd278555ba4d426b35f87f30102460361f/drivers/accel/adxl362).
Their original copyright/license headers remain. Platform/no_os_port is
project-owned, not a complete no-OS distribution: it supplies one fixed
descriptor allocation and the small SPI/allocator API subset used by this driver.
Void-return upstream helpers latch transport errors in the adapter.
No failed read is passed to the classifier.

SHA256:

| File | Hash |
| --- | --- |
| adxl362.c | 328A71A98CF0243523559EA2A63771103488BABDCB989064A66BC9845DD305BD |
| adxl362.h | 9F5FCDAA8095EDC54B821D654D43555CE9C3C693BC9D27658D3C7C4F1CA5501F |

FIFO reads are even-sized, at most 510 payload bytes per official call:
900 = 510+390; 1024 = 510+510+4. The official helper has a 512-byte stack buffer.
Configuration follows the [ADXL362 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl362.pdf).

## Texas Instruments

[SWRC021](https://www.ti.com/tool/download/SWRC021) is TI's official CC1100,
CC1101, CC1100E and CC2500 examples/libraries package, version 01.00.00.0F,
dated 2009-03-31. Its ZIP endpoint returned HTTP 403 during this work.
The sample source was NOT obtained or reviewed, and is NOT vendored.

Platform/radio.c is a new project-owned STM32 reference implementation using
the [CC1101 datasheet](https://www.ti.com/lit/ds/symlink/cc1101.pdf) and
[CC1101 errata](https://www.ti.com/lit/swrz020), not an official TI driver.
It waits for SO after CS assertion and reset, bounds all polls, reads dynamic
status consistently, leaves an RX FIFO byte until packet completion, checks
CRC/address/overflow, and powers down after success or failure.

The base's 47-byte 433 MHz table is frozen by Tests/legacy/rf_config.h.
PA table starts with 0xC0; address EF and sync 1234 remain unchanged.
SmartRF Studio is a [configuration reference](https://www.ti.com/tool/SMARTRFTM-STUDIO);
this work does not claim a newly measured or optimized RF setting.
The old WOR register values remain in the table; production uses module power
gating rather than a new WOR behavior.

## STMicroelectronics

The base STM32CubeL0 V1.12.2 HAL/LL/CMSIS contents are retained.
GNU startup is from cmsis-device-l0 v1.9.1,
[commit 9d7674c4560773dc63ed988fa69959296a9c7935](https://github.com/STMicroelectronics/cmsis-device-l0/tree/9d7674c4560773dc63ed988fa69959296a9c7935).
SHA256 startup_stm32l051xx.s:
1CC0BB095296FAA9A41AA322BE76F87557B7F7CDEB1484994EF805ECF94D0357.
Its BSD-3-Clause notice is preserved; Core/Startup/LICENSE.ST-BSD.txt includes
the license referenced by this source and the extracted ST clock setup.

Project metadata was compared with the installed official ide-project-creator
1.0.1. Only metadata was adopted; the generated sample's libraries, startup,
linker script and compiler choices were not substituted.

## PCG

Drivers/PCG contains unchanged PCG Minimal C source and LICENSE.txt at
[bc39cd76ac3d541e618606bcc6e1e5ba5e5e6aa3](https://github.com/imneme/pcg-c-basic/tree/bc39cd76ac3d541e618606bcc6e1e5ba5e5e6aa3).
The source identifies the minimal C implementation and retains its Apache-2.0
license. This fills non-security random fields only; it is not encryption,
authentication or a secure random generator.

| File | SHA256 |
| --- | --- |
| pcg_basic.c | B6582A071A8A090A293621523C063D125A532D772A2A1EB7D60B3E695FE47746 |
| pcg_basic.h | CD823DDC225DA9BE520A54F13EF2C491506C353800CAE04E8B673B2DE58F2CC4 |
| LICENSE.txt | B40930BBCF80744C86C46A12BC9DA056641D722716C378F5659B9E555EF833E1 |

Tools/fetch-vendors.ps1 documents immutable retrieval URLs. It is not a build
step. Normal builds use tracked source and do not require network access.
