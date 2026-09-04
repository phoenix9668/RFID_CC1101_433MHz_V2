# TI CC1101 Reference Sources

This project uses the TI CC1101 examples as a reference, not as vendored
buildable source.

Official references:

- `SWRC021` - CC1100, CC1101, CC1100E and CC2500 examples/libraries.
- Version inspected: `01.00.00.0F`, released 2009-03-31.
- Download page: https://www.ti.com/tool/download/SWRC021
- SmartRF Studio: https://www.ti.com/tool/SMARTRFTM-STUDIO

Reference files inspected from SWRC021:

- `SpiReadReg.c`
- `SpiReadStatus.c`
- `SpiSWriteReg.c`
- `SpiWriteBurstReg.c`
- `SpiStrobe.c`
- `RfSendPacket.c`
- `RfReceivePacket.c`
- `RfWriteRfSettings.c`

The SWRC021 implementation targets the Chipcon SRF04 / Silabs F320 C51
platform, so the firmware keeps STM32L0-specific code in `Core/Src` and only
ports the driver behavior that matters for CC1101 transactions:

- assert CSn before each SPI transaction;
- wait for SO/MISO to go low before sending the SPI header byte;
- use burst access for FIFO traffic;
- use appended status bytes for RX RSSI/LQI/CRC status;
- use SmartRF Studio register settings as the RF configuration source.
