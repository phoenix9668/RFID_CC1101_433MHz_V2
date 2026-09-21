# RFID CC1101 V3.8 Firmware

STM32L051C8 collar firmware rebuilt from
`34d57c2a93b57979fecb929067e49df47c17c14d` using GNU Arm GCC, CMake and Ninja.
Develop with STM32CubeIDE for Visual Studio Code.

- [Development and debugging](Doc/Development/V38_REBUILD.md)
- [Source versions and licenses](Doc/Development/SOURCES.md)
- [Protocol and EEPROM format](Doc/Development/PROTOCOL_STORAGE.md)
- [V3.8 hardware and power ownership](Doc/Development/HARDWARE.md)
- [Verification results and open acceptance gates](Doc/Development/VERIFICATION.md)

The original six-class algorithm and 191-byte radio payload are compatibility
contracts. Software regression success is not physical acceptance: current
sensor sampling rate, base-station reception and power measurements must be
checked before deployment. Production raw-data radio and UART logs are disabled.
