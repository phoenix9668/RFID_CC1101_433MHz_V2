ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 8K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Stack_Size = 2048;
PHDRS { flash PT_LOAD FLAGS(5); ram PT_LOAD FLAGS(6); }
SECTIONS {
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) } > FLASH :flash
  .text : {
    . = ALIGN(4); *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4); _etext = .;
  } > FLASH
  .ARM.extab : { *(.ARM.extab*) } > FLASH
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT> FLASH :ram
  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; __bss_start__ = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; __bss_end__ = .; } > RAM
  .stack_reserve (NOLOAD) : { . = ALIGN(8); _end = .; end = .; . += _Min_Stack_Size; } > RAM
  /DISCARD/ : { *(.note.GNU-stack) }
}
ASSERT(_end <= _estack - _Min_Stack_Size, "RAM leaves less than 2 KiB for stack")
