MEMORY
{
  FLASH_STAGE0 (rx) : ORIGIN = 0x60000000, LENGTH = 512K
  ITCM         (rx) : ORIGIN = 0x00000000, LENGTH = 512K
  DTCM         (rwx): ORIGIN = 0x20000000, LENGTH = 512K
}

ENTRY(stage0_main)

SECTIONS
{
  .vectors :
  {
    KEEP(*(.isr_vector))
  } > ITCM AT> FLASH_STAGE0

  .text :
  {
    *(.itcm_text*)
    *(.text*)
    *(.rodata*)
  } > ITCM AT> FLASH_STAGE0

  .data :
  {
    *(.dtcm_data*)
    *(.data*)
  } > DTCM AT> FLASH_STAGE0

  .bss (NOLOAD) :
  {
    *(.dtcm_bss*)
    *(.bss*)
    *(COMMON)
  } > DTCM

  .ARM.exidx :
  {
    *(.ARM.exidx*)
  } > ITCM AT> FLASH_STAGE0
}

_itcm_load = LOADADDR(.text);
_itcm_start = ADDR(.text);
_itcm_end = _itcm_start + SIZEOF(.text);

_dtcm_load = LOADADDR(.data);
_dtcm_start = ADDR(.data);
_dtcm_end = _dtcm_start + SIZEOF(.data);
