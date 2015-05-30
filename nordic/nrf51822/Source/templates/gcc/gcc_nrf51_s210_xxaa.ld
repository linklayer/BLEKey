/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x0000C000, LENGTH = 0x34000 
  RAM (rwx) :  ORIGIN = 0x20000900, LENGTH = 0x3700 
}


INCLUDE "gcc_nrf51_common.ld"