ENTRY(_start)

MEMORY
{
	ROM (rx) : ORIGIN = 0x00000000, LENGTH = 192K 
	RAM (rwx): ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
	.text :
	{
		*(.init)
		*(.init*)
		. = ALIGN(4);
		*(.text)
		*(.text*)
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} >ROM AT>ROM

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		*(.bss)
		*(.sbss*)
		*(.COMMON*)
		. = ALIGN(4);
	} >RAM AT>ROM
}
