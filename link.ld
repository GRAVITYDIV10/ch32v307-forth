ENTRY(_start)

MEMORY
{
	ROM (rx) : ORIGIN = 0x00000000, LENGTH = 192K
	RAM (rwx): ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
	.init :
	{
		. = ALIGN(4);
		*(.init)
		. = ALIGN(4);
	} >ROM AT>ROM

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text*)
		. = ALIGN(4);
	} >ROM AT>ROM

	.rodata :
	{
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} >ROM AT>ROM =0xFF

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>ROM

	.dlalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_lma = .);
	} >ROM AT>ROM

	.data :
	{
		. = ALIGN(4);
		*(.data)
		*(.data*)
		. = ALIGN(4);
		PROVIDE(_edata = .);
	} >RAM AT>ROM =0xFF

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		PROVIDE(_sbss = .);
		*(.sbss)
		*(.bss)
		PROVIDE(_ebss = .);
		. = ALIGN(4);
	} >RAM AT>ROM

	.stack ORIGIN(RAM) + LENGTH(RAM) :
	{
		PROVIDE(_eusrstack = .);
	} >RAM AT>ROM

	PROVIDE(SRAM_BASE = ORIGIN(RAM));
	PROVIDE(SRAM_SIZE = LENGTH(RAM));
	PROVIDE(SRAM_END  = SRAM_BASE + SRAM_SIZE);
}
