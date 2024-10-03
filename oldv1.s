#define fwp t0
#define fxp t1
#define fyp t2
#define fzp t3
#define fip s0
#define frp s1
#define fsp s2
#define fst s3
#define fss s4
#define fup tp

	.equ addrsize, 4
	.equ stksize, 64
	.equ tasksize, 32
	.equ tibsize, 128

	.equ tasknxt, (0 * addrsize)
	.equ taskfip, (1 * addrsize)
	.equ taskfrp, (2 * addrsize)
	.equ taskfsp, (3 * addrsize)
	.equ taskfst, (4 * addrsize)
	.equ taskfss, (5 * addrsize)

	.macro tasksave
		sw fip, taskfip(fup)
		sw frp, taskfrp(fup)
		sw fsp, taskfsp(fup)
		sw fst, taskfst(fup)
		sw fss, taskfss(fup)
	.endm

	.macro taskload
		lw fip, taskfip(fup)
		lw frp, taskfrp(fup)
		lw fsp, taskfsp(fup)
		lw fst, taskfst(fup)
		lw fss, taskfss(fup)
	.endm

        .macro rpush reg
                sw \reg, 0(frp)
                addi frp, frp, -addrsize
        .endm

        .macro rpop reg
                addi frp, frp, addrsize
                lw \reg, 0(frp)
        .endm

	.macro dpush reg
		sw \reg, 0(fsp)
		addi fsp, fsp, -addrsize
	.endm

	.macro dpop reg
		addi fsp, fsp, addrsize
		lw \reg, 0(fsp)
	.endm

	.macro next
		j a_next
	.endm

	.global _start
_start:
	.option norvc;
	j reset

reset:
	.equ RCC_BASE, 0x40021000
	.equ RCC_APB2PCENR, 0x18
	.equ RCC_IOPB_EN, (1 << 3)
	.equ RCC_IOPA_EN, (1 << 2)
	.equ RCC_UART1_EN, (1 << 14)

	# 使能时钟:
	# GPIOA GPIOB
	# UART1
	li t0, RCC_BASE
	lw t1, RCC_APB2PCENR(t0)
	li t2, RCC_IOPB_EN | RCC_IOPA_EN | RCC_UART1_EN
	or t1, t1, t2
	sw t1, RCC_APB2PCENR(t0)

	.equ GPIOB_BASE, 0x40010C00
	.equ GPIOA_BASE, 0x40010800
	.equ GPIO_CFGLR, 0x00
	.equ GPIO_CFGHR, 0x04
	.equ GPIO_OUTDR, 0x0C
	.equ GPIO_PB4CFG_MASK, (0xF << 16)
	.equ GPIO_PB4CFG_PPOUT_2Mhz, (0x2 << 16)
	.equ GPIO_PA15CFG_MASK, (0xF << 28)
	.equ GPIO_PA15CFG_PPOUT_2Mhz, (0x2 << 28)
	.equ GPIO_PA9CFG_MASK, (0xF << 4)
	.equ GPIO_PA9CFG_MUPPOUT_50Mhz, (0xB << 4)
	.equ GPIO_PA10CFG_MASK, (0xF << 8)
	.equ GPIO_PA10CFG_PUIN, (0x8 << 8)

	# 将PB4配置为推挽输出模式
	li t0, GPIOB_BASE
	lw t1, GPIO_CFGLR(t0)
	li t2, GPIO_PB4CFG_MASK
	xori t2, t2, -1
	and t1, t1, t2
	li t2, GPIO_PB4CFG_PPOUT_2Mhz
	or t1, t1, t2
	sw t2, GPIO_CFGLR(t0)

	# 将PA15配置为推挽输出模式
	# 将PA9 配置为复用推挽输出模式
	# 将PA10配置为上下拉输入模式
	li t0, GPIOA_BASE
	lw t1, GPIO_CFGHR(t0)
	li t2, GPIO_PA15CFG_MASK | GPIO_PA9CFG_MASK | GPIO_PA10CFG_MASK
	xori t2, t2, -1
	and t1, t1, t2
	li t2, GPIO_PA15CFG_PPOUT_2Mhz | GPIO_PA9CFG_MUPPOUT_50Mhz | GPIO_PA10CFG_PUIN
	or t1, t1, t2
	sw t2, GPIO_CFGHR(t0)

	# PB4输出高电平
	li t0, GPIOB_BASE
	lw t1, GPIO_OUTDR(t0)
	ori t1, t1, (1 << 4)
	sw t1, GPIO_OUTDR(t0)

	# PA15输出高电平
	# PA10启用上拉
	li t0, GPIOA_BASE
	lw t1, GPIO_OUTDR(t0)
	li t2, (1 << 15) | (1 << 10)
	or t1, t1, t2
	sw t1, GPIO_OUTDR(t0)

	.equ UART1_BASE, 0x40013800
	.equ UART_DATAR, 0x04
	.equ UART_BRR,   0x08

	# 从寄存器手册内容和时钟树可以得到:
	# UART1的时钟源为PCLK2,即APB2提供的
	# PCLK2的时钟源是HCLK
	# HCLK的时钟源是SYSCLK

	# SYSCLK默认使用HSI，SYSCLK=8Mhz
	# SYSCLK根据HPRE分频得到HCLK，默认没有分频，HCLK=8Mhz
	# HCLK根据PPRE2分频得到PCLK2，默认没有分频，PCLK2=8Mhz

	# 波特率计算公式:
	# UART1这里的FCLK是PCLK2
	# 波特率 = FCLK / (16 * UARTDIV)
	# 那么:
	# (8 * 1000 * 1000) / (16 * 4.34) = 115207
	# UARTDIV=4.34
	# UARTDIV的计算公式:
	# UARTDIV=DIV_M+(DIV_F/16)
	# 那么:
	# 4 + (5 / 16) = 4.3125
	# DIV_M=4
	# DIV_F=5

	.equ BAUD_115200, ((4 << 4) | (5 << 0))

	li t0, UART1_BASE

	# 配置UART1波特率为115200
	li t1, BAUD_115200
	sw t1, UART_BRR(t0)

	# 使能UART控制器启用TX启用RX
	.equ UART_CTLR1, 0x0C
	.equ UART_UE, (1 << 13)
	.equ UART_TE, (1 << 3)
	.equ UART_RE, (1 << 2)
	li t1, UART_UE | UART_TE | UART_RE
	sw t1, UART_CTLR1(t0)

forth:
	la frp, rstk_human_top
	li fwp, 0x01234567
	rpush fwp
	rpop fxp
	bne fwp, fxp, a_AaAa
	la fwp, rstk_human_top
	bne frp, fwp, a_AaAa

	la fsp, dstk_human_top
	li fwp, 0x89ABCDEF
	dpush fwp
	dpop fxp
	bne fwp, fxp, a_AaAa
	la fwp, dstk_human_top
	bne fsp, fwp, a_AaAa

	la fst, dstk_human_top

	la fip, boot_human

	la fup, task_human
	sw fup, tasknxt(fup)

	tasksave
	taskload

	next

rom_dict_base:
	.equ offset_shift, 4
	.equ nlen_shift, 12
	.equ offset_mask, (0xFF << offset_shift)
	.equ nlen_mask, (0xFF << nlen_shift)
	.macro wdef label, name, entry, link, attr
		.p2align 2, 0
		.word \link
	f_\label:
		.word \entry
		.word \attr + (offset_\label << offset_shift) \
				+ (nlen_\label << nlen_shift)
	n_\label:
		.ascii "\name"
	n_end_\label:
		.p2align 2, 0
		.set nlen_\label, n_end_\label - n_\label
	p_\label:
		.set offset_\label, p_\label - f_\label
	.endm

	.macro wdefcode label, name, link, attr
		wdef \label, \name, a_\label, \link, \attr
	a_\label:
	.endm

	wdefcode UuUu, "UuUu", 0, 0
	# UART1 PA9 TX 不断发送 0x55 0x75
	.equ UART_STATR, 0x00
	.equ UART_TC, (1 << 6)
	li t0, UART1_BASE
	li t2, 0x55

2:
1:
	lw t1, UART_STATR(t0)
	andi t1, t1, UART_TC
	beqz t1, 1b
	sw t2, UART_DATAR(t0)
	xori t2, t2, (1 << 5)
	j 2b

        wdefcode AaAa, "AaAa", f_UuUu, 0
        # UART1 PA9 TX 不断发送 0x41 0x61
        .equ UART_STATR, 0x00
        .equ UART_TC, (1 << 6)
        li t0, UART1_BASE
        li t2, 0x41

2:
1:
        lw t1, UART_STATR(t0)
        andi t1, t1, UART_TC
        beqz t1, 1b
        sw t2, UART_DATAR(t0)
        xori t2, t2, (1 << 5)
        j 2b

	wdefcode next, "next", f_AaAa, 0
	lw fwp, 0(fip)
	addi fip, fip, addrsize
	lw fxp, 0(fwp)
	jr fxp

	wdefcode noop, "noop", f_next, 0
	next

	wdefcode call, "call", f_noop, 0
	rpush fip
	lw fxp, addrsize(fwp)
	li fyp, offset_mask
	and fip, fxp, fyp
	srli fip, fip, offset_shift
	add fip, fwp, fip
	next

	wdefcode exit, "exit", f_call, 0
	rpop fip
	next

	.macro wdefword label, name, link, attr
		wdef \label, \name, a_call, \link, \attr
	w_\label:
	.endm

	wdefword fnop, "fnop", f_exit, 0
	.word f_exit

	wdefcode branch, "branch", f_fnop, 0
	lw fip, 0(fip)
	next

	wdefcode lit, "lit", f_branch, 0
	lw fxp, 0(fip)
	addi fip, fip, addrsize
	dpush fxp
	next

	wdefcode branch0, "branch0", f_lit, 0
	dpop fwp
	lw fxp, 0(fip)
	addi fip, fip, addrsize
	bnez fwp, 1f
	mv fip, fxp
1:
	next

	wdef equ, "=", a_equ, f_branch0, 0
a_equ:
	dpop fwp
	dpop fxp
	beq fwp, fxp, 1f
	dpush zero
	next
1:
	li fxp, -1
	dpush fxp
	next

	wdef load, "@", a_load, f_equ, 0
a_load:
	dpop fwp
	lw fxp, 0(fwp)
	dpush fxp
	next

	wdef store, "!", a_store, f_load, 0
a_store:
	dpop fwp
	dpop fxp
	sw fxp, 0(fwp)
	next

	wdefcode doconst, "doconst", f_store, 0
	lw fxp, addrsize(fwp)
	li fyp, offset_mask
	and fxp, fxp, fyp
	srli fxp, fxp, offset_shift
	add fxp, fwp, fxp
	lw fxp, 0(fxp)
	dpush fxp
	next

        .macro wdefconst label, name, link, attr
                wdef \label, \name, a_doconst, \link, \attr
        v_\label:
        .endm

	wdefconst cell, "cell", f_doconst, 0
	.word addrsize

	wdefcode yield, "yield", f_cell, 0
	tasksave
	lw fup, tasknxt(fup)
	taskload
	next 

	wdefcode 2lit, "2lit", f_yield, 0
	lw fxp, 0(fip)
	addi fip, fip, addrsize
	dpush fxp
	lw fxp, 0(fip)
	addi fip, fip, addrsize
	dpush fxp
	next

	wdef add, "+", a_add, f_yield, 0
a_add:
	dpop fwp
	dpop fxp
	add fwp, fwp, fxp
	dpush fwp
	next

	wdef sub, "-", a_sub, f_add, 0
a_sub:
	dpop fwp
	dpop fxp
	sub fxp, fxp, fwp
	dpush fxp
	next

	wdef mul, "*", a_mul, f_sub, 0
a_mul:
	dpop fwp
	dpop fxp
	mul fxp, fxp, fwp
	dpush fxp
	next

	wdef div, "/", a_div, f_mul, 0
a_div:
	dpop fwp
	dpop fxp
	div fxp, fxp, fwp
	dpush fxp
	next

	wdefcode lshift, "lshift", f_div, 0
	dpop fwp
	dpop fxp
	sll fxp, fxp, fwp
	dpush fxp
	next

	wdefcode rshift, "rshift", f_lshift, 0
	dpop fwp
	dpop fxp
	srl fxp, fxp, fwp
	dpush fxp
	next

	wdefcode spget, "sp@", f_rshift, 0
	dpush fsp
	next

	wdefcode stget, "st@", f_spget, 0
	dpush fst
	next

	wdefcode 2div, "2/", f_stget, 0
	dpop fwp
	srli fwp, fwp, 1
	dpush fwp
	next

	wdefcode 2mul, "2*", f_2div, 0
	dpop fwp
	slli fwp, fwp, 1
	dpush fwp
	next

	wdefcode celldiv, "cell/", f_2mul, 0
	dpop fwp
	srli fwp, fwp, 2
	dpush fwp
	next

	wdefcode cellmul, "cell*", f_celldiv, 0
	dpop fwp
	slli fwp, fwp, 2
	dpush fwp
	next

	wdefconst true, "true", f_cellmul, 0
	.word -1

	wdefconst false, "false", f_true, 0
	.word 0

	wdefcode swap, "swap", f_false, 0
	dpop fwp
	dpop fxp
	dpush fwp
	dpush fxp
	next

	wdefword depth, "depth", f_swap, 0
	.word f_spget
	.word f_stget
	.word f_swap
	.word f_sub
	.word f_celldiv
	.word f_exit

	wdefword dzchk, "dzchk", f_depth, 0
	.word f_depth
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_exit

	wdefcode cload, "c@", f_dzchk, 0
	dpop fwp
	lbu fxp, 0(fwp)
	dpush fxp
	next

	wdefcode dec, "1-", f_cload, 0
	dpop fwp
	addi fwp, fwp, -1
	dpush fwp
	next

	wdefcode inc, "1+", f_dec, 0
	dpop fwp
	addi fwp, fwp, 1
	dpush fwp
	next

	wdefcode neq, "<>", f_inc, 0
	dpop fwp
	dpop fxp
	bne fwp, fxp, 1f
	dpush zero
	next
1:
	li fxp, -1
	dpush fxp
	next

	wdefcode and, "and", f_neq, 0
	dpop fwp
	dpop fxp
	and fwp, fwp, fxp
	dpush fwp
	next

	wdefcode nez, "0<>", f_and, 0
	dpop fwp
	bnez fwp, 1f
	dpush zero
	next
1:
	li fwp, -1
	dpush fwp
	next

	wdefcode drop, "drop", f_nez, 0
	dpop fwp
	next

	wdefcode dup, "dup", f_drop, 0
	dpop fwp
	dpush fwp
	dpush fwp
	next

	wdefconst UART1, "UART1", f_dup, 0
	.word UART1_BASE

	wdefconst UART_DATAR, "UART_DATAR", f_UART1, 0
	.word UART_DATAR

	wdefconst UART_STATR, "UART_STATR", f_UART_DATAR, 0
	.word UART_STATR

	wdefconst UART_TC, "UART_TC", f_UART_STATR, 0
	.word UART_TC

	wdefword uarttxava, "uarttx?", f_UART_TC, 0
	.word f_UART_STATR
	.word f_add
	.word f_load
	.word f_UART_TC
	.word f_and
	.word f_nez
	.word f_exit

	wdefword uarttxwait, "uarttxwait", f_uarttxava, 0
1:
	.word f_yield
	.word f_dup
	.word f_uarttxava
	.word f_branch0
	.word 1b
	.word f_drop
	.word f_exit

	wdefword uarttxfill, "uarttxfill", f_uarttxwait, 0
	.word f_UART_DATAR
	.word f_add
	.word f_store
	.word f_exit

	wdefword uarttx, "uarttx", f_uarttxfill, 0
	.word f_dup
	.word f_uarttxwait
	.word f_uarttxfill
	.word f_exit

	wdefword uart1tx, "uart1tx", f_uarttx, 0
	.word f_UART1
	.word f_uarttx
	.word f_exit

	wdefword emit, "emit", f_uart1tx, 0
	.word f_uart1tx
	.word f_exit

	wdefcode or, "or", f_emit, 0
	dpop fwp
	dpop fxp
	or fwp, fwp, fxp
	dpush fwp
	next

	wdefcode 2drop, "2drop", f_or, 0
	dpop fwp
	dpop fwp
	next

	wdefword type, "type", f_2drop, 0
2:
	.word f_dup
	.word f_branch0
	.word 1f
	.word f_dec
	.word f_swap
	.word f_dup
	.word f_cload
	.word f_emit
	.word f_inc
	.word f_swap
	.word f_branch
	.word 2b
1:
	.word f_2drop
	.word f_exit

	.equ UART_RXNE, (1 << 5)
	wdefconst UART_RXNE, "UART_RXNE", f_type, 0
	.word UART_RXNE

	wdefword uartrxava, "uartrx?", f_UART_RXNE, 0
	.word f_UART_STATR
	.word f_add
	.word f_load
	.word f_UART_RXNE
	.word f_and
	.word f_nez
	.word f_exit

	wdefword uartrxwait, "uartrxwait", f_uartrxava, 0
1:
	.word f_yield
	.word f_dup
	.word f_uartrxava
	.word f_branch0
	.word 1b
	.word f_drop
	.word f_exit

	wdefword uartrxread, "uartrxread", f_uartrxwait, 0
	.word f_UART_DATAR
	.word f_add
	.word f_load
	.word f_exit


	wdefword uartrx, "uartrx", f_uartrxread, 0
	.word f_dup
	.word f_uartrxwait
	.word f_uartrxread
	.word f_exit

	wdefword uart1rx, "uart1rx", f_uartrx, 0
	.word f_UART1
	.word f_uartrx
	.word f_exit

	wdefword key, "key", f_uart1rx, 0
	.word f_uart1rx
	.word f_exit

	wdefword isspace, "isspace", f_key, 0
	.word f_dup
	.word f_lit
	.word ' '
	.word f_equ
	.word f_swap
	.word f_lit
	.word '\t'
	.word f_equ
	.word f_or
	.word f_exit

	wdefword isnewline, "isnewline", f_isspace, 0
	.word f_dup
	.word f_lit
	.word '\n'
	.word f_equ
	.word f_swap
	.word f_lit
	.word '\r'
	.word f_equ
	.word f_or
	.word f_exit

	wdefword isdelete, "isdelete", f_isnewline, 0
	.word f_dup
	.word f_lit
	.word '\b'
	.word f_equ
	.word f_swap
	.word f_lit
	.word 0x7F
	.word f_equ
	.word f_or
	.word f_exit

	wdefconst toin, ">in", f_isdelete, 0
	.word toin

	wdefconst tib, "tib", f_toin, 0
	.word tib

	wdefword toinget, ">in@", f_tib, 0
	.word f_toin
	.word f_load
	.word f_exit

	wdefword toinset, ">in!", f_toinget, 0
	.word f_toin
	.word f_store
	.word f_exit

	wdefword toinrst, ">inrst", f_toinset, 0
	.word f_lit
	.word 0
	.word f_toinset
	.word f_exit

	wdefconst tibsize, "tibsize", f_toinrst, 0
	.word tibsize

	wdefcode rot, "rot", f_tibsize, 0
	dpop fwp
	dpop fxp
	dpop fyp
	dpush fxp
	dpush fwp
	dpush fyp
	next

	wdefcode over, "over", f_rot, 0
	dpop fwp
	dpop fxp
	dpush fxp
	dpush fwp
	dpush fxp
	next

	wdefcode lt, "<", f_over, 0
	dpop fwp
	dpop fxp
	blt fxp, fwp, 1f
	dpush zero
	next
1:
	li fxp, -1
	dpush fxp
	next

	wdefcode gt, ">", f_lt, 0
	dpop fwp
	dpop fxp
	bgt fxp, fwp, 1f
	dpush zero
	next
1:
	li fxp, -1
	dpush fxp
	next

	wdefcode xor, "xor", f_gt, 0
	dpop fwp
	dpop fxp
	xor fwp, fwp, fxp
	dpush fwp
	next

	wdefcode invert, "invert", f_xor, 0
	dpop fwp
	xori fwp, fwp, -1
	dpush fwp
	next

	wdef le, "<=", a_le, f_invert, 0
a_le:
	dpop fwp
	dpop fxp
	ble fxp, fwp, 1f
	dpush zero
	next
1:
	li fxp, -1
	dpush fxp
	next

	wdef ge, ">=", a_ge, f_le, 0
a_ge:
	dpop fwp
	dpop fxp
	bge fxp, fwp, 1f
	dpush zero
	next
1:
	li fxp, -1
	dpush fxp
	next

	wdefword within, "within", f_ge, 0


	.p2align, 2, 0
boot_human:
	.macro display label, string
		.word f_2lit
		.word _str_\label
		.word _str_end_\label - _str_\label
		.word f_type
		.word f_branch
		.word _skip_\label
	_str_\label:
		.ascii "\string"
	_str_end_\label:
		.p2align 2,0 
	_skip_\label:
	.endm

	.word f_lit
	.word 1
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_lit
	.word 0
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_branch
	.word 1f
	.word f_AaAa
1:
	display test_cell,"cell."
	.word f_dzchk
        .word f_cell
        .word f_lit
        .word addrsize
        .word f_equ
        .word f_branch0
        .word fail
	display test_yield,"yield."
	.word f_yield
	display test_2lit,"2lit."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	display test_add,"+."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_add
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_2lit
	.word 0
	.word 1
	.word f_add
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 0
	.word f_add
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 1
	.word f_add
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word -1
	.word f_add
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word -1
	.word -1
	.word f_add
	.word f_lit
	.word -2
	.word f_equ
	.word f_branch0
	.word fail

	display test_sub,"-."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_sub
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 0
	.word f_sub
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 0
	.word 1
	.word f_sub
	.word f_lit
	.word -1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 1
	.word f_sub
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word -1
	.word -1
	.word f_sub
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_mul,"*."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_mul
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 0
	.word f_mul
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 0
	.word 1
	.word f_mul
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 1
	.word f_mul
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 2
	.word 2
	.word f_mul
	.word f_lit
	.word 4
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 3
	.word 5
	.word f_mul
	.word f_lit
	.word 15
	.word f_equ
	.word f_branch0
	.word fail

	display test_div,"/."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_div
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 2
	.word f_div
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 1
	.word f_div
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 2
	.word 1
	.word f_div
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	display test_lshift,"lshift."
	.word f_dzchk
	.word f_2lit
	.word 1
	.word 0
	.word f_lshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 1
	.word f_lshift
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 2
	.word f_lshift
	.word f_lit
	.word 4
	.word f_equ
	.word f_branch0
	.word fail

	display test_rshift,"rshift."
	.word f_dzchk
	.word f_2lit
	.word 4
	.word 0
	.word f_rshift
	.word f_lit
	.word 4
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 4
	.word 1
	.word f_rshift
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 4
	.word 2
	.word f_rshift
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	display test_2div,"2/."
	.word f_dzchk
	.word f_lit
	.word 2
	.word f_2div
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	display test_2mul,"2*."
	.word f_dzchk
	.word f_lit
	.word 1
	.word f_2mul
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	display test_celldiv,"cell/."
	.word f_dzchk
	.word f_cell
	.word f_celldiv
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	display test_cellmul,"cell*."
	.word f_dzchk
	.word f_lit
	.word 1
	.word f_cellmul
	.word f_cell
	.word f_equ
	.word f_branch0
	.word fail

	display test_true_false,"true.false."
	.word f_dzchk
	.word f_true
	.word f_branch0
	.word fail
	.word f_false
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_swap,"swap."
	.word f_dzchk
	.word f_2lit
	.word 1
	.word 0
	.word f_swap
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_depth,"depth."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_depth
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_inc,"1+."
	.word f_dzchk
	.word f_lit
	.word 0
	.word f_inc
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word 1
	.word f_inc
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word -1 
	.word f_inc
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_dec,"1-."
	.word f_dzchk
	.word f_lit
	.word 0
	.word f_dec
	.word f_lit
	.word -1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word 2
	.word f_dec
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word 1
	.word f_dec
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_neq,"<>."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_neq
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 1
	.word f_neq
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_and,"and."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_and
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_and
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_dzchk
	.word f_2lit
	.word 1
	.word 0
	.word f_and
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_dzchk
	.word f_2lit
	.word 1
	.word 1
	.word f_and
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	display test_nez,"0<>."
	.word f_dzchk
	.word f_lit
	.word 0
	.word f_nez
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_lit
	.word 1
	.word f_nez
	.word f_branch0
	.word fail

	display test_drop,"drop."
	.word f_dzchk
	.word f_lit
	.word 1
	.word f_drop
	.word f_dzchk

	display test_dup,"dup."
	.word f_dzchk
	.word f_lit
	.word 0
	.word f_dup
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_or,"or."
	.word f_dzchk
	.word f_2lit
	.word 1
	.word 0
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 0
	.word 1
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 1
	.word f_or
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 0
	.word 0
	.word f_or
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_2drop,"2drop."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_2drop

	display test_isspace,"isspace."
	.word f_dzchk
	.word f_lit
	.word ' '
	.word f_isspace
	.word f_branch0
	.word fail
	.word f_lit
	.word '\t'
	.word f_isspace
	.word f_branch0
	.word fail
	.word f_lit
	.word 'a'
	.word f_isspace
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_isnewline,"isnewline."
	.word f_dzchk
	.word f_lit
	.word '\n'
	.word f_isnewline
	.word f_branch0
	.word fail
	.word f_lit
	.word '\r'
	.word f_isnewline
	.word f_branch0
	.word fail
	.word f_lit
	.word 'a'
	.word f_isnewline
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_isdelete,"isdelete."
	.word f_dzchk
	.word f_lit
	.word '\b'
	.word f_isdelete
	.word f_branch0
	.word fail
	.word f_lit
	.word 0x7F
	.word f_isdelete
	.word f_branch0
	.word fail
	.word f_lit
	.word 'a'
	.word f_isdelete
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_toin,">in.tib.>in@.>in!.>inrst."
	.word f_dzchk
	.word f_toinrst
	.word f_toinget
	.word f_inc
	.word f_toinset
	.word f_toinget
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_toinrst
	.word f_toinget
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	display test_rot, "rot."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_lit
	.word 2
	.word f_rot
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_lit
	.word 2
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail


	display test_over, "over."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 1
	.word f_over
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_lt, "<."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_lt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 0
	.word f_lt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 0
	.word -1
	.word f_lt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word -1
	.word -2
	.word f_lt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 0
	.word 1
	.word f_lt
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 2
	.word f_lt
	.word f_branch0
	.word fail

	.word f_2lit
	.word -1
	.word 0
	.word f_lt
	.word f_branch0
	.word fail

	.word f_2lit
	.word -2
	.word -1
	.word f_lt
	.word f_branch0
	.word fail


	display test_gt, ">."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_2lit
	.word -1
	.word 0
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 0
	.word 1
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word -2
	.word -1
	.word f_gt
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 0
	.word f_gt
	.word f_branch0
	.word fail

	.word f_2lit
	.word 0
	.word -1
	.word f_gt
	.word f_branch0
	.word fail

	.word f_2lit
	.word -1
	.word -2
	.word f_gt
	.word f_branch0
	.word fail

	display test_xor,"xor."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_xor
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 1
	.word f_xor
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 1
	.word 0
	.word f_xor
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_2lit
	.word 0
	.word 1
	.word f_xor
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	display test_invert,"invert."
	.word f_dzchk
	.word f_lit
	.word 0
	.word f_invert
	.word f_lit
	.word -1
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word -1
	.word f_invert
	.word f_lit
	.word 0
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word 1
	.word f_invert
	.word f_lit
	.word -2
	.word f_equ
	.word f_branch0
	.word fail

	.word f_lit
	.word -2
	.word f_invert
	.word f_lit
	.word 1
	.word f_equ
	.word f_branch0
	.word fail

	display test_le,"<=."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_le
	.word f_branch0
	.word fail

	.word f_2lit
	.word -1
	.word 0
	.word f_le
	.word f_branch0
	.word fail

	.word f_2lit
	.word -2
	.word -1
	.word f_le
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 0
	.word f_le
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	.word f_2lit
	.word 2
	.word 1
	.word f_le
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_ge,">=."
	.word f_dzchk
	.word f_2lit
	.word 0
	.word 0
	.word f_ge
	.word f_branch0
	.word fail

	.word f_2lit
	.word 1
	.word 0
	.word f_ge
	.word f_branch0
	.word fail

	.word f_2lit
	.word 0
	.word -1
	.word f_ge
	.word f_branch0
	.word fail

	.word f_2lit
	.word -1
	.word -2
	.word f_ge
	.word f_branch0
	.word fail

	.word f_2lit
	.word 0
	.word 1
	.word f_ge
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_2lit
	.word -1
	.word 0
	.word f_ge
	.word f_branch0
	.word 1f
	.word f_AaAa
1:
	.word f_2lit
	.word -2
	.word -1
	.word f_ge
	.word f_branch0
	.word 1f
	.word f_AaAa
1:

	display test_echo,"echo."
1:
	.word f_key
	.word f_emit
	.word f_dzchk
	.word f_branch
	.word 1b

	display test_end,"end."
1:
	.word f_dzchk
	.word f_fnop
	.word f_noop
	.word f_branch
	.word 1b
	.word f_UuUu
fail:
	.word f_AaAa

	.section .bss
	.p2align 2, 0
tib:
	.fill tibsize, 1, 0
toin:
	.fill 1, addrsize, 0
task_human:
	.fill tasksize, addrsize, 0
dstk_human:
	.fill stksize, addrsize, 0
dstk_human_top:
	.fill 1, addrsize, 0
rstk_human:
	.fill stksize, addrsize, 0
rstk_human_top:
	.fill 1, addrsize, 0
