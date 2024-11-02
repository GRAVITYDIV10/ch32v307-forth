#define wp a0
#define xp a1
#define yp a2	
#define zp a3
#define psp a4
#define psb a5
#define ip s0
#define rsp s1
#define up tp

	.equ STKSIZE, 31
	.equ USRSIZE, 63
	.equ TIBSIZE, 31

	.section .init
	.global _start
_start:
	.option norvc;
	j reset
	#.option rvc;

	.section .text
reset:
        la sp, _eusrstack
	/* Load data section from flash to RAM */
        la a0, _data_lma
        la a1, _data_vma
        la a2, _edata
        bgeu a1, a2, 2f
1:
        lw t0, (a0)
        sw t0, (a1)
        addi a0, a0, 4
        addi a1, a1, 4
        bltu a1, a2, 1b
2:
	/* Clear bss section */
        la a0, _sbss
        la a1, _ebss
        bgeu a0, a1, 2f
1:
        sw zero, (a0)
        addi a0, a0, 4
        bltu a0, a1, 1b
2:
	/* Configure pipelining and instruction prediction */
	li t0, 0x1f
	csrw 0xbc0, t0

	/* Enable floating point and global interrupt */
	/* configure privileged mode */
        li t0, 0x6088
        csrw mstatus, t0

	/* setup irq entry */
	la t0, irq_entry
	ori t0, t0, 0x3
	csrw mtvec, t0

rcc_init:
	call _clk_init


gpiob_init:
	.equ GPIOB_BASE, 0x40010C00
	.equ GPIO_CFGLR, 0x00
	.equ GPIO_OUTDR, 0x0C
	.equ GPIO_BSHR,  0x10

	.equ GPIO_CFG_PPOUT_2M, 0x2

	.equ PB4_CFG_SHIF, (4 * 4)
	.equ PB4_CFG_MASK, 0xF << PB4_CFG_SHIF

	.equ PB4_CFG, (GPIO_CFG_PPOUT_2M << PB4_CFG_SHIF)

	li wp, GPIOB_BASE
	lw xp, GPIO_CFGLR(wp)
	li yp, PB4_CFG_MASK
	xori yp, yp, -1
	and xp, xp, yp
	li yp, PB4_CFG
	or xp, xp, yp
	sw xp, GPIO_CFGLR(wp)

	li xp, (1 << 4)
	sw xp, GPIO_BSHR(wp)


gpioa_init:
	.equ GPIOA_BASE, 0x40010800
	.equ GPIO_CFGHR, 0x04

	.equ GPIO_CFG_MUPPOUT_50M, 0xB
	.equ GPIO_CFG_PUIN, 0x8

	.equ PA8_CFG_SHIF, ((8 - 8) * 4)
	.equ PA8_CFG_MASK, 0xF << PA8_CFG_SHIF
	.equ PA8_CFG, (GPIO_CFG_MUPPOUT_50M << PA8_CFG_SHIF)

	.equ PA9_CFG_SHIF, ((9 - 8) * 4)
	.equ PA9_CFG_MASK, 0xF << PA9_CFG_SHIF
	.equ PA9_CFG, (GPIO_CFG_MUPPOUT_50M << PA9_CFG_SHIF)

	.equ PA10_CFG_SHIF, ((10 - 8) * 4)
	.equ PA10_CFG_MASK, 0xF << PA10_CFG_SHIF
	.equ PA10_CFG, (GPIO_CFG_PUIN << PA10_CFG_SHIF)
	
	.equ PA15_CFG_SHIF, ((15 - 8) * 4)
	.equ PA15_CFG_MASK, 0xF << PA15_CFG_SHIF
	.equ PA15_CFG, (GPIO_CFG_PPOUT_2M << PA15_CFG_SHIF)

	li wp, GPIOA_BASE
	lw xp, GPIO_CFGHR(wp)
	li yp, PA8_CFG_MASK | PA9_CFG_MASK | PA10_CFG_MASK | PA15_CFG_MASK
	xori yp, yp, -1
	and xp, xp, yp
	li yp, PA8_CFG | PA9_CFG | PA10_CFG | PA15_CFG
	or xp, xp, yp
	sw xp, GPIO_CFGHR(wp)

	li xp, (1 << 15) | (1 << 10)
	sw xp, GPIO_BSHR(wp)

uart1_init:
	.equ UART1_BASE, 0x40013800
	.equ UART_STATR, 0x00
	.equ UART_DATAR, 0x04
	.equ UART_BRR, 0x08
	.equ UART_CTLR1, 0x0C

	.equ UART_TC, (1 << 6)
	.equ UART_RXNE, (1 << 5)

	.equ UART_UE, (1 << 13)
	.equ UART_TE, (1 << 3)
	.equ UART_RE, (1 << 2)

	/* # CLK_SRC = 8Mhz
	.equ UART_BAUD_115200, ((4 << 4) | (5 << 0))
	.equ UART_BAUD_9600, ((52 << 4) | (0 << 0))
	*/

	# CLK_SRC = 96Mhz
	.equ UART_BAUD_115200, ((52 << 4) | (3 << 0))

	li wp, UART1_BASE
	li xp, UART_BAUD_115200
	sw xp, UART_BRR(wp)

	li xp, UART_UE | UART_TE | UART_RE
	sw xp, UART_CTLR1(wp)

	.section .rodata
str_usart:
	.asciz "USART."

	.section .text

	la wp, str_usart
	call early_puts

enter_forth:
	la wp, forth
	csrw mepc, wp
	mret

early_txc:
	li zp, UART1_BASE
1:
	lw xp, UART_STATR(zp)
	andi xp, xp, UART_TC
	beqz xp, 1b
	sw wp, UART_DATAR(zp)
	ret

early_puts:
2:
	li zp, UART1_BASE
1:
	lw xp, UART_STATR(zp)
	andi xp, xp, UART_TC
	beqz xp, 1b
	lbu xp, 0(wp)
	addi wp, wp, 1
	beqz xp, 1f
	sw xp, UART_DATAR(zp)
	j 2b
1:
	ret

	.set lastword, 0

	.macro defword label, name, entry, attr
		.p2align 2, 0xFF
	name_\label:
		.ascii "\name"
	name_end_\label:
		.p2align 2, 0xFF
		.set nlen_\label, name_end_\label - name_\label
		.set prev_\label, lastword
	attr_\label:
		.word nlen_\label + \attr
	link_\label:
		.word prev_\label	
	w_\label:
	entr_\label:
		.word \entry
		.set lastword, entr_\label
	body_\label:
	.endm

	.section .text

	defword panic, "panic", e_panic, 0
e_panic:
	li a0, '!'
	call early_txc
1:
	j 1b

	defword halt, "halt", e_halt, 0
e_halt:
	li a0, '@'
	call early_txc
1:
	j 1b

	defword next, "next", e_next, 0
e_next:
	lw wp, 0(ip)
	addi ip, ip, 4
	lw xp, 0(wp)
	jr xp

	.macro next
		j e_next
	.endm

	defword branch, "branch", e_branch, 0
e_branch:
	lw ip, 0(ip)
	next

	.macro rpush reg
		sw \reg, 0(rsp)
		addi rsp, rsp, 4
	.endm

	.macro rpop reg
		addi rsp, rsp, -4
		lw \reg, 0(rsp)
	.endm

	defword call, "call", e_call, 0
e_call:
	rpush ip
	addi ip, wp, 4
	next

	defword exit, "exit", e_exit, 0
e_exit:
	rpop ip
	next

	defword noop, "noop", e_call, 0
	.word w_exit

	.macro dpush reg
		sw \reg, 0(psp)
		addi psp, psp, 4
	.endm

	.macro dpop reg
		addi psp, psp, -4
		lw \reg, 0(psp)
	.endm

	defword lit, "lit", e_lit, 0
e_lit:
	lw wp, 0(ip)
	addi ip, ip, 4
	dpush wp
	next

	defword 0branch, "0branch", e_0branch, 0
e_0branch:
	dpop wp
	lw xp, 0(ip)
	addi ip, ip, 4
	bnez wp, 1f
	mv ip, xp
1:
	next

	defword drop, "drop", e_drop, 0
e_drop:
	dpop wp
	next

	defword dzchk, "dzchk", e_dzchk, 0
e_dzchk:
	bne psb, psp, e_panic
	next

	defword nepanic, "<>panic", e_nepanic, 0
e_nepanic:
	dpop wp
	dpop xp
	bne wp, xp, e_panic
	next

	defword 2lit, "2lit", e_2lit, 0
e_2lit:
	lw wp, 0(ip)
	dpush wp
	lw wp, 4(ip)
	dpush wp
	addi ip, ip, 8
	next

	defword early_txc, "early-txc", e_early_txc, 0
e_early_txc:
	dpop wp
	call early_txc
	next

	defword early_puts, "early-puts", e_early_puts, 0
e_early_puts:
	dpop wp
	call early_puts
	next

	defword doconst, "doconst", e_doconst, 0
e_doconst:
	lw xp, 4(wp)
	dpush xp
	next

	defword true, "true", e_doconst, 0
	.word -1

	defword false, "false", e_doconst, 0
	.word 0

	defword equ, "=", e_equ, 0
e_equ:
	dpop wp
	dpop xp
	li yp, -1
	beq wp, xp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword 0x0, "0x0", e_doconst, 0
	.word 0x0

	defword 0x1, "0x1", e_doconst, 0
	.word 0x1

	defword 0x2, "0x2", e_doconst, 0
	.word 0x2

	defword 0x3, "0x3", e_doconst, 0
	.word 0x3

	defword 0x4, "0x4", e_doconst, 0
	.word 0x4

	defword 0x5, "0x5", e_doconst, 0
	.word 0x5

	defword 0x6, "0x6", e_doconst, 0
	.word 0x6

	defword 0x7, "0x7", e_doconst, 0
	.word 0x7

	defword 0x8, "0x8", e_doconst, 0
	.word 0x8

	defword 0x9, "0x9", e_doconst, 0
	.word 0x9

	defword 0xA, "0xA", e_doconst, 0
	.word 0xA

	defword 0xB, "0xB", e_doconst, 0
	.word 0xB

	defword 0xC, "0xC", e_doconst, 0
	.word 0xC

	defword 0xD, "0xD", e_doconst, 0
	.word 0xD

	defword 0xE, "0xE", e_doconst, 0
	.word 0xE

	defword 0xF, "0xF", e_doconst, 0
	.word 0xF

	defword 0x10, "0x10", e_doconst, 0
	.word 0x10

	defword 0x1F, "0x1F", e_doconst, 0
	.word 0x1F

	defword 0x20, "0x20", e_doconst, 0
	.word 0x20

	defword 0x3F, "0x3F", e_doconst, 0
	.word 0x3F

	defword 0x40, "0x40", e_doconst, 0
	.word 0x40

	defword 0x7F, "0x7F", e_doconst, 0
	.word 0x7F

	defword 0x80, "0x80", e_doconst, 0
	.word 0x80

	defword 0xFF, "0xFF", e_doconst, 0
	.word 0xFF

	defword 0xFFFFFFFF, "0xFFFFFFFF", e_doconst, 0
	.word -1

	defword eqz, "0=", e_call, 0
	.word w_0x0
	.word w_equ
	.word w_exit

	defword load, "@", e_load, 0
e_load:
	dpop wp
	lw xp, 0(wp)
	dpush xp
	next

	defword dp, "dp", e_doconst, 0
	.word dp

	defword here, "here", e_call, 0
	.word w_dp
	.word w_load
	.word w_exit

	defword here_chg, "->here", e_call, 0
	.word w_dp
	.word w_store
	.word w_exit

	defword dup, "dup", e_dup, 0
e_dup:
	dpop wp
	dpush wp
	dpush wp
	next

	defword store, "!", e_store, 0
e_store:
	dpop wp
	dpop xp
	sw xp, 0(wp)
	next

	defword plus, "+", e_plus, 0
e_plus:
	dpop wp
	dpop xp
	add wp, wp, xp
	dpush wp
	next

	defword logand, "&", e_logand, 0
e_logand:
	dpop wp
	dpop xp
	and wp, wp, xp
	dpush wp
	next

	defword logxor, "^", e_logxor, 0
e_logxor:
	dpop wp
	dpop xp
	xor wp, wp, xp
	dpush wp
	next

	defword invert, "invert", e_call, 0
	.word w_0xFFFFFFFF
	.word w_logxor
	.word w_exit

	defword ne, "<>", e_call, 0
	.word w_equ
	.word w_invert
	.word w_exit

	defword nez, "0<>", e_call, 0
	.word w_0x0
	.word w_ne
	.word w_exit

	defword uart1, "uart1", e_doconst, 0
	.word UART1_BASE

	defword uart_stat, "uart-stat", e_doconst, 0
	.word UART_STATR

	defword uart_data, "uart-data", e_doconst, 0
	.word UART_DATAR

	defword uart_stat_load, "uart-stat@", e_call, 0
	.word w_uart_stat
	.word w_plus
	.word w_load
	.word w_exit

	defword uart_flag_tc, "uart-flag-tc", e_doconst, 0
	.word UART_TC

	defword uart_stat_tx, "uart-stat-tx", e_call, 0
	.word w_uart_stat_load
	.word w_uart_flag_tc
	.word w_logand
	.word w_nez
	.word w_exit

	defword pause, "pause", e_pause, 0
e_pause:
	.equ USER_OFFSET_NEXT, 0 * 4
	.equ USER_OFFSET_IP,  1 * 4
	.equ USER_OFFSET_RSP, 2 * 4
	.equ USER_OFFSET_PSP, 3 * 4
	.equ USER_OFFSET_PSB, 4 * 4

	call user_save
	lw up, USER_OFFSET_NEXT(up)
	call user_load
	next

user_save:
	sw ip, USER_OFFSET_IP(up)
	sw rsp, USER_OFFSET_RSP(up)
	sw psp, USER_OFFSET_PSP(up)
	sw psb, USER_OFFSET_PSB(up)
	ret

user_load:
	lw ip, USER_OFFSET_IP(up)
	lw rsp, USER_OFFSET_RSP(up)
	lw psp, USER_OFFSET_PSP(up)
	lw psb, USER_OFFSET_PSB(up)
	ret

	defword uart_wait_tx, "uart-wait-tx", e_call, 0
1:
	.word w_pause
	.word w_dup
	.word w_uart_stat_tx
	.word w_0branch
	.word 1b
	.word w_drop
	.word w_exit

	defword uart_data_store, "uart-data!", e_call, 0
	.word w_uart_data
	.word w_plus
	.word w_store
	.word w_exit

	defword swap, "swap", e_swap, 0
e_swap:
	dpop wp
	dpop xp
	dpush wp
	dpush xp
	next

	defword uart_putc, "uart-putc", e_call, 0
	.word w_dup
	.word w_uart_wait_tx
	.word w_uart_data_store
	.word w_exit

	defword uart1_emit, "uart1-emit", e_call, 0
	.word w_uart1
	.word w_uart_putc
	.word w_exit

	defword execute, "execute", e_execute, 0
e_execute:
	dpop wp
	lw xp, 0(wp)
	jr xp

	defword perform, "perform", e_call ,0
	.word w_load
	.word w_execute
	.word w_exit

	.equ USER_OFFSET_EMIT, (16 * 4)
	defword user_offset_emit, "user-offset-emit", e_doconst, 0
	.word USER_OFFSET_EMIT

	defword user_addr, "user-addr", e_user_addr, 0
e_user_addr:
	dpush up
	next

	defword emit, "emit", e_call, 0
	.word w_user_addr
	.word w_user_offset_emit
	.word w_plus
	.word w_perform
	.word w_exit

	defword cload, "c@", e_cload, 0
e_cload:
	dpop wp
	lbu xp, 0(wp)
	dpush xp
	next

	defword 2drop, "2drop", e_call, 0
	.word w_drop
	.word w_drop
	.word w_exit

	defword 1plus, "1+", e_call ,0
	.word w_0x1
	.word w_plus
	.word w_exit

	defword minus, "-", e_minus, 0
e_minus:
	dpop wp
	dpop xp
	sub xp, xp, wp
	dpush xp
	next

	defword 1minus, "1-", e_call, 0
	.word w_0x1
	.word w_minus
	.word w_exit

	# : type dup if begin swap dup c@ emit 1+ swap 1- dup 0= until then 2drop ;

	defword type, "type", e_call, 0
	.word w_dup
	.word w_0branch
	.word 1f
2:
	.word w_swap
	.word w_dup
	.word w_cload
	.word w_emit
	.word w_1plus
	.word w_swap
	.word w_1minus
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
1:
	.word w_2drop
	.word w_exit

	defword xdigits, "xdigits", e_doconst, 0
	.word xdigits
xdigits:
	.ascii "0123456789ABCDEF"

	# : num2hex 0xF & xdigits + c@ ;

	defword num2hex, "num2hex", e_call, 0
	.word w_0xF
	.word w_logand
	.word w_xdigits
	.word w_plus
	.word w_cload
	.word w_exit

	defword tor, ">r", e_tor, 0
e_tor:
	dpop wp
	rpush wp
	next

	defword fromr, "r>", e_fromr, 0
e_fromr:
	rpop wp
	dpush wp
	next

	defword over, "over", e_call, 0
	.word w_tor
	.word w_dup
	.word w_fromr
	.word w_swap
	.word w_exit

	defword 2dup, "2dup", e_call, 0
	.word w_over
	.word w_over
	.word w_exit

	defword rshift, ">>", e_rshift, 0
e_rshift:
	dpop wp
	dpop xp
	srl xp, xp, wp
	dpush xp
	next

	defword 4minus, "4-", e_call, 0
	.word w_0x4
	.word w_minus
	.word w_exit

	# : hex32 0x20 begin 0x4 - 2dup >> num2hex emit dup 0= until 2drop ;

	defword hex32, "hex32", e_call, 0
	.word w_0x20
1:
	.word w_4minus
	.word w_2dup
	.word w_rshift
	.word w_num2hex
	.word w_emit
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 1b
	.word w_2drop
	.word w_exit

	defword psp_load, "psp@", e_psp_load, 0
e_psp_load:
	dpush psp
	next

	defword psb_load, "psb@", e_psb_load, 0
e_psb_load:
	dpush psb
	next

	defword 4div, "4/", e_call, 0
	.word w_0x2
	.word w_rshift
	.word w_exit

	defword depth, "depth", e_call, 0
	.word w_psp_load
	.word w_psb_load
	.word w_minus
	.word w_4div
	.word w_exit

	defword 4plus, "4+", e_call, 0
	.word w_0x4
	.word w_plus
	.word w_exit

	defword space, "space", e_call, 0
	.word w_0x20
	.word w_emit
	.word w_exit

	defword dsdump, ".s", e_call, 0
	.word w_depth
	.word w_dup

	.word w_lit
	.word '<'
	.word w_emit
	.word w_hex32
	.word w_lit
	.word '>'
	.word w_emit

	.word w_psb_load

	.word w_over
	.word w_0branch
	.word 2f
1:
	.word w_space
	.word w_dup
	.word w_load
	.word w_hex32
	.word w_4plus
	.word w_swap
	.word w_1minus
	.word w_swap
	.word w_over
	.word w_eqz
	.word w_0branch
	.word 1b
2:

	.word w_2drop
	.word w_exit

	defword uart_flag_rxne, "uart-flag-rxne", e_doconst, 0
	.word UART_RXNE

	defword uart_stat_rx, "uart-stat-rx", e_call, 0
	.word w_uart_stat_load
	.word w_uart_flag_rxne
	.word w_logand
	.word w_nez
	.word w_exit

	defword uart_wait_rx, "uart-wait-rx", e_call, 0
1:
	.word w_pause
	.word w_dup
	.word w_uart_stat_rx
	.word w_0branch
	.word 1b
	.word w_drop
	.word w_exit

	defword uart_data_load, "uart-data@", e_call, 0
	.word w_uart_data
	.word w_plus
	.word w_load
	.word w_exit

	defword uart_getc, "uart-getc", e_call, 0
	.word w_dup
	.word w_uart_wait_rx
	.word w_uart_data_load
	.word w_exit

	defword uart1_data_load, "uart1-data@", e_call, 0
	.word w_uart1
	.word w_uart_data_load
	.word w_exit

	defword uart1_key, "uart1-key", e_call, 0
	.word w_uart1
	.word w_uart_getc
	.word w_exit

	defword uart1_keyava, "uart1-key?", e_call, 0
	.word w_uart1
	.word w_uart_stat_rx
	.word w_exit

	.equ USER_OFFSET_KEY, (17 * 4)
	defword user_offset_key, "user-offset-key", e_doconst, 0
	.word USER_OFFSET_KEY

	defword key, "key", e_call, 0
	.word w_user_addr
	.word w_user_offset_key
	.word w_plus
	.word w_perform
	.word w_exit

	.equ USER_OFFSET_KEYAVA, (18 * 4)
	defword user_offset_keyava, "user-offset-key?", e_doconst, 0
	.word USER_OFFSET_KEYAVA

	defword keyava, "key?", e_call, 0
	.word w_user_addr
	.word w_user_offset_keyava
	.word w_plus
	.word w_perform
	.word w_exit

	.equ USER_OFFSET_TIB, (5 * 4)
	defword user_offset_tib, "user-offset-tib", e_doconst, 0
	.word USER_OFFSET_TIB

	defword tib, "tib", e_call, 0
	.word w_user_addr
	.word w_user_offset_tib
	.word w_plus
	.word w_load
	.word w_exit

	.equ USER_OFFSET_TOIN, (6 * 4)
	defword user_offset_toin, "user-offset->in", e_doconst, 0
	.word USER_OFFSET_TOIN

	defword toin, ">in", e_call, 0
	.word w_user_addr
	.word w_user_offset_toin
	.word w_plus
	.word w_exit

	defword toin_load, ">in@", e_call, 0
	.word w_toin
	.word w_load
	.word w_exit

	defword toin_store, ">in!", e_call, 0
	.word w_toin
	.word w_store
	.word w_exit

	defword tib_rst, "tib-rst", e_call, 0
	.word w_0x0
	.word w_toin_store
	.word w_exit

	defword lt, "<", e_lt, 0
e_lt:
	dpop wp
	dpop xp
	li yp, -1
	blt xp, wp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword gt, ">", e_gt, 0
e_gt:
	dpop wp
	dpop xp
	li yp, -1
	bgt xp, wp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword le, "<=", e_le, 0
e_le:
	dpop wp
	dpop xp
	li yp, -1
	ble xp, wp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword ge, ">=", e_ge, 0
e_ge:
	dpop wp
	dpop xp
	li yp, -1
	bge xp, wp, 1f
	li yp, 0
1:
	dpush yp
	next

	defword within, "within", e_within, 0
e_within:
	dpop wp # max
	dpop xp # min
	dpop yp # val
	li zp, 0
	bge yp, xp, 1f
	dpush zp
	next
1:
	blt yp, wp, 1f
	dpush zp
	next
1:
	li zp, -1
	dpush zp
	next

	defword toin_min, "toin-min", e_doconst, 0
	.word 0x0

	defword toin_max, "toin-max", e_doconst, 0
	.word TIBSIZE - 1

	defword tib_chk, "tib-chk", e_call, 0
	.word w_toin_load
	.word w_toin_min
	.word w_toin_max
	.word w_within
	.word w_exit

	defword cstore, "c!", e_cstore, 0
e_cstore:
	dpop wp
	dpop xp
	sb xp, 0(wp)
	next

	defword tib_in, "tib-in", e_call, 0
	.word w_tib_chk
	.word w_invert
	.word w_0branch
	.word 1f
	.word w_tib_rst
	.word w_drop
	.word w_exit
1:
	.word w_tib
	.word w_toin_load
	.word w_plus
	.word w_cstore
	.word w_toin_load
	.word w_1plus
	.word w_toin_store
	.word w_exit

	defword tib_out, "tib-out", e_call, 0
	.word w_toin_load
	.word w_1minus
	.word w_toin_store
	.word w_tib_chk
	.word w_invert
	.word w_0branch
	.word 1f
	.word w_tib_rst
1:
	.word w_tib
	.word w_toin_load
	.word w_plus
	.word w_cload
	.word w_exit

	defword nip, "nip", e_call, 0
	.word w_swap
	.word w_drop
	.word w_exit

	defword logor, "|", e_logor, 0
e_logor:
	dpop wp
	dpop xp
	or xp, xp, wp
	dpush xp
	next

	defword rot, "rot", e_call, 0
	.word w_tor
	.word w_swap
	.word w_fromr
	.word w_swap
	.word w_exit

	defword isspace, "space?", e_call, 0
	.word w_dup
	.word w_lit
	.word ' '
	.word w_equ
	.word w_swap
	.word w_lit
	.word '\t'
	.word w_equ
	.word w_logor
	.word w_exit

	defword isdel, "del?", e_call, 0
	.word w_dup
	.word w_lit
	.word '\b'
	.word w_equ
	.word w_swap
	.word w_0x7F
	.word w_equ
	.word w_logor
	.word w_exit

	defword isnewline, "newline?", e_call, 0
	.word w_dup
	.word w_lit
	.word '\n'
	.word w_equ
	.word w_swap
	.word w_lit
	.word '\r'
	.word w_equ
	.word w_logor
	.word w_exit

	defword cr, "cr", e_call, 0
	.word w_2lit
	.word '\n'
	.word '\r'
	.word w_emit
	.word w_emit
	.word w_exit

	defword _token, "_token", e_call, 0
2:
	.word w_key
	.word w_dup
	.word w_isspace
	.word w_0branch
	.word 1f
	.word w_emit
	.word w_exit
1:
	.word w_dup
	.word w_isnewline
	.word w_0branch
	.word 1f
	.word w_drop
	.word w_cr
	.word w_exit
1:
	.word w_dup
	.word w_isdel
	.word w_0branch
	.word 1f
	.word w_tib_out
	.word w_drop
	.word w_emit
	.word w_branch
	.word 2b
1:
	.word w_dup
	.word w_emit
	.word w_tib_in
	.word w_branch
	.word 2b

	defword token "token", e_call, 0
1:
	.word w_tib_rst
	.word w__token
	.word w_toin_load
	.word w_0branch
	.word 1b
	.word w_exit

	defword ishexhdr, "hexhdr?", e_call, 0
	.word w_0x2
	.word w_gt
	.word w_0branch
	.word 1f
	.word w_dup
	.word w_cload
	.word w_lit
	.word '0'
	.word w_equ
	.word w_swap
	.word w_1plus
	.word w_cload
	.word w_lit
	.word 'x'
	.word w_equ
	.word w_logand
	.word w_exit
1:
	.word w_drop
	.word w_false
	.word w_exit

	defword 2plus, "2+", e_call, 0
	.word w_0x2
	.word w_plus
	.word w_exit

	defword 2minus, "2-", e_call, 0
	.word w_0x2
	.word w_minus
	.word w_exit

	defword isxdigit, "xdigit?", e_call, 0
	.word w_dup
	.word w_lit
	.word '0'
	.word w_lit
	.word '9' + 1
	.word w_within
	.word w_swap
	.word w_lit
	.word 'A'
	.word w_lit
	.word 'F' + 1
	.word w_within
	.word w_logor
	.word w_exit

	defword isnumber, "number?", e_call, 0
	.word w_2dup
	.word w_ishexhdr
	.word w_invert
	.word w_0branch
	.word 1f
	.word w_2drop
	.word w_false
	.word w_exit
1:
	.word w_2minus
	.word w_swap
	.word w_2plus
	.word w_swap
2:
	.word w_swap
	.word w_dup
	.word w_cload
	.word w_isxdigit
	.word w_invert
	.word w_0branch
	.word 1f
	.word w_2drop
	.word w_false
	.word w_exit
1:

	.word w_1plus
	.word w_swap
	.word w_1minus

	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
	.word w_2drop
	.word w_true
	.word w_exit

	defword lshift, "<<", e_lshift, 0
e_lshift:
	dpop wp
	dpop xp
	sll xp, xp, wp
	dpush xp
	next

	defword 4multi, "4*", e_call, 0
	.word w_0x2
	.word w_lshift
	.word w_exit

	defword 2swap, "2swap", e_call, 0
	.word w_rot
	.word w_tor
	.word w_rot
	.word w_fromr
	.word w_exit

	defword hex2num, "hex2num", e_call, 0
	.word w_dup
	.word w_lit
	.word '9' + 1
	.word w_lt
	.word w_0branch
	.word 1f
	.word w_lit
	.word '0'
	.word w_minus
	.word w_exit
1:
	.word w_lit
	.word 'A' - 0xA
	.word w_minus
	.word w_exit

	defword number, "number", e_call, 0
	.word w_2dup
	.word w_isnumber
	.word w_invert
	.word w_0branch
	.word 1f
	.word w_2drop
	.word w_false
	.word w_exit
1:

	.word w_2minus
	.word w_swap
	.word w_2plus
	.word w_swap

	.word w_dup
	.word w_1minus
	.word w_4multi
	.word w_0x0

	# ( addr u shi out )
1:
	.word w_2swap
	.word w_swap
	.word w_dup
	.word w_cload
	.word w_hex2num
	.word w_tor
	.word w_1plus
	.word w_swap
	.word w_1minus

	.word w_2swap
	.word w_over
	.word w_fromr
	.word w_swap
	.word w_lshift
	.word w_logor

	.word w_swap
	.word w_4minus
	.word w_swap

	.word w_2swap
	.word w_dup
	.word w_tor
	.word w_2swap
	.word w_fromr

	.word w_eqz
	.word w_0branch
	.word 1b
	.word w_nip
	.word w_nip
	.word w_nip
	.word w_exit

	defword min, "min", e_min, 0
e_min:
	dpop wp
	dpop xp
	blt xp, wp, 1f
	dpush wp
	next
1:
	dpush xp
	next

	defword max, "min", e_max, 0
e_max:
	dpop wp
	dpop xp
	bgt xp, wp, 1f
	dpush wp
	next
1:
	dpush xp
	next

	defword compare, "compare", e_call, 0
	.word w_rot
	.word w_min

	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 1f
	.word w_nip
	.word w_nip
	.word w_exit
1:
2:
	.word w_rot
	.word w_dup
	.word w_cload
	.word w_tor
	.word w_1plus

	.word w_rot
	.word w_dup
	.word w_cload
	.word w_tor
	.word w_1plus

	.word w_rot
	.word w_1minus

	.word w_fromr
	.word w_fromr
	.word w_minus
	.word w_dup
	.word w_0branch
	.word 1f
	.word w_nip
	.word w_nip
	.word w_nip
	.word w_exit
1:
	.word w_drop
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
	.word w_nip
	.word w_nip
	.word w_exit

	.equ WORD_OFFSET_ENTR, -(0 * 4)
	defword word_offset_entr, "word-offset-entr", e_doconst, 0
	.word WORD_OFFSET_ENTR

	defword xtentr, "xtentr", e_call, 0
	.word w_word_offset_entr
	.word w_plus
	.word w_load
	.word w_exit

	.equ WORD_OFFSET_LINK, -(1 * 4)
	defword word_offset_link, "word-offset-link", e_doconst, 0
	.word WORD_OFFSET_LINK

	defword xtlink, "xtlink", e_call, 0
	.word w_word_offset_link
	.word w_plus
	.word w_load
	.word w_exit

	.equ WORD_OFFSET_ATTR, -(2 * 4)
	defword word_offset_attr, "word-offset-attr", e_doconst, 0
	.word WORD_OFFSET_ATTR

	defword xtattr, "xtattr", e_call, 0
	.word w_word_offset_attr
	.word w_plus
	.word w_load
	.word w_exit

	.equ WORD_MASK_NLEN, 0xFF
	defword word_mask_nlen, "word-mask-nlen", e_doconst, 0
	.word WORD_MASK_NLEN

	defword xtnlen, "xtnlen", e_call, 0
	.word w_xtattr
	.word w_word_mask_nlen
	.word w_logand
	.word w_exit

	defword aligned, "aligned", e_call, 0
	.word w_0x3
	.word w_plus
	.word w_lit
	.word -4
	.word w_logand
	.word w_exit

	defword xtname, "xtname", e_call, 0
	.word w_word_offset_attr
	.word w_over
	.word w_xtnlen
	.word w_aligned
	.word w_minus
	.word w_plus
	.word w_exit

	defword latest, "latest", e_call ,0
	.word w_lit
	.word latest
	.word w_load
	.word w_exit

	defword latest_chg, "->latest", e_call, 0
	.word w_lit
	.word latest
	.word w_store
	.word w_exit

	defword words, "words", e_call, 0
	.word w_latest

1:
	.word w_dup
	.word w_xtishide
	.word w_invert
	.word w_0branch
	.word 2f

	.word w_dup
	.word w_xtname
	.word w_over
	.word w_xtnlen
	.word w_type
	.word w_space
2:
	.word w_xtlink
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 1b
	.word w_drop
	.word w_exit

	defword 2over, "2over", e_call, 0
	.word w_tor
	.word w_tor
	.word w_2dup
	.word w_fromr
	.word w_fromr
	.word w_2swap
	.word w_exit

	defword find, "find", e_call, 0
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 1f
	.word w_nip
	.word w_exit
1:
	.word w_latest

2:
	.word w_dup
	.word w_xtishide
	.word w_invert
	.word w_0branch
	.word 3f

	.word w_2dup
	.word w_xtnlen
	.word w_equ
	.word w_0branch
	.word 1f
	.word w_dup
	.word w_xtname
	.word w_2over
	.word w_swap
	.word w_over
	.word w_compare
	.word w_eqz
	.word w_0branch
	.word 3f
	.word w_nip
	.word w_nip
	.word w_exit
3:
1:
	.word w_xtlink
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
	.word w_nip
	.word w_nip
	.word w_exit

	defword dot, ".", e_call, 0
	.word w_hex32
	.word w_exit

	defword psp_rst, "psp-rst", e_psp_rst, 0
e_psp_rst:
	mv psp, psb
	next

	defword psp_chk, "psp-chk", e_call, 0
	.word w_psp_load
	.word w_psb_load
	.word w_lt
	.word w_0branch
	.word 1f
	.word w_psp_rst
	.word w_lit
	.word msg_stkerr
	.word w_lit
	.word msg_len_stkerr
	.word w_type
	.word w_cr
1:
	.word w_exit

	.section .rodata
msg_stkerr:
	.ascii "stack error"
	.set msg_len_stkerr, . - msg_stkerr
	.section .text

	.equ USER_OFFSET_STATE, (7 * 4)
	defword user_offset_state, "user-offset-state", e_doconst, 0
	.word USER_OFFSET_STATE

	defword state, "state", e_call, 0
	.word w_user_addr
	.word w_user_offset_state
	.word w_plus
	.word w_exit

	defword on, "on", e_call, 0
	.word w_true
	.word w_swap
	.word w_store
	.word w_exit

	defword off, "off", e_call, 0
	.word w_false
	.word w_swap
	.word w_store
	.word w_exit

	.equ ATTR_FLAG_COMP, (1 << 8)
	defword attr_flag_comp, "attr-flag-comp", e_doconst, 0
	.word ATTR_FLAG_COMP

	defword xtisimm, "xtimm?", e_call, 0
	.word w_xtattr
	.word w_attr_flag_comp
	.word w_logand
	.word w_nez
	.word w_exit

	defword compoff, "[", e_call, ATTR_FLAG_COMP
	.word w_state
	.word w_off
	.word w_exit

	defword compon, "]", e_call, 0
	.word w_state
	.word w_on
	.word w_exit

	defword quest, "?", e_call, 0
	.word w_load
	.word w_dot
	.word w_exit

	defword comma, ",", e_call, 0
	.word w_here
	.word w_store
	.word w_here
	.word w_4plus
	.word w_here_chg
	.word w_exit

	defword align, "align", e_call, 0
	.word w_here
	.word w_aligned
	.word w_here_chg
	.word w_exit

	defword move, "move", e_call, 0
	.word w_dup
	.word w_0branch
	.word 1f
2:
	.word w_tor
	.word w_over
	.word w_load
	.word w_over
	.word w_store
	.word w_4plus
	.word w_swap
	.word w_4plus
	.word w_swap
	.word w_fromr
	.word w_1minus
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
1:
	.word w_drop
	.word w_2drop
	.word w_exit

	.equ ATTR_FLAG_HIDE, (1 << 9)
	defword attr_flag_hide, "attr-flag-hide", e_doconst, 0
	.word ATTR_FLAG_HIDE

	# ( addr u )
	defword defword, "defword", e_call, 0
	.word w_align

	.word w_2dup
	.word w_aligned
	.word w_here
	.word w_swap
	.word w_4div
	.word w_move

	.word w_dup
	.word w_aligned
	.word w_here
	.word w_plus
	.word w_here_chg

	.word w_attr_flag_hide
	.word w_logor
	.word w_comma
	.word w_drop

	.word w_latest
	.word w_comma

	.word w_lit
	.word name_call
	.word w_lit
	.word nlen_call
	.word w_find
	.word w_xtentr
	.word w_here
	.word w_latest_chg
	.word w_comma

	.word w_exit

	defword bitclr, "bitclr", e_call, 0
	.word w_invert
	.word w_logand
	.word w_exit
	
	defword xtattr_chg, "->xtattr", e_call, 0
	.word w_word_offset_attr
	.word w_plus
	.word w_store
	.word w_exit

	defword unhide, "unhide", e_call, 0
	.word w_dup
	.word w_xtattr
	.word w_attr_flag_hide
	.word w_bitclr
	.word w_swap
	.word w_xtattr_chg
	.word w_exit

	defword hide, "hide", e_call, 0
	.word w_dup
	.word w_xtattr
	.word w_attr_flag_hide
	.word w_logor
	.word w_swap
	.word w_xtattr_chg
	.word w_exit

	defword xtishide, "xthide?", e_call, 0
	.word w_xtattr
	.word w_attr_flag_hide
	.word w_logand
	.word w_nez
	.word w_exit

	defword colon, ":", e_call, 0
	.word w_token
	.word w_tib
	.word w_toin_load
	.word w_defword
	.word w_compon
	.word w_exit

	defword semicolon ";", e_call, ATTR_FLAG_COMP
	.word w_lit
	.word name_exit
	.word w_lit
	.word nlen_exit
	.word w_find
	.word w_comma
	.word w_compoff
	.word w_latest
	.word w_unhide
	.word w_exit

	defword if, "if", e_call, ATTR_FLAG_COMP
	.word w_lit
	.word name_0branch
	.word w_lit
	.word nlen_0branch
	.word w_find
	.word w_comma
	.word w_here
	.word w_0xFFFFFFFF
	.word w_comma
	.word w_exit

	defword then, "then", e_call, ATTR_FLAG_COMP
	.word w_here
	.word w_swap
	.word w_store
	.word w_exit

	defword begin, "begin", e_call, ATTR_FLAG_COMP
	.word w_here
	.word w_exit

	defword again, "again", e_call, ATTR_FLAG_COMP
	.word w_lit
	.word name_branch
	.word w_lit
	.word nlen_branch
	.word w_find
	.word w_comma
	.word w_comma
	.word w_exit

	defword until, "until", e_call, ATTR_FLAG_COMP
	.word w_lit
	.word name_0branch
	.word w_lit
	.word nlen_0branch
	.word w_find
	.word w_comma
	.word w_comma
	.word w_exit

	defword tick, "'", e_call, ATTR_FLAG_COMP
	.word w_token
	.word w_tib
	.word w_toin_load
	.word w_find

	.word w_state
	.word w_load
	.word w_0branch
	.word 1f
	.word w_lit
	.word name_lit
	.word w_lit
	.word nlen_lit
	.word w_find
	.word w_comma
	.word w_comma
1:
	.word w_exit

	defword xtentr_chg, "->xtentr", e_call, 0
	.word w_word_offset_entr
	.word w_plus
	.word w_store
	.word w_exit

	defword constant, "constant", e_call, 0
	.word w_colon
	.word w_compoff
	.word w_lit
	.word name_doconst
	.word w_lit
	.word nlen_doconst
	.word w_find
	.word w_xtentr
	.word w_latest
	.word w_xtentr_chg
	.word w_comma
	.word w_latest
	.word w_unhide
	.word w_exit

	defword irq_exit, "irq-exit", e_irq_exit, 0
	.equ REG_SIZE, 4
	.equ REG_OFFSET_ZERO, (0 * REG_SIZE)
	.equ REG_OFFSET_RA, (1 * REG_SIZE)
	.equ REG_OFFSET_SP, (2 * REG_SIZE)
	.equ REG_OFFSET_GP, (3 * REG_SIZE)
	.equ REG_OFFSET_TP, (4 * REG_SIZE)
	.equ REG_OFFSET_T0, (5 * REG_SIZE)
	.equ REG_OFFSET_T1, (6 * REG_SIZE)
	.equ REG_OFFSET_T2, (7 * REG_SIZE)
	.equ REG_OFFSET_S0, (8 * REG_SIZE)
	.equ REG_OFFSET_S1, (9 * REG_SIZE)
	.equ REG_OFFSET_A0, (10 * REG_SIZE)
	.equ REG_OFFSET_A1, (11 * REG_SIZE)
	.equ REG_OFFSET_A2, (12 * REG_SIZE)
	.equ REG_OFFSET_A3, (13 * REG_SIZE)
	.equ REG_OFFSET_A4, (14 * REG_SIZE)
	.equ REG_OFFSET_A5, (15 * REG_SIZE)
	.equ REG_OFFSET_A6, (16 * REG_SIZE)
	.equ REG_OFFSET_A7, (17 * REG_SIZE)
	.equ REG_OFFSET_S2, (18 * REG_SIZE)
	.equ REG_OFFSET_S3, (19 * REG_SIZE)
	.equ REG_OFFSET_S4, (20 * REG_SIZE)
	.equ REG_OFFSET_S5, (21 * REG_SIZE)
	.equ REG_OFFSET_S6, (22 * REG_SIZE)
	.equ REG_OFFSET_S7, (23 * REG_SIZE)
	.equ REG_OFFSET_S8, (24 * REG_SIZE)
	.equ REG_OFFSET_S9, (25 * REG_SIZE)
	.equ REG_OFFSET_S10, (26 * REG_SIZE)
	.equ REG_OFFSET_S11, (27 * REG_SIZE)
	.equ REG_OFFSET_T3, (28 * REG_SIZE)
	.equ REG_OFFSET_T4, (29 * REG_SIZE)
	.equ REG_OFFSET_T5, (30 * REG_SIZE)
	.equ REG_OFFSET_T6, (31 * REG_SIZE)
	.equ REGS_SIZE, (32 * REG_SIZE)
e_irq_exit:
	la wp, in_irq
	sw zero, 0(wp)

	lw zero, REG_OFFSET_ZERO(sp)
	lw ra, REG_OFFSET_RA(sp)
	# not need restore sp
	lw gp, REG_OFFSET_GP(sp)
	lw tp, REG_OFFSET_TP(sp)
	lw t0, REG_OFFSET_T0(sp)
	lw t1, REG_OFFSET_T1(sp)
	lw t2, REG_OFFSET_T2(sp)
	lw s0, REG_OFFSET_S0(sp)
	lw s1, REG_OFFSET_S1(sp)
	lw a0, REG_OFFSET_A0(sp)
	lw a1, REG_OFFSET_A1(sp)
	lw a2, REG_OFFSET_A2(sp)
	lw a3, REG_OFFSET_A3(sp)
	lw a4, REG_OFFSET_A4(sp)
	lw a5, REG_OFFSET_A5(sp)
	lw a6, REG_OFFSET_A6(sp)
	lw a7, REG_OFFSET_A7(sp)
	lw s2, REG_OFFSET_S2(sp)
	lw s3, REG_OFFSET_S3(sp)
	lw s4, REG_OFFSET_S4(sp)
	lw s5, REG_OFFSET_S5(sp)
	lw s6, REG_OFFSET_S6(sp)
	lw s7, REG_OFFSET_S7(sp)
	lw s8, REG_OFFSET_S8(sp)
	lw s9, REG_OFFSET_S9(sp)
	lw s10, REG_OFFSET_S10(sp)
	lw s11, REG_OFFSET_S11(sp)
	lw t3, REG_OFFSET_T3(sp)
	lw t4, REG_OFFSET_T4(sp)
	lw t5, REG_OFFSET_T5(sp)
	lw t6, REG_OFFSET_T6(sp)
	addi sp, sp, REGS_SIZE
	mret

	defword mcause_load, "mcause@", e_mcause_load, 0
e_mcause_load:
	csrr wp, mcause
	dpush wp
	next

	defword mtval_load, "mtval@", e_mtval_load, 0
e_mtval_load:
	csrr wp, mtval
	dpush wp
	next

	defword mepc_load, "mepc@", e_mepc_load, 0
e_mepc_load:
	csrr wp, mepc
	dpush wp
	next

	defword reg_sp_load, "reg-sp@", e_reg_sp_load, 0
e_reg_sp_load:
	dpush sp
	next

	defword dump, "dump", e_call, 0
	.word w_dup
	.word w_0branch
	.word 1f

2:
	.word w_cr
	.word w_over
	.word w_hex32
	.word w_lit
	.word ':'
	.word w_emit
	.word w_space
	.word w_over
	.word w_load
	.word w_hex32
	.word w_1minus
	.word w_swap
	.word w_4plus
	.word w_swap

	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
1:
	.word w_2drop
	.word w_cr
	.word w_exit

	defword regs_dump, "regs-dump", e_call, 0
	.section .rodata
str_mcause:
	.ascii "\n\rMCAUSE: "
	.set len_str_mcause, . - str_mcause
str_mtval:
	.ascii "\n\rMTVAL: "
	.set len_str_mtval, . - str_mtval
str_mepc:
	.ascii "\n\rMEPC: "
	.set len_str_mepc, . - str_mepc
	.section .text
	.word w_lit
	.word str_mcause
	.word w_lit
	.word len_str_mcause
	.word w_type
	.word w_mcause_load
	.word w_hex32

	.word w_lit
	.word str_mtval
	.word w_lit
	.word len_str_mtval
	.word w_type
	.word w_mtval_load
	.word w_hex32

	.word w_lit
	.word str_mepc
	.word w_lit
	.word len_str_mepc
	.word w_type
	.word w_mepc_load
	.word w_hex32

	.word w_reg_sp_load
	.word w_lit
	.word 32
	.word w_dump

	.word w_exit

	defword forth_irq_base, "forth-irq-base", e_doconst, 0
	.word forth_irq_base

	defword uart_ctlr1, "uart-ctlr1", e_doconst, 0
	.word UART_CTLR1

	defword uart_ctlr1_load, "uart-ctlr1@", e_call, 0
	.word w_uart_ctlr1
	.word w_plus
	.word w_load
	.word w_exit

	defword uart_ctlr1_store, "uart-ctlr1!", e_call, 0
	.word w_uart_ctlr1
	.word w_plus
	.word w_store
	.word w_exit

	.equ UART_RXNEIE, (1 << 5)
	defword uart_flag_drie, "uart-flag-drie", e_doconst, 0
	.word UART_RXNEIE

	defword uart_dri_en, "uart-dri-en", e_call, 0
	.word w_dup
	.word w_uart_ctlr1_load
	.word w_uart_flag_drie
	.word w_logor
	.word w_swap
	.word w_uart_ctlr1_store
	.word w_exit

	defword uart_dri_dis, "uart-dri-dis", e_call, 0
	.word w_dup
	.word w_uart_ctlr1_load
	.word w_uart_flag_drie
	.word w_bitclr
	.word w_swap
	.word w_uart_ctlr1_store
	.word w_exit

	defword allot, "allot", e_call, 0
	.word w_here
	.word w_plus
	.word w_here_chg
	.word w_exit

	defword cell, "cell", e_doconst, 0
	.word 4

	defword cells, "cells", e_call, 0
	.word w_4multi
	.word w_exit

	defword variable, "variable", e_call, 0
	.word w_colon
	.word w_latest
	.word w_unhide
	.word w_compoff

	.word w_lit
	.word name_doconst
	.word w_lit
	.word nlen_doconst
	.word w_find
	.word w_xtentr
	.word w_latest
	.word w_xtentr_chg
	.word w_here
	.word w_4plus
	.word w_comma
	.word w_exit

	defword blank, "blank", e_call, 0
	.word w_dup
	.word w_0branch
	.word 1f
2:
	.word w_swap
	.word w_0x0
	.word w_over
	.word w_cstore
	.word w_1plus
	.word w_swap
	.word w_1minus
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
1:
	.word w_2drop
	.word w_exit

	defword fill, "fill", e_call, 0
	.word w_over
	.word w_0branch
	.word 1f
2:
	.word w_rot
	.word w_2dup
	.word w_cstore
	.word w_1plus
	.word w_rot
	.word w_1minus
	.word w_rot
	.word w_over
	.word w_eqz
	.word w_0branch
	.word 2b
1:
	.word w_drop
	.word w_2drop
	.word w_exit

	defword pad, "pad", e_call, 0
	.word w_here
	.word w_lit
	.word 0x100
	.word w_plus
	.word w_exit

	defword paren, "(", e_call, ATTR_FLAG_COMP
1:
	.word w_token
	.word w_toin_load
	.word w_0x1
	.word w_equ
	.word w_tib
	.word w_cload
	.word w_lit
	.word ')'
	.word w_equ
	.word w_logand
	.word w_0branch
	.word 1b
	.word w_exit

	defword ccomma, "c,", e_call, 0
	.word w_here
	.word w_cstore
	.word w_here
	.word w_1plus
	.word w_here_chg
	.word w_exit

	defword cmove, "cmove", e_call, 0
	.word w_dup
	.word w_0branch
	.word 1f
2:
	.word w_tor
	.word w_over
	.word w_cload
	.word w_over
	.word w_cstore
	.word w_1plus
	.word w_swap
	.word w_1plus
	.word w_swap
	.word w_fromr
	.word w_1minus
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 2b
1:
	.word w_drop
	.word w_2drop
	.word w_exit

	defword quote, "\"", e_call, ATTR_FLAG_COMP
	.word w_lit
	.word name_branch
	.word w_lit
	.word nlen_branch
	.word w_find
	.word w_comma
	.word w_here
	.word w_0xFFFFFFFF
	.word w_comma

2:
	.word w_token
	.word w_toin_load
	.word w_0x1
	.word w_equ
	.word w_tib
	.word w_cload
	.word w_lit
	.word '\"'
	.word w_equ
	.word w_logand
	.word w_0branch
	.word 1f
	.word w_here
	.word w_over
	.word w_4plus
	.word w_minus
	.word w_tor
	.word w_align
	.word w_here
	.word w_over
	.word w_store
	.word w_4plus
	.word w_fromr

	.word w_state
	.word w_load
	.word w_0branch
	.word 3f
	.word w_lit
	.word name_lit
	.word w_lit
	.word nlen_lit
	.word w_find
	.word w_rot
	.word w_over
	.word w_comma
	.word w_comma
	.word w_comma
	.word w_comma
	.word w_exit
3:
	.word w_exit
1:
	.word w_tib
	.word w_here
	.word w_toin_load
	.word w_cmove
	.word w_here
	.word w_toin_load
	.word w_plus
	.word w_here_chg

	.word w_0x20
	.word w_ccomma

	.word w_branch
	.word 2b

	.equ FIFO_OFFSET_MASK, (0 * 4)
	defword fifo_offset_mask, "fifo-offset-mask", e_doconst, 0
	.word FIFO_OFFSET_MASK

	.equ FIFO_OFFSET_HEAD, (1 * 4)
	defword fifo_offset_head, "fifo-offset-head", e_doconst, 0
	.word FIFO_OFFSET_HEAD

	.equ FIFO_OFFSET_NUM, (2 * 4)
	defword fifo_offset_num, "fifo-offset-num", e_doconst, 0
	.word FIFO_OFFSET_NUM

	.equ FIFO_OFFSET_BUF, (3 * 4)
	defword fifo_offset_buf, "fifo-offset-buf", e_doconst, 0
	.word FIFO_OFFSET_BUF

	defword fifo_new, "fifo-new", e_call, 0
	.word w_variable
	.word w_dup
	.word w_comma
	.word w_0x0
	.word w_comma
	.word w_0x0
	.word w_comma
	.word w_dup
	.word w_here
	.word w_plus
	.word w_here_chg
	.word w_allot
	.word w_align
	.word w_latest
	.word w_execute
	.word w_fifo_rst
	.word w_exit

	defword fifo_mask_load, "fifo-mask@", e_call, 0
	.word w_fifo_offset_mask
	.word w_plus
	.word w_load
	.word w_exit

	defword fifo_head_load, "fifo-head@", e_call, 0
	.word w_fifo_offset_head
	.word w_plus
	.word w_load
	.word w_exit

	defword fifo_head_store, "fifo-head!", e_call, 0
	.word w_fifo_offset_head
	.word w_plus
	.word w_store
	.word w_exit

	defword fifo_num_load, "fifo-num@", e_call, 0
	.word w_fifo_offset_num
	.word w_plus
	.word w_load
	.word w_exit

	defword fifo_num_store, "fifo-num!", e_call, 0
	.word w_fifo_offset_num
	.word w_plus
	.word w_store
	.word w_exit

	defword fifo_buf, "fifo-buf", e_call, 0
	.word w_fifo_offset_buf
	.word w_plus
	.word w_exit

	defword fifo_rst, "fifo-rst", e_call, 0
	.word w_dup
	.word w_fifo_head_rst
	.word w_fifo_num_rst
	.word w_exit

	defword fifo_used, "fifo-used", e_call, 0
	.word w_fifo_num_load
	.word w_exit

	defword fifo_free, "fifo-free", e_call, 0
	.word w_dup
	.word w_fifo_mask_load
	.word w_swap
	.word w_fifo_num_load
	.word w_minus
	.word w_exit

	defword lez, "0<=", e_lez, 0
e_lez:
	dpop wp
	li yp, -1
	blez wp, 1f
	li yp, 0
1:	
	dpush yp
	next

	defword fifo_is_full, "fifo-full?", e_call, 0
	.word w_fifo_free
	.word w_lez
	.word w_exit

	defword fifo_is_empty, "fifo-empty?", e_call, 0
	.word w_fifo_used
	.word w_eqz
	.word w_exit

	defword modulus, "%", e_modulus, 0
e_modulus:
	dpop wp
	dpop xp
	rem xp, xp, wp
	dpush xp
	next

	defword fifo_num_chk, "fifo-num-chk", e_call, 0
	.word w_dup
	.word w_fifo_num_load
	.word w_swap
	.word w_fifo_mask_load
	.word w_0x0
	.word w_swap
	.word w_within
	.word w_exit

	defword fifo_num_rst, "fifo-num-rst", e_call, 0
	.word w_0x0
	.word w_swap
	.word w_fifo_num_store
	.word w_exit

	defword fifo_head_chk, "fifo-head-chk", e_call, 0
	.word w_dup
	.word w_fifo_head_load
	.word w_swap
	.word w_fifo_mask_load
	.word w_0x0
	.word w_swap
	.word w_within
	.word w_exit

	defword fifo_head_rst, "fifo-head-rst", e_call, 0
	.word w_0x0
	.word w_swap
	.word w_fifo_head_store
	.word w_exit

	defword fifo_num_1plus, "fifo-num-1+", e_call, 0
	.word w_dup
	.word w_fifo_num_load
	.word w_1plus
	.word w_over
	.word w_fifo_num_store
	.word w_dup
	.word w_fifo_num_chk
	.word w_0branch
	.word 1f
	.word w_drop
	.word w_exit
1:
	.word w_fifo_num_rst
	.word w_exit

	defword fifo_num_1minus, "fifo-num-1-", e_call, 0
	.word w_dup
	.word w_fifo_num_load
	.word w_1minus
	.word w_over
	.word w_fifo_num_store
	.word w_dup
	.word w_fifo_num_chk
	.word w_0branch
	.word 1f
	.word w_drop
	.word w_exit
1:
	.word w_fifo_num_rst
	.word w_exit


	defword fifo_head_1plus, "fifo-head-1+", e_call, 0
	.word w_dup
	.word w_fifo_head_load
	.word w_1plus
	.word w_over
	.word w_fifo_head_store
	.word w_dup
	.word w_fifo_head_chk
	.word w_0branch
	.word 1f
	.word w_drop
	.word w_exit
1:
	.word w_fifo_head_rst
	.word w_exit

	defword fifo_head_1minus, "fifo-head-1-", e_call, 0
	.word w_dup
	.word w_fifo_head_load
	.word w_1minus
	.word w_over
	.word w_fifo_head_store
	.word w_dup
	.word w_fifo_head_chk
	.word w_0branch
	.word 1f
	.word w_drop
	.word w_exit
1:
	.word w_fifo_head_rst
	.word w_exit

	defword fifo_push, "fifo-push", e_call, 0
	.word w_dup
	.word w_fifo_head_load
	.word w_over
	.word w_fifo_num_load
	.word w_plus
	.word w_over
	.word w_fifo_mask_load
	.word w_modulus
	.word w_over
	.word w_fifo_buf
	.word w_plus
	.word w_swap
	.word w_fifo_num_1plus
	.word w_cstore
	.word w_exit

	.macro _irq_en
		la wp, in_irq
		lw xp, 0(wp)
		bnez xp, 1000f
		li wp, 0x88
		csrs 0x800, wp
	1000:
	.endm

	defword irq_en, "irq-en", e_irq_en, 0
e_irq_en:
	_irq_en
	next

	.macro _irq_dis
		la wp, in_irq
		lw xp, 0(wp)
		bnez xp, 1001f
		li wp, 0x88
		csrc 0x800, wp
	1001:
	.endm

	defword irq_dis, "irq-dis", e_irq_dis, 0
e_irq_dis:
	_irq_dis
	next

	defword fifo_pop, "fifo-pop", e_fifo_pop, 0
	.word w_dup
	.word w_fifo_head_load
	.word w_over
	.word w_fifo_buf
	.word w_plus
	.word w_cload
	.word w_tor

	.word w_dup
	.word w_fifo_head_1plus
	.word w_fifo_num_1minus

	.word w_fromr
	.word w_exit

e_fifo_pop:
	_irq_dis
	dpop wp
	lw xp, FIFO_OFFSET_HEAD(wp)
	add yp, wp, xp
	lbu yp, FIFO_OFFSET_BUF(yp)
	dpush yp

	lw zp, FIFO_OFFSET_MASK(wp)

	addi xp, xp, 1
	and xp, xp, zp
	sw xp, FIFO_OFFSET_HEAD(wp)

	lw xp, FIFO_OFFSET_NUM(wp)
	addi xp, xp, -1
	and xp, xp, zp
	sw xp, FIFO_OFFSET_NUM(wp)

	_irq_en
	next

	defword uart1_rxfifo, "uart1-rxfifo", e_doconst, 0
	.word uart1_rxfifo

	defword uart1_rxfifo_push, "uart1-rxfifo-push", e_call, 0
	.word w_uart1_rxfifo
	.word w_irq_dis
	.word w_fifo_push
	.word w_irq_en
	.word w_exit

	defword uart1_rxfifo_pop, "uart1-rxfifo-pop", e_call, 0
	.word w_uart1_rxfifo
	.word w_irq_dis
	.word w_fifo_pop
	.word w_irq_en
	.word w_exit

	defword uart1_rxfifo_used, "uart1-rxfifo-used", e_call, 0
	.word w_uart1_rxfifo
	.word w_fifo_used
	.word w_exit

	defword uart1_rxfifo_free, "uart1-rxfifo-free", e_call, 0
	.word w_uart1_rxfifo
	.word w_fifo_free
	.word w_exit

	defword uart1_rxfifo_rst, "uart1-rxfifo-rst", e_call, 0
	.word w_uart1_rxfifo
	.word w_fifo_rst
	.word w_exit

	defword uart1_rxfifo_is_full, "uart1-rxfifo-full?", e_call, 0
	.word w_uart1_rxfifo
	.word w_fifo_is_full
	.word w_exit

	defword uart1_rxfifo_is_empty, "uart1-rxfifo-empty?", e_call, 0
	.word w_uart1_rxfifo
	.word w_fifo_is_empty
	.word w_exit

	defword uart1_rxfifo_keyava, "uart1-rxfifo-key?", e_call, 0
	.word w_uart1_rxfifo_is_empty
	.word w_invert
	.word w_exit

	defword uart1_rxfifo_wait, "uart1-rxfifo-wait", e_call, 0
1:
	.word w_pause
	.word w_uart1_rxfifo_keyava
	.word w_0branch
	.word 1b
	.word w_exit

	defword uart1_rxfifo_key, "uart1-rxfifo-key", e_call, 0
	.word w_uart1_rxfifo_wait
	.word w_uart1_rxfifo_pop
	.word w_exit

	defword irq_no_uart1, "irq-no-uart1", e_doconst, 0
	.word 53

	defword pfic, "pfic", e_doconst, 0
	.word 0xE000E000

	defword pfic_sctlr, "pfic-sctlr", e_doconst, 0
	.word 0xD10

	defword pfic_flag_sysrst, "pfic-flag-sysrst", e_doconst, 0
	.word (1 << 31)

	defword sysrst, "sysrst", e_call, 0
	.word w_pfic_flag_sysrst
	.word w_pfic
	.word w_pfic_sctlr
	.word w_plus
	.word w_store
1:
	.word w_branch
	.word 1b

	defword pfic_ienr1, "pfic-ienr1", e_doconst, 0
	.word 0x100

	defword pfic_ienr2, "pfic-ienr2", e_doconst, 0
	.word 0x104

	defword pfic_ienr3, "pfic-ienr3", e_doconst, 0
	.word 0x108

	defword pfic_ienr4, "pfic-ienr4", e_doconst, 0
	.word 0x10C

	defword division, "/", e_division, 0
e_division:
	dpop wp
	dpop xp
	div xp, xp, wp
	dpush xp
	next

	defword 32div, "32/", e_call, 0
	.word w_0x5
	.word w_rshift
	.word w_exit

	defword pfic_irq_en, "pfic-irq-en", e_call, 0
	.word w_dup
	.word w_32div
	.word w_4multi
	.word w_pfic
	.word w_pfic_ienr1
	.word w_plus
	.word w_plus
	.word w_dup
	.word w_load

	.word w_rot
	.word w_0x20
	.word w_modulus
	.word w_0x1
	.word w_swap
	.word w_lshift
	.word w_logor

	.word w_swap
	.word w_store
	.word w_exit

	defword dummy_key, "dummy-key", e_call, 0
1:
	.word w_pause
	.word w_branch
	.word 1b

	defword multi, "*", e_multi, 0
e_multi:
	dpop wp
	dpop xp
	mul xp, xp, wp
	dpush xp
	next

	defword 2multi, "2*", e_call, 0
	.word w_0x1
	.word w_lshift
	.word w_exit

	defword clk_init, "clk-init", e_clk_init, 0
e_clk_init:
	call _clk_init
	next

_clk_init:
	.equ RCC_BASE, 0x40021000
	.equ RCC_CTLR, 0x0
	.equ RCC_CFGR0, 0x4

	la wp, RCC_BASE
	lw xp, RCC_CTLR(wp)
	.equ RCC_HSEON, (1 << 16)
	li yp, RCC_HSEON
	or xp, xp, yp
	sw xp, RCC_CTLR(wp)
	.equ RCC_HSERDY, (1 << 17)
	li yp, RCC_HSERDY
1:	
	nop
	nop
	nop
	nop
	lw xp, RCC_CTLR(wp)
	and xp, xp, yp
	beqz xp, 1b

	.equ RCC_MCO_SYSCLK, (0x4 << 24)
	.equ RCC_USB_RRE_96M, (0x1 << 22)
	.equ RCC_PLL_MUL_96M, (0xA << 18)
	.equ RCC_PLL_SRC_HSE, (0x1 << 16)
	li xp, RCC_MCO_SYSCLK | RCC_USB_RRE_96M | RCC_PLL_MUL_96M | RCC_PLL_SRC_HSE
	sw xp, RCC_CFGR0(wp)

	lw xp, RCC_CTLR(wp)
	.equ RCC_PLL1ON, (1 << 24)
	li yp, RCC_PLL1ON
	or xp, xp, yp
	sw xp, RCC_CTLR(wp)

	.equ RCC_PLL1RDY, (1 << 25)
	li yp, RCC_PLL1RDY
1:
	nop
	nop
	nop
	nop
	lw xp, RCC_CTLR(wp)
	and xp, xp, yp
	beqz xp, 1b

	.equ RCC_SW_PLL, (0x2 << 0)
	lw xp, RCC_CFGR0(wp)
	ori xp, xp, RCC_SW_PLL
	sw xp, RCC_CFGR0(wp)

	.equ RCC_SWS_PLL, (0x2 << 2)
	li yp, RCC_SWS_PLL
1:
	lw xp, RCC_CFGR0(wp)
	and xp, xp, yp
	bne xp, yp, 1b

apb2_clk_init:
	.equ RCC_APB2PCENR, 0x18
	.equ RCC_APB2_ALLEN, -1
	li xp, RCC_APB2_ALLEN
	sw xp, RCC_APB2PCENR(wp)

apb1_clk_init:
	.equ RCC_APB1PCENR, 0x1C
	.equ RCC_APB1_ALLEN, -1
	li xp, RCC_APB1_ALLEN
	sw xp, RCC_APB1PCENR(wp)


	.equ PWR_BASE, 0x40007000
	.equ PWR_CTLR, 0x00
	.equ PWR_DBP, (1 << 8)
	li wp, PWR_BASE
	lw xp, PWR_CTLR(wp)
	ori xp, xp, PWR_DBP
	sw xp, PWR_CTLR(wp)

	li wp, RCC_BASE
	.equ RCC_BDCTLR, 0x20
	.equ RCC_RSTSCKR, 0x24

	lw xp, RCC_BDCTLR(wp)
	.equ RCC_LSERDY, (1 << 1)
	.equ RCC_LSEON, (1 << 0)
	andi yp, xp, RCC_LSERDY
	bnez yp, lse_ok

	ori xp, xp, RCC_LSEON
	sw xp, RCC_BDCTLR(wp)

1:
	lw xp, RCC_BDCTLR(wp)
	andi xp, xp, RCC_LSERDY
	beqz xp, 1b

lse_ok:

	.equ RCC_RTCEN, (1 << 15)
	lw xp, RCC_BDCTLR(wp)
	li yp, RCC_RTCEN
	and yp, xp, yp
	bnez yp, rtc_ok

	.equ RCC_RTC_SEL_LSE, (1 << 8)
	li yp, RCC_RTC_SEL_LSE
	or xp, xp, yp
	sw xp, RCC_BDCTLR(wp)

	li yp, RCC_RTCEN
	or xp, xp, yp
	sw xp, RCC_BDCTLR(wp)
rtc_ok:
	.equ RCC_LSION, (1 << 0)
	.equ RCC_LSIRDY, (1 << 1)
	lw xp, RCC_RSTSCKR(wp)
	ori xp, xp, RCC_LSION
	sw xp, RCC_RSTSCKR(wp)

1:
	lw xp, RCC_RSTSCKR(wp)
	andi xp, xp, RCC_LSIRDY
	beqz xp, 1b


	# now:
	# SYSCLK: 96Mhz
	# PLLCLK: 96Mhz
	# HCLK:   96Mhz
	# PB1CLK: 96Mhz
	# PB2CLK: 96Mhz
	# LSE:    32Khz
	# LSI:    40Khz

	ret

	.equ IWDG_BASE, 0x40003000
	.equ IWDG_CTLR, 0x0
	.equ IWDG_PSCR, 0x4
	.equ IWDG_RLDR, 0x8
	.equ IWDG_STATR, 0xC
	defword iwdg, "iwdg", e_doconst, 0
	.word IWDG_BASE

	defword iwdg_ctlr, "iwdg-ctlr", e_doconst, 0
	.word IWDG_CTLR

	defword iwdg_pscr, "iwdg-pscr", e_doconst, 0
	.word IWDG_PSCR

	defword iwdg_rldr, "iwdg-rldr", e_doconst, 0
	.word IWDG_RLDR

	defword iwdg_statr, "iwdg-statr", e_doconst, 0
	.word IWDG_STATR

	defword wstore, "w!", e_wstore, 0
e_wstore:
	dpop wp
	dpop xp
	sh xp, 0(wp)
	next

	defword wload, "w@", e_wload, 0
e_wload:
	dpop wp
	lhu xp, 0(wp)
	dpush xp
	next

	defword iwdg_ctlr_store, "iwdg-ctlr!", e_call, 0
	.word w_iwdg
	.word w_iwdg_ctlr
	.word w_plus
	.word w_wstore
	.word w_exit

	defword iwdg_key_unlock, "iwdg-key-unlock", e_doconst, 0
	.word 0x5555

	defword iwdg_key_feed, "iwdg-key-feed", e_doconst, 0
	.word 0xAAAA

	defword iwdg_key_start, "iwdg-key-start", e_doconst, 0
	.word 0xCCCC

	defword iwdg_wait, "iwdg-wait", e_call, 0
1:
	.word w_iwdg
	.word w_iwdg_statr
	.word w_plus
	.word w_wload
	.word w_0x3
	.word w_logand
	.word w_eqz
	.word w_0branch
	.word 1b
	.word w_exit

	defword iwdg_start, "iwdg-start", e_call, 0
	.word w_iwdg_key_unlock
	.word w_iwdg_ctlr_store

	.word w_iwdg_wait
	.word w_0x7
	.word w_iwdg
	.word w_iwdg_pscr
	.word w_plus
	.word w_wstore

	.word w_iwdg_wait
	.word w_lit
	.word 0xFFF
	.word w_iwdg
	.word w_iwdg_rldr
	.word w_plus
	.word w_wstore

	.word w_iwdg_key_start
	.word w_iwdg_ctlr_store
	.word w_exit

	defword iwdg_feed, "iwdg-feed", e_call, 0
	.word w_iwdg_key_feed
	.word w_iwdg_ctlr_store
	.word w_exit

	

	defword interpret, "interpret", e_call, 0
	.word w_token
	.word w_tib
	.word w_toin_load
	.word w_find
	.word w_dup
	.word w_0branch
	.word interpret_nofound
	.word w_dup
	.word w_xtisimm
	.word w_0branch
	.word 1f
interpret_execute:
	.word w_execute
	.word w_psp_chk
	.word w_exit
1:
	.word w_state
	.word w_load
	.word w_0branch
	.word interpret_execute
	.word w_comma
	.word w_exit

interpret_nofound:
	.word w_drop
	.word w_tib
	.word w_toin_load
	.word w_isnumber
	.word w_invert
	.word w_0branch
	.word interpret_number
	.word w_tib
	.word w_toin_load
	.word w_type
	.word w_space
	.word w_lit
	.word msg_notfound
	.word w_lit
	.word msg_len_notfound
	.word w_type
	.word w_cr
	.word w_state
	.word w_off
	.word w_psp_rst
	.word w_exit

interpret_number:
	.word w_tib
	.word w_toin_load
	.word w_number
	.word w_state
	.word w_load
	.word w_0branch
	.word 1f
	.word w_lit
	.word name_lit
	.word w_lit
	.word nlen_lit
	.word w_find
	.word w_comma
	.word w_comma
1:
	.word w_exit

	.section .rodata
msg_notfound:
	.ascii "not found"
	.set msg_len_notfound, . - msg_notfound
	.section .text

	.p2align 2, 0xFF
boot_human:
	.macro display sym, msg
		.section .text
		.word w_lit
		.word str_\sym
		.word w_early_puts
		.section .rodata
	str_\sym:
		.asciz "\msg"
		.section .text
	.endm

	display test_next, "next."
	.word w_next

	display test_noop, "noop."
	.word w_noop
	.word w_dzchk


	display test_lit, "lit."
	.word w_lit
	.word 0
	.word w_lit
	.word 0
	.word w_nepanic
	.word w_dzchk

	display test_branch, "branch."
	.word w_lit
	.word 0
	.word w_lit
	.word 1
	.word w_lit
	.word 2

	.word w_branch
	.word 1f
	.word w_panic
2:
	.word w_drop
	.word w_branch
	.word 3f
	.word w_panic
1:
	.word w_drop
	.word w_branch
	.word 2b
	.word w_panic
3:
	.word w_drop
	.word w_dzchk

	display test_0branch, "0branch."
	.word w_lit
	.word 0
	.word w_lit
	.word 0
	.word w_lit
	.word 0

	.word w_0branch
	.word 1f
2:
	.word w_0branch
	.word 3f
1:
	.word w_0branch
	.word 2b
3:
	.word w_dzchk

	.word w_lit
	.word 1
	.word w_0branch
	.word panic
	.word w_dzchk

	display test_2lit, "2lit."
	.word w_2lit
	.word 1
	.word 1
	.word w_nepanic
	.word w_dzchk

	display test_equ, "=."
	.word w_2lit
	.word 2
	.word 2
	.word w_equ
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_2lit
	.word 3
	.word 2
	.word w_equ
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_eqz, "0=."
	.word w_0x0
	.word w_eqz
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_eqz
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_load, "@."
	.word w_lit
1:
	.word 0x01234567
	.word w_lit
	.word 1b
	.word w_load
	.word w_nepanic
	.word w_dzchk
	.word w_lit
1:
	.word 0x89ABCDEF
	.word w_lit
	.word 1b
	.word w_load
	.word w_nepanic
	.word w_dzchk

	display test_dup, "dup."
	.word w_lit
	.word 0x12345678
	.word w_dup
	.word w_nepanic
	.word w_dzchk

	display test_store, "!."
	.word w_lit
	.word 0x01234567
	.word w_dup
	.word w_here
	.word w_store
	.word w_here
	.word w_load
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 0x89ABCDEF
	.word w_dup
	.word w_here
	.word w_store
	.word w_here
	.word w_load
	.word w_nepanic
	.word w_dzchk

	display test_plus, "+."
	.word w_0x1
	.word w_0x1
	.word w_plus
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_plus
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_plus
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_lit
	.word -1
	.word w_plus
	.word w_lit
	.word -1
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word -1
	.word w_lit
	.word -1
	.word w_plus
	.word w_lit
	.word -2
	.word w_nepanic
	.word w_dzchk

	display test_logand, "&."
	.word w_0x0
	.word w_0x0
	.word w_logand
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_logand
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_logand
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x1
	.word w_logand
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display logxor, "^."
	.word w_0x0
	.word w_0x0
	.word w_logxor
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_logxor
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_logxor
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x1
	.word w_logxor
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_invert, "invert."
	.word w_true
	.word w_invert
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_false
	.word w_invert
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_invert
	.word w_lit
	.word -2
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word -2
	.word w_invert
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_ne, "<>."
	.word w_0x0
	.word w_0x0
	.word w_ne
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_ne
	.word w_true
	.word w_nepanic
	.word w_dzchk

	display test_nez, "0<>."
	.word w_0x0
	.word w_nez
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_nez
	.word w_true
	.word w_nepanic
	.word w_dzchk

	display test_pause, "pause."
	.word w_pause

	display test_swap, "swap."
	.word w_0x0
	.word w_0x1
	.word w_swap
	.word w_0x0
	.word w_nepanic
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_uart1_wait_tx, "uart1.uart-wait-tx.uart-data!"
	.word w_uart1
	.word w_dup
	.word w_uart_wait_tx
	.word w_lit
	.word '.'
	.word w_swap
	.word w_uart_data_store
	.word w_dzchk

	display test_uart1_putc, "uart1.uart-putc"
	.word w_lit
	.word '.'
	.word w_uart1
	.word w_uart_putc
	.word w_dzchk

	display test_uart1_emit, "uart1-emit"
	.word w_lit
	.word '.'
	.word w_uart1_emit
	.word w_dzchk

	display test_execute, "execute."
	.word w_lit
	.word w_true
	.word w_execute
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_0x5
	.word w_execute
	.word w_0x5
	.word w_nepanic
	.word w_dzchk

	display test_perform, "perform."
	.word w_lit
	.word 1f
	.word w_perform
1:
	.word w_0x9
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 1f
	.word w_perform
1:
	.word w_0xF
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 1f
	.word w_perform
1:
	.word w_true
	.word w_nepanic
	.word w_dzchk

	display test_emit, "emit"
	.word w_lit
	.word '.'
	.word w_emit
	.word w_dzchk

	display test_cload, "c@."
	.word w_lit
1:
	.word 'A'
	.word w_lit
	.word 1b
	.word w_cload
	.word w_nepanic
	.word w_dzchk

	display test_2drop, "2drop."
	.word w_0x1
	.word w_0x4
	.word w_2drop
	.word w_dzchk

	display test_1plus, "1+."
	.word w_0x1
	.word w_1plus
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	display test_minus, "-."
	.word w_0x1
	.word w_0x0
	.word w_minus
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x1
	.word w_minus
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_minus
	.word w_lit
	.word -1
	.word w_nepanic
	.word w_dzchk

	display test_1minus, "1-."
	.word w_0x1
	.word w_1minus
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_1minus
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_1minus
	.word w_lit
	.word -1
	.word w_nepanic
	.word w_dzchk

	.section .rodata
str_type_dot:
	.ascii "type."
	.section .text
	.word w_lit
	.word str_type_dot
	.word w_lit
	.word 5
	.word w_type
	.word w_dzchk

	display test_num2hex, "num2hex."
	.word w_0x0
	.word w_num2hex
	.word w_lit
	.word '0'
	.word w_nepanic
	.word w_dzchk

	.word w_0x9
	.word w_num2hex
	.word w_lit
	.word '9'
	.word w_nepanic
	.word w_dzchk

	.word w_0xA
	.word w_num2hex
	.word w_lit
	.word 'A'
	.word w_nepanic
	.word w_dzchk

	.word w_0xF
	.word w_num2hex
	.word w_lit
	.word 'F'
	.word w_nepanic
	.word w_dzchk

	.word w_0x10
	.word w_num2hex
	.word w_lit
	.word '0'
	.word w_nepanic
	.word w_dzchk

	display test_tor_fromr, ">r.r>."
	.word w_0x9
	.word w_tor
	.word w_dzchk
	.word w_fromr
	.word w_0x9
	.word w_nepanic
	.word w_dzchk

	display test_over, "over."
	.word w_0x1
	.word w_0x0
	.word w_over
	.word w_0x1
	.word w_nepanic
	.word w_0x0
	.word w_nepanic
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_2dup, "2dup."
	.word w_0x2
	.word w_0x3
	.word w_2dup
	.word w_0x3
	.word w_nepanic
	.word w_0x2
	.word w_nepanic
	.word w_0x3
	.word w_nepanic
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	display test_rshift, ">>."
	.word w_0x1
	.word w_0x0
	.word w_rshift
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_0x1
	.word w_rshift
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x4
	.word w_0x2
	.word w_rshift
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_4minus, "4-."
	.word w_0x4
	.word w_4minus
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_hex32, "hex32"
	.word w_lit
	.word '['
	.word w_emit
	.word w_lit
	.word 0x01234567
	.word w_hex32
	.word w_lit
	.word 0x89ABCDEF
	.word w_hex32
	.word w_lit
	.word ']'
	.word w_emit
	.word w_dzchk
	.word w_lit
	.word '.'
	.word w_emit

	display test_psp_psb_load, "psp@.psb@."
	.word w_psp_load
	.word w_psb_load
	.word w_nepanic
	.word w_dzchk

	display test_4div, "4/."
	.word w_0x4
	.word w_4div
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x8
	.word w_4div
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	display test_depth, "depth."
	.word w_depth
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_depth
	.word w_nepanic

	.word w_0x0
	.word w_0x2
	.word w_depth
	.word w_nepanic
	.word w_drop
	.word w_dzchk

	display test_4plus, "4+."
	.word w_0x0
	.word w_4plus
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	display test_dsdump, ".s"
	.word w_lit
	.word '['
	.word w_emit
	.word w_dsdump
	.word w_lit
	.word ']'
	.word w_emit

	.word w_lit
	.word '['
	.word w_emit
	.word w_0x0
	.word w_dsdump
	.word w_lit
	.word ']'
	.word w_emit

	.word w_lit
	.word '['
	.word w_emit
	.word w_0x1
	.word w_dsdump
	.word w_lit
	.word ']'
	.word w_emit

	.word w_lit
	.word '['
	.word w_emit
	.word w_drop
	.word w_dsdump
	.word w_lit
	.word ']'
	.word w_emit

	.word w_lit
	.word '['
	.word w_emit
	.word w_drop
	.word w_dsdump
	.word w_lit
	.word ']'
	.word w_emit

	.word w_lit
	.word '.'
	.word w_emit

	.word w_dzchk

/*
	display test_uart1_echo, "uart1.echo."
1:
	.word w_uart1
	.word w_uart_getc
	.word w_uart1
	.word w_uart_putc
	.word w_dzchk
	.word w_branch
	.word 1b
*/


	display test_toin_tib, "tib.>in."
	.word w_toin
	.word w_lit
	.word user_human + USER_OFFSET_TOIN
	.word w_nepanic
	.word w_dzchk

	.word w_0x3F
	.word w_toin_store
	.word w_toin_load
	.word w_0x3F
	.word w_nepanic
	.word w_dzchk

	.word w_tib_rst
	.word w_toin_load
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_lt, "<."
	.word w_0x0
	.word w_0x0
	.word w_lt
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_lt
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_lt
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word -1
	.word w_0x0
	.word w_lt
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_lit
	.word -1
	.word w_lt
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_gt, ">."
	.word w_0x0
	.word w_0x0
	.word w_gt
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_gt
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_gt
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word -1
	.word w_0x0
	.word w_gt
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_lit
	.word -1
	.word w_gt
	.word w_true
	.word w_nepanic
	.word w_dzchk

	display test_le, "<=."
	.word w_0x0
	.word w_0x0
	.word w_le
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_le
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_le
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_ge, ">=."
	.word w_0x0
	.word w_0x0
	.word w_ge
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_ge
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_ge
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_within, "within."
	.word w_0x0
	.word w_0x1
	.word w_0x2
	.word w_within
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x0
	.word w_0x2
	.word w_within
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_0x1
	.word w_0x2
	.word w_within
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_tib_chk, "tib-chk."
	.word w_toin_min
	.word w_1minus
	.word w_toin_store
	.word w_tib_chk
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_toin_max
	.word w_toin_store
	.word w_tib_chk
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_tib_rst
	.word w_tib_chk
	.word w_true
	.word w_nepanic
	.word w_dzchk

	display test_cstore, "c!."
	.word w_0x1F
	.word w_dup
	.word w_tib
	.word w_cstore
	.word w_tib
	.word w_cload
	.word w_nepanic
	.word w_dzchk

	.word w_0xFF
	.word w_dup
	.word w_tib
	.word w_cstore
	.word w_tib
	.word w_cload
	.word w_nepanic
	.word w_dzchk

	display test_tib_in_out, "tib-in.tib-out."
	.word w_0x20
	.word w_tib_in
	.word w_toin_load
	.word w_0x1
	.word w_nepanic
	.word w_tib_out
	.word w_0x20
	.word w_nepanic
	.word w_toin_load
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_tib_out
	.word w_drop
	.word w_toin_load
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_toin_max
	.word w_1plus
	.word w_toin_store
	.word w_0x0
	.word w_tib_in
	.word w_toin_load
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_nip, "nip."
	.word w_0x0
	.word w_0x1
	.word w_nip
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_logor, "|."
	.word w_0x0
	.word w_0x0
	.word w_logor
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_0x1
	.word w_logor
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_logor
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x1
	.word w_logor
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_rot, "rot."
	.word w_0x2
	.word w_0x1
	.word w_0x0
	.word w_rot
	.word w_0x2
	.word w_nepanic
	.word w_0x0
	.word w_nepanic
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_isspace, "space?."
	.word w_lit
	.word ' '
	.word w_isspace
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word '\t'
	.word w_isspace
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'A'
	.word w_isspace
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_isdel, "del?."
	.word w_lit
	.word '\b'
	.word w_isdel
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_0x7F
	.word w_isdel
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'N'
	.word w_isdel
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_isnewline, "newline?."
	.word w_lit
	.word '\n'
	.word w_isnewline
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word '\r'
	.word w_isnewline
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'B'
	.word w_isnewline
	.word w_false
	.word w_nepanic
	.word w_dzchk

/*
	display test_token, "token."
1:
	.word w_token
	.word w_lit
	.word '['
	.word w_emit
	.word w_tib
	.word w_toin_load
	.word w_type
	.word w_lit
	.word ']'
	.word w_emit
	.word w_dzchk
	.word w_branch
	.word 1b
*/

	display test_ishexhdr, "hexhdr?."
	.section .rodata
str_hex:
	.ascii "0x1"
str_badhex0:
	.ascii "0A0"
str_badhex1:
	.ascii "Fx0"
	.section .text
	.word w_lit
	.word str_hex
	.word w_0x2
	.word w_ishexhdr
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_hex
	.word w_0x3
	.word w_ishexhdr
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badhex0
	.word w_0x3
	.word w_ishexhdr
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badhex1
	.word w_0x3
	.word w_ishexhdr
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_2plus, "2+."
	.word w_0x0
	.word w_2plus
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	display test_2minus, "2-."
	.word w_0x2
	.word w_2minus
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_isxdigit, "xdigit?."
	.word w_lit
	.word '0'
	.word w_isxdigit
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word '9'
	.word w_isxdigit
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word '9' + 1
	.word w_isxdigit
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word '0' - 1
	.word w_isxdigit
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'A'
	.word w_isxdigit
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'F'
	.word w_isxdigit
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'F' + 1
	.word w_isxdigit
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'A' - 1
	.word w_isxdigit
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_isnumber, "number?."
	.section .rodata
str_num:
	.ascii "0x1"
str_num1:
	.ascii "0x123"
str_badnum0:
	.ascii "0A0"
str_badnum1:
	.ascii "Fx0"
str_badnum2:
	.ascii "0x0M"
str_badnum3:
	.ascii "0xM0"
	.section .text

	.word w_lit
	.word str_num
	.word w_0x2
	.word w_isnumber
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum0
	.word w_0x3
	.word w_isnumber
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum1
	.word w_0x3
	.word w_isnumber
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_num
	.word w_0x3
	.word w_isnumber
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_num1
	.word w_0x5
	.word w_isnumber
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum2
	.word w_0x4
	.word w_isnumber
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum3
	.word w_0x4
	.word w_isnumber
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_lshift, "<<."
	.word w_0x1
	.word w_0x1
	.word w_lshift
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_lshift
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x2
	.word w_lshift
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	display test_4multi, "4*."
	.word w_0x1
	.word w_4multi
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_4multi
	.word w_0x8
	.word w_nepanic
	.word w_dzchk

	display test_2swap, "2swap."
	.word w_0x0
	.word w_0x1
	.word w_0x2
	.word w_0x3
	.word w_2swap
	.word w_0x1
	.word w_nepanic
	.word w_0x0
	.word w_nepanic
	.word w_0x3
	.word w_nepanic
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	display test_hex2num, "hex2num."
	.word w_lit
	.word '0'
	.word w_hex2num
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word '9'
	.word w_hex2num
	.word w_0x9
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'A'
	.word w_hex2num
	.word w_0xA
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 'F'
	.word w_hex2num
	.word w_0xF
	.word w_nepanic
	.word w_dzchk

	display test_number, "number."
	.word w_lit
	.word str_num
	.word w_0x3
	.word w_number
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_num1
	.word w_0x5
	.word w_number
	.word w_lit
	.word 0x123
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum0
	.word w_0x3
	.word w_number
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum1
	.word w_0x3
	.word w_number
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum2
	.word w_0x4
	.word w_number
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_badnum3
	.word w_0x4
	.word w_number
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.section .rodata
str_num2:
	.ascii "0x01234567"
str_num3:
	.ascii "0x89ABCDEF"
	.section .text

	.word w_lit
	.word str_num2
	.word w_0xA
	.word w_number
	.word w_lit
	.word 0x01234567
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word str_num3
	.word w_0xA
	.word w_number
	.word w_lit
	.word 0x89ABCDEF
	.word w_nepanic
	.word w_dzchk

	display test_min, "min."
	.word w_0x0
	.word w_0x1
	.word w_min
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_min
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_max, "max."
	.word w_0x0
	.word w_0x1
	.word w_max
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x0
	.word w_max
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_compare, "compare."
	.word w_0x0
	.word w_dup
	.word w_2dup
	.word w_compare
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

1:
	.word w_lit
	.word 1b
	.word w_0x4
1:
	.word w_lit
	.word 1b
	.word w_0x4
	.word w_compare
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

1:
	.word w_lit
	.word 1b
	.word w_0x8
1:
	.word w_lit
	.word 1b
	.word w_0x8
	.word w_compare
	.word w_nez
	.word w_true
	.word w_nepanic
	.word w_dzchk

	display test_xtentr, "xtentr."
	.word w_lit
	.word w_next
	.word w_xtentr
	.word w_lit
	.word e_next
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_xtentr
	.word w_xtentr
	.word w_lit
	.word e_call
	.word w_nepanic
	.word w_dzchk

	display test_xtlink, "xtlink."
	.word w_lit
	.word w_next
	.word w_xtlink
	.word w_lit
	.word prev_next
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_xtlink
	.word w_xtlink
	.word w_lit
	.word prev_xtlink
	.word w_nepanic
	.word w_dzchk

	display test_xtattr, "xtattr."
	.word w_lit
	.word w_next
	.word w_xtattr
	.word w_lit
	.word attr_next
	.word w_load
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_xtattr
	.word w_xtattr
	.word w_lit
	.word attr_xtattr
	.word w_load
	.word w_nepanic
	.word w_dzchk

	display test_xtnlen, "xtnlen."
	.word w_lit
	.word w_next
	.word w_xtnlen
	.word w_lit
	.word nlen_next
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_xtnlen
	.word w_xtnlen
	.word w_lit
	.word nlen_xtnlen
	.word w_nepanic
	.word w_dzchk

	display test_aligned, "aligned."
	.word w_0x0
	.word w_aligned
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_aligned
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_aligned
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	.word w_0x3
	.word w_aligned
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	.word w_0x4
	.word w_aligned
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	.word w_0x5
	.word w_aligned
	.word w_0x8
	.word w_nepanic
	.word w_dzchk

	display test_xtname, "xtname."
	.word w_lit
	.word w_next
	.word w_xtname
	.word w_lit
	.word name_next
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_key
	.word w_xtname
	.word w_lit
	.word name_key
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_xtname
	.word w_xtname
	.word w_lit
	.word name_xtname
	.word w_nepanic
	.word w_dzchk

	display test_latest, "latest."
	.word w_latest
	.word w_lit
	.word lastword
	.word w_nepanic
	.word w_dzchk

	display test_words, "words"
	.word w_lit
	.word '['
	.word w_emit
	.word w_words
	.word w_lit
	.word ']'
	.word w_emit
	.word w_lit
	.word '.'
	.word w_emit
	.word w_dzchk

	display test_2over, "2over."
	.word w_0x1
	.word w_0x2
	.word w_0x3
	.word w_0x4
	.word w_2over
	.word w_0x2
	.word w_nepanic
	.word w_0x1
	.word w_nepanic
	.word w_0x4
	.word w_nepanic
	.word w_0x3
	.word w_nepanic
	.word w_0x2
	.word w_nepanic
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	display test_find, "find."
	.word w_0x0
	.word w_0x0
	.word w_find
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word name_next
	.word w_lit
	.word nlen_next
	.word w_find
	.word w_lit
	.word w_next
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word name_lit
	.word w_lit
	.word nlen_lit
	.word w_find
	.word w_lit
	.word w_lit
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word name_find
	.word w_lit
	.word nlen_find
	.word w_find
	.word w_lit
	.word w_find
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_lit
	.word 0xFF
	.word w_find
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x0
	.word w_lit
	.word 0x4
	.word w_find
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_on_off, "on.off."
	.word w_here
	.word w_on
	.word w_here
	.word w_load
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_here
	.word w_off
	.word w_here
	.word w_load
	.word w_false
	.word w_nepanic
	.word w_dzchk
	
	display test_xtisimm, "xtimm?."
	.word w_lit
	.word w_compoff
	.word w_xtisimm
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word w_compon
	.word w_xtisimm
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_compon_compoff, "[.]."
	.word w_compon
	.word w_state
	.word w_load
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_compoff
	.word w_state
	.word w_load
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_comma, ",."
	.word w_here
	.word w_lit
	.word 0x01234567
	.word w_comma
	.word w_dup
	.word w_load
	.word w_lit
	.word 0x01234567
	.word w_nepanic
	.word w_here
	.word w_swap
	.word w_minus
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	display test_move, "move."
	.word w_0x0
	.word w_here
	.word w_0x2
	.word w_move

	.word w_0x0
	.word w_0x8
	.word w_here
	.word w_over
	.word w_compare
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_defword, "defword."
	.word w_lit
	.word name_noop
	.word w_lit
	.word nlen_noop
	.word w_defword
	.word w_dzchk

	.word w_latest
	.word w_xtname
	.word w_latest
	.word w_xtnlen
	.word w_lit
	.word name_noop
	.word w_lit
	.word nlen_noop
	.word w_compare
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_latest
	.word w_xtlink
	.word w_lit
	.word lastword
	.word w_nepanic
	.word w_dzchk

	.word w_latest
	.word w_xtentr
	.word w_lit
	.word e_call
	.word w_nepanic
	.word w_dzchk

	.word w_latest
	.word w_xtattr
	.word w_lit
	.word nlen_noop
	.word w_attr_flag_hide
	.word w_logor
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word name_exit
	.word w_lit
	.word nlen_exit
	.word w_find
	.word w_comma
	.word w_dzchk

	.word w_lit
	.word name_noop
	.word w_lit
	.word nlen_noop
	.word w_find
	.word w_latest
	.word w_equ
	.word w_false
	.word w_nepanic
	.word w_dzchk

	display test_bitclr, "bitclr."
	.word w_lit
	.word 0x81
	.word w_0x80
	.word w_bitclr
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 0x81
	.word w_0x1
	.word w_bitclr
	.word w_0x80
	.word w_nepanic
	.word w_dzchk

	display test_hide_unhide_xtishide, "hide.unhide.xthide?."
	.word w_latest
	.word w_xtishide
	.word w_true
	.word w_nepanic
	.word w_dzchk

	.word w_latest
	.word w_unhide
	.word w_dzchk

	.word w_latest
	.word w_xtishide
	.word w_false
	.word w_nepanic
	.word w_dzchk

	.word w_latest
	.word w_hide
	.word w_dzchk

	.word w_latest
	.word w_xtishide
	.word w_true
	.word w_nepanic
	.word w_dzchk

	display test_modulus, "%."
	.word w_0x1
	.word w_0x2
	.word w_modulus
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_0x1
	.word w_modulus
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_0x1
	.word w_modulus
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	.word w_0x7
	.word w_0x5
	.word w_modulus
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	display test_division, "/."
	.word w_0x1
	.word w_0x1
	.word w_division
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x2
	.word w_0x1
	.word w_division
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x2
	.word w_division
	.word w_0x0
	.word w_nepanic
	.word w_dzchk

	display test_32div, "32/."
	.word w_0x20
	.word w_32div
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x40
	.word w_32div
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	display test_2multi, "2*."
	.word w_0x2
	.word w_2multi
	.word w_0x4
	.word w_nepanic
	.word w_dzchk

	display test_multi, "*."
	.word w_0x1
	.word w_0x1
	.word w_multi
	.word w_0x1
	.word w_nepanic
	.word w_dzchk

	.word w_0x1
	.word w_0x2
	.word w_multi
	.word w_0x2
	.word w_nepanic
	.word w_dzchk

	.word w_0x4
	.word w_0x2
	.word w_multi
	.word w_0x8
	.word w_nepanic
	.word w_dzchk

	display test_wstore_wload, "w!.w@."
	.word w_lit
1:
	.word 0x789A
	.word w_lit
	.word 1b
	.word w_wload
	.word w_nepanic
	.word w_dzchk

	.word w_lit
	.word 0x789A
	.word w_dup
	.word w_here
	.word w_wstore
	.word w_here
	.word w_wload
	.word w_nepanic
	.word w_dzchk


	#=================================================
	display system_init, "system.init."

	display uart1, "uart1."
	.word w_irq_no_uart1
	.word w_pfic_irq_en

	.word w_uart1_rxfifo_rst
	.word w_uart1
	.word w_uart_dri_en

	.word w_lit
	.word w_uart1_rxfifo_key
	.word w_user_addr
	.word w_user_offset_key
	.word w_plus
	.word w_store

	.word w_lit
	.word w_uart1_rxfifo_keyava
	.word w_user_addr
	.word w_user_offset_keyava
	.word w_plus
	.word w_store

	display iwdg, "iwdg."
	.word w_iwdg_start


/*
	display test_echo, "echo."
1:
	.word w_key
	.word w_emit
	.word w_dzchk
	.word w_branch
	.word 1b
*/

	display enter_interpret, "interpret."
1:
	.word w_interpret
	.word w_branch
	.word 1b

	display test_halt, "halt."
	.word w_halt
panic:
	.word w_panic

boot_root:
1:
	.word w_pause
	.word w_iwdg_feed
	.word w_branch
	.word 1b

boot_irq:
	.word w_mcause_load
	.word w_0xFF
	.word w_logand
	.word w_4multi
	.word w_forth_irq_base
	.word w_plus
	.word w_load
	.word w_dup
	.word w_eqz
	.word w_0branch
	.word 1f
	.section .rodata
str_unhandle_irq:

	.ascii "\n\r!!! UNHANDLE IRQ !!!"
	.set len_str_unhandle_irq, . - str_unhandle_irq
	.section .text
	.word w_lit
	.word str_unhandle_irq
	.word w_lit
	.word len_str_unhandle_irq
	.word w_type
	.word w_regs_dump
2:
	.word w_branch
	.word 2b
1:
	.word w_execute
	.word w_irq_exit

forth:
	.section .rodata
str_forth:
	.asciz "FORTH."

	.section .text

	la wp, str_forth
	call early_puts

	la up, user_irq
	sw up, USER_OFFSET_NEXT(up)
	la ip, boot_irq
	la rsp, rstk_irq
	la psb, dstk_irq
	mv psp, psb
	call user_save

	la wp, w_early_txc
	sw wp, USER_OFFSET_EMIT(up)

	la up, user_root
	la wp, user_human
	sw wp, USER_OFFSET_NEXT(up)
	la ip, boot_root
	la rsp, rstk_root
	la psb, dstk_root
	mv psp, psb
	call user_save

	la up, user_human
	la wp, user_root
	sw wp, USER_OFFSET_NEXT(up)
	la ip, boot_human
	la rsp, rstk_human
	la psb, dstk_human
	mv psp, psb
	call user_save

	la wp, w_uart1_emit
	sw wp, USER_OFFSET_EMIT(up)

	la wp, w_uart1_key
	sw wp, USER_OFFSET_KEY(up)

	la wp, w_uart1_keyava
	sw wp, USER_OFFSET_KEYAVA(up)

	la wp, tib_human
	sw wp, USER_OFFSET_TIB(up)

	sw zero, USER_OFFSET_TOIN(up)

	next

	.section .data
irq_entry:
	.word   _start
	.word   0
	.word   NMI_Handler                /* NMI */
	.word   HardFault_Handler          /* Hard Fault */
	.word   0
	.word   Ecall_M_Mode_Handler       /* Ecall M Mode */
	.word   0
	.word   0
	.word   Ecall_U_Mode_Handler       /* Ecall U Mode */
	.word   Break_Point_Handler        /* Break Point */
	.word   0
	.word   0
	.word   SysTick_Handler            /* SysTick */
	.word   0
	.word   SW_Handler                 /* SW */
	.word   0
	/* External Interrupts */
	.word   WWDG_IRQHandler            /* Window Watchdog */
	.word   PVD_IRQHandler             /* PVD through EXTI Line detect */
	.word   TAMPER_IRQHandler          /* TAMPER */
	.word   RTC_IRQHandler             /* RTC */
	.word   FLASH_IRQHandler           /* Flash */
	.word   RCC_IRQHandler             /* RCC */
	.word   EXTI0_IRQHandler           /* EXTI Line 0 */
	.word   EXTI1_IRQHandler           /* EXTI Line 1 */
	.word   EXTI2_IRQHandler           /* EXTI Line 2 */
	.word   EXTI3_IRQHandler           /* EXTI Line 3 */
	.word   EXTI4_IRQHandler           /* EXTI Line 4 */
	.word   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
	.word   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
	.word   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
	.word   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
	.word   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
	.word   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
	.word   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
	.word   ADC1_2_IRQHandler          /* ADC1_2 */
	.word   USB_HP_CAN1_TX_IRQHandler  /* USB HP and CAN1 TX */
	.word   USB_LP_CAN1_RX0_IRQHandler /* USB LP and CAN1RX0 */
	.word   CAN1_RX1_IRQHandler        /* CAN1 RX1 */
	.word   CAN1_SCE_IRQHandler        /* CAN1 SCE */
	.word   EXTI9_5_IRQHandler         /* EXTI Line 9..5 */
	.word   TIM1_BRK_IRQHandler        /* TIM1 Break */
	.word   TIM1_UP_IRQHandler         /* TIM1 Update */
	.word   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
	.word   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
	.word   TIM2_IRQHandler            /* TIM2 */
	.word   TIM3_IRQHandler            /* TIM3 */
	.word   TIM4_IRQHandler            /* TIM4 */
	.word   I2C1_EV_IRQHandler         /* I2C1 Event */
	.word   I2C1_ER_IRQHandler         /* I2C1 Error */
	.word   I2C2_EV_IRQHandler         /* I2C2 Event */
	.word   I2C2_ER_IRQHandler         /* I2C2 Error */
	.word   SPI1_IRQHandler            /* SPI1 */
	.word   SPI2_IRQHandler            /* SPI2 */
	.word   USART1_IRQHandler          /* USART1 */
	.word   USART2_IRQHandler          /* USART2 */
	.word   USART3_IRQHandler          /* USART3 */
	.word   EXTI15_10_IRQHandler       /* EXTI Line 15..10 */
	.word   RTCAlarm_IRQHandler        /* RTC Alarm through EXTI Line */
	.word   USBWakeUp_IRQHandler       /* USB Wakeup from suspend */
	.word   TIM8_BRK_IRQHandler        /* TIM8 Break */
	.word   TIM8_UP_IRQHandler         /* TIM8 Update */
	.word   TIM8_TRG_COM_IRQHandler    /* TIM8 Trigger and Commutation */
	.word   TIM8_CC_IRQHandler         /* TIM8 Capture Compare */
	.word   RNG_IRQHandler             /* RNG */
	.word   0
	.word   SDIO_IRQHandler            /* SDIO */
	.word   TIM5_IRQHandler            /* TIM5 */
	.word   SPI3_IRQHandler            /* SPI3 */
	.word   UART4_IRQHandler           /* UART4 */
	.word   UART5_IRQHandler           /* UART5 */
	.word   TIM6_IRQHandler            /* TIM6 */
	.word   TIM7_IRQHandler            /* TIM7 */
	.word   DMA2_Channel1_IRQHandler   /* DMA2 Channel 1 */
	.word   DMA2_Channel2_IRQHandler   /* DMA2 Channel 2 */
	.word   DMA2_Channel3_IRQHandler   /* DMA2 Channel 3 */
	.word   DMA2_Channel4_IRQHandler   /* DMA2 Channel 4 */
	.word   DMA2_Channel5_IRQHandler   /* DMA2 Channel 5 */
	.word   ETH_IRQHandler             /* ETH */
	.word   ETH_WKUP_IRQHandler        /* ETH WakeUp */
	.word   CAN2_TX_IRQHandler         /* CAN2 TX */
	.word   CAN2_RX0_IRQHandler        /* CAN2 RX0 */
	.word   CAN2_RX1_IRQHandler        /* CAN2 RX1 */
	.word   CAN2_SCE_IRQHandler        /* CAN2 SCE */
	.word   USBFS_IRQHandler           /* USBFS */
	.word   USBHSWakeup_IRQHandler     /* USBHS Wakeup */
	.word   USBHS_IRQHandler           /* USBHS */
	.word   DVP_IRQHandler             /* DVP */
	.word   UART6_IRQHandler           /* UART6 */
	.word   UART7_IRQHandler           /* UART7 */
	.word   UART8_IRQHandler           /* UART8 */
	.word   TIM9_BRK_IRQHandler        /* TIM9 Break */
	.word   TIM9_UP_IRQHandler         /* TIM9 Update */
	.word   TIM9_TRG_COM_IRQHandler    /* TIM9 Trigger and Commutation */
	.word   TIM9_CC_IRQHandler         /* TIM9 Capture Compare */
	.word   TIM10_BRK_IRQHandler       /* TIM10 Break */
	.word   TIM10_UP_IRQHandler        /* TIM10 Update */
	.word   TIM10_TRG_COM_IRQHandler   /* TIM10 Trigger and Commutation */
	.word   TIM10_CC_IRQHandler        /* TIM10 Capture Compare */
	.word   DMA2_Channel6_IRQHandler   /* DMA2 Channel 6 */
	.word   DMA2_Channel7_IRQHandler   /* DMA2 Channel 7 */
	.word   DMA2_Channel8_IRQHandler   /* DMA2 Channel 8 */
	.word   DMA2_Channel9_IRQHandler   /* DMA2 Channel 9 */
	.word   DMA2_Channel10_IRQHandler  /* DMA2 Channel 10 */
	.word   DMA2_Channel11_IRQHandler  /* DMA2 Channel 11 */

	.section .text
NMI_Handler:
HardFault_Handler:
Ecall_M_Mode_Handler:
Ecall_U_Mode_Handler:
Break_Point_Handler:
SysTick_Handler:
SW_Handler:
WWDG_IRQHandler:
PVD_IRQHandler:
TAMPER_IRQHandler:
RTC_IRQHandler:
FLASH_IRQHandler:
RCC_IRQHandler:
EXTI0_IRQHandler:
EXTI1_IRQHandler:
EXTI2_IRQHandler:
EXTI3_IRQHandler:
EXTI4_IRQHandler:
DMA1_Channel1_IRQHandler:
DMA1_Channel2_IRQHandler:
DMA1_Channel3_IRQHandler:
DMA1_Channel4_IRQHandler:
DMA1_Channel5_IRQHandler:
DMA1_Channel6_IRQHandler:
DMA1_Channel7_IRQHandler:
ADC1_2_IRQHandler:
USB_HP_CAN1_TX_IRQHandler:
USB_LP_CAN1_RX0_IRQHandler:
CAN1_RX1_IRQHandler:
CAN1_SCE_IRQHandler:
EXTI9_5_IRQHandler:
TIM1_BRK_IRQHandler:
TIM1_UP_IRQHandler:
TIM1_TRG_COM_IRQHandler:
TIM1_CC_IRQHandler:
TIM2_IRQHandler:
TIM3_IRQHandler:
TIM4_IRQHandler:
I2C1_EV_IRQHandler:
I2C1_ER_IRQHandler:
I2C2_EV_IRQHandler:
I2C2_ER_IRQHandler:
SPI1_IRQHandler:
SPI2_IRQHandler:
USART2_IRQHandler:
USART3_IRQHandler:
EXTI15_10_IRQHandler:
RTCAlarm_IRQHandler:
USBWakeUp_IRQHandler:
TIM8_BRK_IRQHandler:
TIM8_UP_IRQHandler:
TIM8_TRG_COM_IRQHandler:
TIM8_CC_IRQHandler:
RNG_IRQHandler:
SDIO_IRQHandler:
TIM5_IRQHandler:
SPI3_IRQHandler:
UART4_IRQHandler:
UART5_IRQHandler:
TIM6_IRQHandler:
TIM7_IRQHandler:
DMA2_Channel1_IRQHandler:
DMA2_Channel2_IRQHandler:
DMA2_Channel3_IRQHandler:
DMA2_Channel4_IRQHandler:
DMA2_Channel5_IRQHandler:
ETH_IRQHandler:
ETH_WKUP_IRQHandler:
CAN2_TX_IRQHandler:
CAN2_RX0_IRQHandler:
CAN2_RX1_IRQHandler:
CAN2_SCE_IRQHandler:
USBFS_IRQHandler:
USBHSWakeup_IRQHandler:
USBHS_IRQHandler:
DVP_IRQHandler:
UART6_IRQHandler:
UART7_IRQHandler:
UART8_IRQHandler:
TIM9_BRK_IRQHandler:
TIM9_UP_IRQHandler:
TIM9_TRG_COM_IRQHandler:
TIM9_CC_IRQHandler:
TIM10_BRK_IRQHandler:
TIM10_UP_IRQHandler:
TIM10_TRG_COM_IRQHandler:
TIM10_CC_IRQHandler:
DMA2_Channel6_IRQHandler:
DMA2_Channel7_IRQHandler:
DMA2_Channel8_IRQHandler:
DMA2_Channel9_IRQHandler:
DMA2_Channel10_IRQHandler:
DMA2_Channel11_IRQHandler:
	j forth_irq_handler

forth_irq_handler:
	addi sp, sp, -REGS_SIZE
	sw zero, REG_OFFSET_ZERO(sp)
	sw ra, REG_OFFSET_RA(sp)
	sw sp, REG_OFFSET_SP(sp) # not need save sp
	sw gp, REG_OFFSET_GP(sp)
	sw tp, REG_OFFSET_TP(sp)
	sw t0, REG_OFFSET_T0(sp)
	sw t1, REG_OFFSET_T1(sp)
	sw t2, REG_OFFSET_T2(sp)
	sw s0, REG_OFFSET_S0(sp)
	sw s1, REG_OFFSET_S1(sp)
	sw a0, REG_OFFSET_A0(sp)
	sw a1, REG_OFFSET_A1(sp)
	sw a2, REG_OFFSET_A2(sp)
	sw a3, REG_OFFSET_A3(sp)
	sw a4, REG_OFFSET_A4(sp)
	sw a5, REG_OFFSET_A5(sp)
	sw a6, REG_OFFSET_A6(sp)
	sw a7, REG_OFFSET_A7(sp)
	sw s2, REG_OFFSET_S2(sp)
	sw s3, REG_OFFSET_S3(sp)
	sw s4, REG_OFFSET_S4(sp)
	sw s5, REG_OFFSET_S5(sp)
	sw s6, REG_OFFSET_S6(sp)
	sw s7, REG_OFFSET_S7(sp)
	sw s8, REG_OFFSET_S8(sp)
	sw s9, REG_OFFSET_S9(sp)
	sw s10, REG_OFFSET_S10(sp)
	sw s11, REG_OFFSET_S11(sp)
	sw t3, REG_OFFSET_T3(sp)
	sw t4, REG_OFFSET_T4(sp)
	sw t5, REG_OFFSET_T5(sp)
	sw t6, REG_OFFSET_T6(sp)

	la wp, in_irq
	li xp, -1
	sw xp, 0(wp)

	la up, user_irq
	call user_load
	next

USART1_IRQHandler:
	li t0, UART1_BASE
	lw t1, UART_STATR(t0)
	andi t1, t1, UART_RXNE
	beqz t1, 1f
	lw t6, UART_DATAR(t0)

	# forth is slow
	# so we use machine code in uart irq

	# t6 is input data
	.macro fast_fifo_push fifo
	la t0, \fifo
	lw t1, FIFO_OFFSET_NUM(t0)
	lw t2, FIFO_OFFSET_HEAD(t0)
	lw t3, FIFO_OFFSET_MASK(t0)
	add t2, t2, t1
	and t2, t2, t3
	add t2, t2, t0
	sb t6, FIFO_OFFSET_BUF(t2)
	addi t1, t1, 1
	and t1, t1, t3
	sw t1, FIFO_OFFSET_NUM(t0)
	.endm

	fast_fifo_push uart1_rxfifo
1:
	mret

	.section .data
forth_irq_base:
	.fill 256, 4, 0
dp:
	.word dict
latest:
	.word lastword
uart1_rxfifo:
	.word 0xFF
	.word 0
	.word 0
	.fill 0xFF + 1, 1, 0

	.section .bss
in_irq:
	.fill 1, 4, 0
user_irq:
	.fill USRSIZE, 4, 0
rstk_irq:
	.fill STKSIZE, 4, 0
dstk_irq:
	.fill STKSIZE, 4, 0
user_root:
	.fill USRSIZE, 4, 0
rstk_root:
	.fill STKSIZE, 4, 0
dstk_root:
	.fill STKSIZE, 4, 0
user_human:
	.fill USRSIZE, 4, 0
rstk_human:
	.fill STKSIZE, 4, 0
dstk_human:
	.fill STKSIZE, 4, 0
tib_human:
	.fill TIBSIZE, 1, 0
	.p2align 2, 0x0
dict:
