#define wp a0
#define xp a1
#define yp a2
#define tos a3
#define ip s0
#define rp s1
#define st a4
#define ss a5
#define up tp

	.equ ADDRSIZE, 4
	.equ STKSIZE, 63
	.equ TASKSIZE, 32
	.equ TIBSIZE, 31

	.section .init
	.global _start
_start:
	.option norvc;
	j reset
_vector:
	.word   _start
	.word   0
	.word   NMI_handler                /* NMI */
	.word   HardFault_handler          /* Hard Fault */
	.word   0
	.word   Ecall_M_Mode_handler       /* Ecall M Mode */
	.word   0
	.word   0
	.word   Ecall_U_Mode_handler       /* Ecall U Mode */
	.word   Break_Point_handler        /* Break Point */
	.word   0
	.word   0
	.word   SysTick_handler            /* SysTick */
	.word   0
	.word   SW_handler                 /* SW */
	.word   0
	/* External Interrupts */
	.word   WWDG_irq_handler            /* Window Watchdog */
	.word   PVD_irq_handler             /* PVD through EXTI Line detect */
	.word   TAMPER_irq_handler          /* TAMPER */
	.word   RTC_irq_handler             /* RTC */
	.word   FLASH_irq_handler           /* Flash */
	.word   RCC_irq_handler             /* RCC */
	.word   EXTI0_irq_handler           /* EXTI Line 0 */
	.word   EXTI1_irq_handler           /* EXTI Line 1 */
	.word   EXTI2_irq_handler           /* EXTI Line 2 */
	.word   EXTI3_irq_handler           /* EXTI Line 3 */
	.word   EXTI4_irq_handler           /* EXTI Line 4 */
	.word   DMA1_Channel1_irq_handler   /* DMA1 Channel 1 */
	.word   DMA1_Channel2_irq_handler   /* DMA1 Channel 2 */
	.word   DMA1_Channel3_irq_handler   /* DMA1 Channel 3 */
	.word   DMA1_Channel4_irq_handler   /* DMA1 Channel 4 */
	.word   DMA1_Channel5_irq_handler   /* DMA1 Channel 5 */
	.word   DMA1_Channel6_irq_handler   /* DMA1 Channel 6 */
	.word   DMA1_Channel7_irq_handler   /* DMA1 Channel 7 */
	.word   ADC1_2_irq_handler          /* ADC1_2 */
	.word   USB_HP_CAN1_TX_irq_handler  /* USB HP and CAN1 TX */
	.word   USB_LP_CAN1_RX0_irq_handler /* USB LP and CAN1RX0 */
	.word   CAN1_RX1_irq_handler        /* CAN1 RX1 */
	.word   CAN1_SCE_irq_handler        /* CAN1 SCE */
	.word   EXTI9_5_irq_handler         /* EXTI Line 9..5 */
	.word   TIM1_BRK_irq_handler        /* TIM1 Break */
	.word   TIM1_UP_irq_handler         /* TIM1 Update */
	.word   TIM1_TRG_COM_irq_handler    /* TIM1 Trigger and Commutation */
	.word   TIM1_CC_irq_handler         /* TIM1 Capture Compare */
	.word   TIM2_irq_handler            /* TIM2 */
	.word   TIM3_irq_handler            /* TIM3 */
	.word   TIM4_irq_handler            /* TIM4 */
	.word   I2C1_EV_irq_handler         /* I2C1 Event */
	.word   I2C1_ER_irq_handler         /* I2C1 Error */
	.word   I2C2_EV_irq_handler         /* I2C2 Event */
	.word   I2C2_ER_irq_handler         /* I2C2 Error */
	.word   SPI1_irq_handler            /* SPI1 */
	.word   SPI2_irq_handler            /* SPI2 */
	.word   USART1_irq_handler          /* USART1 */
	.word   USART2_irq_handler          /* USART2 */
	.word   USART3_irq_handler          /* USART3 */
	.word   EXTI15_10_irq_handler       /* EXTI Line 15..10 */
	.word   RTCAlarm_irq_handler        /* RTC Alarm through EXTI Line */
	.word   USBWakeUp_irq_handler       /* USB Wakeup from suspend */
	.word   TIM8_BRK_irq_handler        /* TIM8 Break */
	.word   TIM8_UP_irq_handler         /* TIM8 Update */
	.word   TIM8_TRG_COM_irq_handler    /* TIM8 Trigger and Commutation */
	.word   TIM8_CC_irq_handler         /* TIM8 Capture Compare */
	.word   RNG_irq_handler             /* RNG */
	.word   0
	.word   SDIO_irq_handler            /* SDIO */
	.word   TIM5_irq_handler            /* TIM5 */
	.word   SPI3_irq_handler            /* SPI3 */
	.word   UART4_irq_handler           /* UART4 */
	.word   UART5_irq_handler           /* UART5 */
	.word   TIM6_irq_handler            /* TIM6 */
	.word   TIM7_irq_handler            /* TIM7 */
	.word   DMA2_Channel1_irq_handler   /* DMA2 Channel 1 */
	.word   DMA2_Channel2_irq_handler   /* DMA2 Channel 2 */
	.word   DMA2_Channel3_irq_handler   /* DMA2 Channel 3 */
	.word   DMA2_Channel4_irq_handler   /* DMA2 Channel 4 */
	.word   DMA2_Channel5_irq_handler   /* DMA2 Channel 5 */
	.word   ETH_irq_handler             /* ETH */
	.word   ETH_WKUP_irq_handler        /* ETH WakeUp */
	.word   CAN2_TX_irq_handler         /* CAN2 TX */
	.word   CAN2_RX0_irq_handler        /* CAN2 RX0 */
	.word   CAN2_RX1_irq_handler        /* CAN2 RX1 */
	.word   CAN2_SCE_irq_handler        /* CAN2 SCE */
	.word   USBFS_irq_handler           /* USBFS */
	.word   USBHSWakeup_irq_handler     /* USBHS Wakeup */
	.word   USBHS_irq_handler           /* USBHS */
	.word   DVP_irq_handler             /* DVP */
	.word   UART6_irq_handler           /* UART6 */
	.word   UART7_irq_handler           /* UART7 */
	.word   UART8_irq_handler           /* UART8 */
	.word   TIM9_BRK_irq_handler        /* TIM9 Break */
	.word   TIM9_UP_irq_handler         /* TIM9 Update */
	.word   TIM9_TRG_COM_irq_handler    /* TIM9 Trigger and Commutation */
	.word   TIM9_CC_irq_handler         /* TIM9 Capture Compare */
	.word   TIM10_BRK_irq_handler       /* TIM10 Break */
	.word   TIM10_UP_irq_handler        /* TIM10 Update */
	.word   TIM10_TRG_COM_irq_handler   /* TIM10 Trigger and Commutation */
	.word   TIM10_CC_irq_handler        /* TIM10 Capture Compare */
	.word   DMA2_Channel6_irq_handler   /* DMA2 Channel 6 */
	.word   DMA2_Channel7_irq_handler   /* DMA2 Channel 7 */
	.word   DMA2_Channel8_irq_handler   /* DMA2 Channel 8 */
	.word   DMA2_Channel9_irq_handler   /* DMA2 Channel 9 */
	.word   DMA2_Channel10_irq_handler  /* DMA2 Channel 10 */
	.word   DMA2_Channel11_irq_handler  /* DMA2 Channel 11 */
_vector_end:
	.equ _vector_size, (_vector_end - _vector)

NMI_handler:
HardFault_handler:
Ecall_M_Mode_handler:
Ecall_U_Mode_handler:
Break_Point_handler:
SysTick_handler:
SW_handler:
WWDG_irq_handler:
PVD_irq_handler:
TAMPER_irq_handler:
RTC_irq_handler:
FLASH_irq_handler:
RCC_irq_handler:
EXTI0_irq_handler:
EXTI1_irq_handler:
EXTI2_irq_handler:
EXTI3_irq_handler:
EXTI4_irq_handler:
DMA1_Channel1_irq_handler:
DMA1_Channel2_irq_handler:
DMA1_Channel3_irq_handler:
DMA1_Channel4_irq_handler:
DMA1_Channel5_irq_handler:
DMA1_Channel6_irq_handler:
DMA1_Channel7_irq_handler:
ADC1_2_irq_handler:
USB_HP_CAN1_TX_irq_handler:
USB_LP_CAN1_RX0_irq_handler:
CAN1_RX1_irq_handler:
CAN1_SCE_irq_handler:
EXTI9_5_irq_handler:
TIM1_BRK_irq_handler:
TIM1_UP_irq_handler:
TIM1_TRG_COM_irq_handler:
TIM1_CC_irq_handler:
TIM2_irq_handler:
TIM3_irq_handler:
TIM4_irq_handler:
I2C1_EV_irq_handler:
I2C1_ER_irq_handler:
I2C2_EV_irq_handler:
I2C2_ER_irq_handler:
SPI1_irq_handler:
SPI2_irq_handler:
USART1_irq_handler:
USART2_irq_handler:
USART3_irq_handler:
EXTI15_10_irq_handler:
RTCAlarm_irq_handler:
USBWakeUp_irq_handler:
TIM8_BRK_irq_handler:
TIM8_UP_irq_handler:
TIM8_TRG_COM_irq_handler:
TIM8_CC_irq_handler:
RNG_irq_handler:
SDIO_irq_handler:
TIM5_irq_handler:
SPI3_irq_handler:
UART4_irq_handler:
UART5_irq_handler:
TIM6_irq_handler:
TIM7_irq_handler:
DMA2_Channel1_irq_handler:
DMA2_Channel2_irq_handler:
DMA2_Channel3_irq_handler:
DMA2_Channel4_irq_handler:
DMA2_Channel5_irq_handler:
ETH_irq_handler:
ETH_WKUP_irq_handler:
CAN2_TX_irq_handler:
CAN2_RX0_irq_handler:
CAN2_RX1_irq_handler:
CAN2_SCE_irq_handler:
USBFS_irq_handler:
USBHSWakeup_irq_handler:
USBHS_irq_handler:
DVP_irq_handler:
UART6_irq_handler:
UART7_irq_handler:
UART8_irq_handler:
TIM9_BRK_irq_handler:
TIM9_UP_irq_handler:
TIM9_TRG_COM_irq_handler:
TIM9_CC_irq_handler:
TIM10_BRK_irq_handler:
TIM10_UP_irq_handler:
TIM10_TRG_COM_irq_handler:
TIM10_CC_irq_handler:
DMA2_Channel6_irq_handler:
DMA2_Channel7_irq_handler:
DMA2_Channel8_irq_handler:
DMA2_Channel9_irq_handler:
DMA2_Channel10_irq_handler:
DMA2_Channel11_irq_handler:
1:
	j 1b
	.option rvc;

	.word 0x0

	.section .text
reset:
clean_sram:
	.equ SRAM_BASE, 0x20000000
	.equ SRAM_SIZE, 0x00020000
	li wp, SRAM_BASE
	li xp, -1
	li yp, SRAM_SIZE
2:
	beqz yp, 1f
	sw xp, 0(wp)
	addi wp, wp, ADDRSIZE
	addi yp, yp, -ADDRSIZE
	j 2b
1:

copy_vector:
	la wp, _vector
	la xp, _ram_vector
	la yp, _ram_vector_end
1:
	beq xp, yp, 2f
	lw tos, 0(wp)
	sw tos, 0(xp)
	addi wp, wp, ADDRSIZE
	addi xp, xp, ADDRSIZE
	j 1b
2:

	.equ RCC_BASE, 0x40021000
	li wp, RCC_BASE

	.equ RCC_CTLR, 0x00
	.equ RCC_HSEON, (1 << 16)
	.equ RCC_HSERDY, (1 << 17)
	lw xp, RCC_CTLR(wp)
	li yp, RCC_HSEON
	or xp, xp, yp
	sw xp, RCC_CTLR(wp)

	li yp, RCC_HSERDY
1:
	lw xp, RCC_CTLR(wp)
	and xp, xp, yp
	bne xp, yp, 1b

	.equ RCC_CFGR0, 0x04
	.equ RCC_PLLSRC_PREDIV1, (1 << 16)
	lw xp, RCC_CFGR0(wp)
	li yp, RCC_PLLSRC_PREDIV1
	or xp, xp, yp
	sw xp, RCC_CFGR0(wp)

	.equ RCC_PLLMUL_12MUL, (0xA << 18)
	lw xp, RCC_CFGR0(wp)
	li yp, RCC_PLLMUL_12MUL
	or xp, xp, yp
	sw xp, RCC_CFGR0(wp)

	.equ RCC_PLLON, (1 << 24)
	lw xp, RCC_CTLR(wp)
	li yp, RCC_PLLON
	or xp, xp, yp
	sw xp, RCC_CTLR(wp)
	
	.equ RCC_PLLRDY, (1 << 25)
	li yp, RCC_PLLRDY
1:
	lw xp, RCC_CTLR(wp)
	and xp, xp, yp
	bne xp, yp, 1b

	.equ RCC_SW_PLL, (0x2 << 0)
	lw xp, RCC_CFGR0(wp)
	ori xp, xp, RCC_SW_PLL
	sw xp, RCC_CFGR0(wp)

	.equ RCC_SWS_MASK, (0x3 << 2)
	.equ RCC_SWS_PLL,  (0x2 << 2)
	li yp, RCC_SWS_PLL
1:
	lw xp, RCC_CFGR0(wp)
	andi xp, xp, RCC_SWS_MASK
	bne xp, yp, 1b

	.equ RCC_RSTSCKR, 0x24
	.equ RCC_LSION, (1 << 0)
	lw xp, RCC_RSTSCKR(wp)
	ori xp, xp, RCC_LSION
	sw xp, RCC_RSTSCKR(wp)

	.equ RCC_LSIRDY, (1 << 1)
	li yp, RCC_LSIRDY
1:
	lw xp, RCC_RSTSCKR(wp)
	and xp, xp, yp
	bne xp, yp, 1b

pb2_clk_init:
	.equ RCC_APB2PCENR, 0x18
	.equ RCC_UART1_EN, (1 << 14)
	.equ RCC_SPI1_EN, (1 << 12)
	.equ RCC_IOPB_EN, (1 << 3)
	.equ RCC_IOPA_EN, (1 << 2)
	lw xp, RCC_APB2PCENR(wp)
	li yp, RCC_UART1_EN | RCC_SPI1_EN | RCC_IOPB_EN | RCC_IOPA_EN
	or xp, xp, yp
	sw xp, RCC_APB2PCENR(wp)

pb1_clk_init:
	.equ RCC_APB1PCENR, 0x1C
	.equ RCC_I2C2_EN, (1 << 22)
	lw xp, RCC_APB1PCENR(wp)
	li yp, RCC_I2C2_EN
	or xp, xp, yp
	sw xp, RCC_APB1PCENR(wp)

	.equ GPIOA_BASE, 0x40010800
	.equ GPIOB_BASE, 0x40010C00
	.equ GPIO_CFGLR, 0x00
	.equ GPIO_CFGHR, 0x04
	.equ GPIO_OUTDR, 0x0C

	.equ GPIO_CFG_PPOUT_2Mhz, 0x2
	.equ GPIO_CFG_PPOUT_50Mhz, 0x3
	.equ GPIO_CFG_MUPPOUT_50Mhz, 0xB
	.equ GPIO_CFG_MUODOUT_50Mhz, 0xF
	.equ GPIO_CFG_PUIN, 0x8

	j 1f
	.p2align 2, 0xFF
gpioa_config:
	# PA2 SPI FLASH CS
	.equ PA2CFG, (GPIO_CFG_PPOUT_50Mhz << 8)
	# PA4 SPI1 HW CS
	.equ PA4CFG, (GPIO_CFG_PPOUT_50Mhz << 16)
	# PA5 SPI1 CLK
	.equ PA5CFG, (GPIO_CFG_MUPPOUT_50Mhz << 20)
	# PA6 SPI1 MISO
	.equ PA6CFG, (GPIO_CFG_PUIN << 24)
	# PA7 SPI1 MOSI
	.equ PA7CFG, (GPIO_CFG_MUPPOUT_50Mhz << 28)
	.word PA2CFG | PA4CFG | PA5CFG | PA6CFG | PA7CFG

	# PA9 UART1 TX
	.equ PA9CFG, (GPIO_CFG_MUPPOUT_50Mhz << 4)
	# PA10 UART1 RX
	.equ PA10CFG, (GPIO_CFG_PUIN << 8)
	# PA15 LED RED
	.equ PA15CFG, (GPIO_CFG_PPOUT_2Mhz << 28)
	.word PA9CFG | PA10CFG | PA15CFG
gpioa_outdr:
	.word (1 << 2) | (1 << 4) | (1 << 6) | (1 << 10) | (1 << 15)
gpiob_config:
	# PB4 LED GREEN
	.equ PB4CFG, (GPIO_CFG_PPOUT_2Mhz << 16)
	.word PB4CFG
	# PB10 I2C2 SCL
	.equ PB10CFG, (GPIO_CFG_MUODOUT_50Mhz << 8)
	# PB11 I2C2 SDA
	.equ PB11CFG, (GPIO_CFG_MUODOUT_50Mhz << 12)
	.word PB10CFG | PB11CFG
gpiob_outdr:
	.word (1 << 4)
	.p2align 2, 0xFF
1:	

	la wp, GPIOA_BASE
	la yp, gpioa_config
	lw xp, 0(yp)
	sw xp, GPIO_CFGLR(wp)
	lw xp, 4(yp)
	sw xp, GPIO_CFGHR(wp)
	la yp, gpioa_outdr
	lw xp, 0(yp)
	sw xp, GPIO_OUTDR(wp)

	la wp, GPIOB_BASE
	la yp, gpiob_config
	lw xp, 0(yp)
	sw xp, GPIO_CFGLR(wp)
	lw xp, 4(yp)
	sw xp, GPIO_CFGHR(wp)
	la yp, gpiob_outdr
	lw xp, 0(yp)
	sw xp, GPIO_OUTDR(wp)

	.equ UART1_BASE, 0x40013800
	.equ UART_DATAR, 0x04
	.equ UART_BRR, 0x08

	.equ BAUD_921600, ((6 << 4) | (6 << 0))

	li wp, UART1_BASE
	li xp, BAUD_921600
	sw xp, UART_BRR(wp)

	.equ UART_CTLR1, 0x0C
	.equ UART_UE, (1 << 13)
	.equ UART_TE, (1 << 3)
	.equ UART_RE, (1 << 2)
	li xp, UART_UE | UART_TE | UART_RE
	sw xp, UART_CTLR1(wp)

	j forth

	call a_fail

	.set lastword, 0

	.equ ATTR_IMMED, (1 << 7)
	.equ ATTR_HIDEN, (1 << 6)
	.equ NLEN_MASK,  0x1F
	.macro wdef label, name, entry, link, attr
		.p2align 2, 0xFF
	l_\label:
		.word \link
	f_\label:
		.word \entry
		.byte \attr + nlen_\label
	n_\label:
		.ascii "\name"
	n_end_\label:
		.set nlen_\label, n_end_\label - n_\label
		.set lastword, f_\label
		.p2align 2, 0xFF
	.endm

early_txc:
	.equ UART_STATR, (0x00)
	.equ UART_TC, (1 << 6)
	li wp, UART1_BASE
	li yp, UART_TC
1:
	lw xp, UART_STATR(wp)
	and xp, xp, yp
	bne xp, yp, 1b
	sw tos, UART_DATAR(wp)
	ret

early_rxc:
	.equ UART_RXNE, (1 << 5)
	li wp, UART1_BASE
	li yp, UART_RXNE
1:
	lw xp, UART_STATR(wp)
	and xp, xp, yp
	bne xp, yp, 1b
	lw tos, UART_DATAR(wp)
	ret

	wdef okay, "okay", a_okay, 0, 0
a_okay:
	li tos, 'O'
	call early_txc
	li tos, 'K'
	call early_txc
	li tos, 'A'
	call early_txc
	li tos, 'Y'
	call early_txc
	ret

	wdef fail, "fail", a_fail, f_okay, 0
a_fail:
	li tos, 'F'
	call early_txc
	li tos, 'A'
	call early_txc
	li tos, 'I'
	call early_txc
	li tos, 'L'
	call early_txc
	ret

	wdef next, "next", a_next, f_fail, 0
a_next:
	lw wp, 0(ip)
	addi ip, ip, ADDRSIZE
	lw xp, 0(wp)
	jr xp

	.macro next
		j a_next
	.endm

rpush:
	sw tos, 0(rp)
	addi rp, rp, -ADDRSIZE
	ret

rpop:
	addi rp, rp, ADDRSIZE
	lw tos, 0(rp)
	ret

	wdef call, "call", a_call, f_next, 0
a_call:
	mv tos, ip
	call rpush
	lbu xp, ADDRSIZE(wp)
	andi xp, xp, NLEN_MASK
	addi xp, xp, 1 + ADDRSIZE + ADDRSIZE - 1
	andi xp, xp, -ADDRSIZE
	add ip, wp, xp
	next

	wdef exit, "exit", a_exit, f_call, 0
a_exit:
	call rpop
	mv ip, tos
	next

	wdef noop, "noop", a_call, f_exit, 0
	.word f_exit

dpush:
	sw tos, 0(sp)
	addi sp, sp, -ADDRSIZE
	ret

	wdef branch, "branch", a_branch, f_noop, 0
a_branch:
	lw ip, 0(ip)
	next

	wdef lit, "lit", a_lit, f_branch, 0
a_lit:
	lw tos, 0(ip)
	addi ip, ip, ADDRSIZE
	call dpush
	next

	.equ SS_DUND, (1 << 1)
dpop:
	addi sp, sp, ADDRSIZE
	ble sp, st, 1f
	ori ss, ss, SS_DUND
1:
	lw tos, 0(sp)
	ret

	wdef 0branch, "0branch", a_0branch, f_lit, 0
a_0branch:
	lw wp, 0(ip)
	addi ip, ip, ADDRSIZE
	call dpop 
	bnez tos, 1f
	mv ip, wp
1:
	next

	wdef doconst, "doconst", a_doconst, f_0branch, 0
a_doconst:
	lbu xp, ADDRSIZE(wp)
	andi xp, xp, NLEN_MASK
	addi xp, xp, 1 + ADDRSIZE + ADDRSIZE - 1
	andi xp, xp, -ADDRSIZE
	add wp, wp, xp
	lw tos, 0(wp)
	call dpush
	next

	wdef true, "true", a_doconst, f_doconst, 0
	.word -1

	wdef false, "false", a_doconst, f_true, 0
	.word 0

	wdef 0, "0", a_doconst, f_false, 0
	.word 0

	wdef 1, "1", a_doconst, f_0, 0
	.word 1

	wdef 2, "2", a_doconst, f_1, 0
	.word 2

	wdef 3, "3", a_doconst, f_2, 0
	.word 3

	wdef 4, "4", a_doconst, f_3, 0
	.word 4

	wdef 5, "5", a_doconst, f_4, 0
	.word 5

	wdef 6, "6", a_doconst, f_5, 0
	.word 6

	wdef 7, "7", a_doconst, f_6, 0
	.word 7

	wdef 8, "8", a_doconst, f_7, 0
	.word 8

	wdef 9, "9", a_doconst, f_8, 0
	.word 9

	wdef 0xA, "0xA", a_doconst, f_9, 0
	.word 0xA

	wdef 0xB, "0xB", a_doconst, f_0xA, 0
	.word 0xB

	wdef 0xC, "0xC", a_doconst, f_0xB, 0
	.word 0xC

	wdef 0xD, "0xD", a_doconst, f_0xC, 0
	.word 0xD

	wdef 0xE, "0xE", a_doconst, f_0xD, 0
	.word 0xE

	wdef 0xF, "0xF", a_doconst, f_0xE, 0
	.word 0xF

	wdef xor, "xor", a_xor, f_0xF, 0
a_xor:
	call dpop
	mv wp, tos
	call dpop
	xor tos, tos, wp
	call dpush
	next

	wdef 2lit, "2lit", a_2lit, f_xor, 0
a_2lit:
	lw tos, 0(ip)
	addi ip, ip, ADDRSIZE
	call dpush
	lw tos, 0(ip)
	addi ip, ip, ADDRSIZE
	call dpush
	next

	wdef equ, "=", a_equ, f_2lit, 0
a_equ:
	call dpop
	mv wp, tos
	call dpop
	beq tos, wp, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next
	
	wdef eqz, "0=", a_call, f_equ, 0
	.word f_0
	.word f_equ
	.word f_exit

	wdef invert, "invert", a_call, f_eqz, 0
	.word f_true
	.word f_xor
	.word f_exit

	wdef neq, "<>", a_call, f_invert, 0
	.word f_equ
	.word f_invert
	.word f_exit

	wdef nez, "0<>", a_call, f_neq, 0
	.word f_eqz
	.word f_invert
	.word f_exit

	wdef faileqz, "fail0=", a_call, f_nez, 0
	.word f_eqz
	.word f_0branch
	.word 1f
	.word f_fail
1:
	.word f_exit

	wdef failnez, "fail0<>", a_call, f_faileqz, 0
	.word f_nez
	.word f_0branch
	.word 1f
	.word f_fail
1:
	.word f_exit

	wdef failneq, "fail<>", a_call, f_failnez, 0
	.word f_neq
	.word f_failnez
	.word f_exit

	wdef failequ, "fail=", a_call, f_failneq, 0
	.word f_equ
	.word f_failnez
	.word f_exit

	wdef add, "+", a_add, f_failequ, 0
a_add:
	call dpop
	mv wp, tos
	call dpop
	add tos, tos, wp
	call dpush
	next

	wdef inc, "1+", a_inc, f_add, 0
a_inc:
	call dpop
	addi tos, tos, 1
	call dpush
	next

	wdef negate, "negate", a_call, f_inc, 0
	.word f_invert
	.word f_inc
	.word f_exit

	wdef sub, "-", a_sub, f_negate, 0
a_sub:
	call dpop
	mv wp, tos
	call dpop
	sub tos, tos, wp
	call dpush
	next

	wdef execute, "execute", a_execute, f_sub, 0
a_execute:
	call dpop
	mv wp, tos
	lw xp, 0(wp)
	jr xp

	wdef load, "@", a_call, f_execute, 0
	.word f_dup
	.word f_VSPI_FLASH_BASE
	.word f_VSPI_FLASH_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VSPI_FLASH_BASE
	.word f_sub
	.word f_spi_flash_read4b
	.word f_exit
1:
	.word f_dup
	.word f_VI2C_FT24_BASE
	.word f_VI2C_FT24_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VI2C_FT24_BASE
	.word f_sub
	.word f_i2c_ft24_read4b
	.word f_exit
1:
	.word f_memload
	.word f_exit

	wdef memload, "mem@", a_memload, f_load, 0
a_memload:
	call dpop
	lw tos, 0(tos)
	call dpush
	next

	wdef loadexecute, "@execute", a_call, f_memload, 0
	.word f_load
	.word f_execute
	.word f_exit

	wdef func_emit, "func-emit", a_doconst, f_loadexecute, 0
	.word func_emit

	wdef emit, "emit", a_call, f_func_emit, 0
	.word f_func_emit
	.word f_loadexecute
	.word f_exit

	wdef early_txc, "early-txc", a_early_txc, f_emit, 0
a_early_txc:
	call dpop
	call early_txc
	next

	wdef dec, "1-", a_dec, f_early_txc, 0
	/*
	.word f_1
	.word f_sub
	.word f_exit
	*/
a_dec:
	call dpop
	addi tos, tos, -1
	call dpush
	next

	wdef cload, "c@", a_call, f_dec, 0
	.word f_dup
	.word f_VSPI_FLASH_BASE
	.word f_VSPI_FLASH_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VSPI_FLASH_BASE
	.word f_sub
	.word f_spi_flash_read1b
	.word f_exit
1:
	.word f_dup
	.word f_VI2C_FT24_BASE
	.word f_VI2C_FT24_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VI2C_FT24_BASE
	.word f_sub
	.word f_i2c_ft24_read1b
	.word f_exit
1:
	.word f_memcload
	.word f_exit

	wdef memcload, "memc@", a_memcload, f_cload, 0
a_memcload:
	call dpop
	lbu tos, 0(tos)
	call dpush
	next

	wdef swap, "swap", a_swap, f_memcload, 0
a_swap:
	call dpop
	mv wp, tos
	call dpop
	mv xp, tos

	mv tos, wp
	call dpush
	mv tos, xp
	call dpush

	next

	wdef dup, "dup", a_dup, f_swap, 0
a_dup:
	call dpop
	call dpush
	call dpush
	next

	wdef drop, "drop", a_drop, f_dup, 0
a_drop:
	call dpop
	next

	wdef 2drop, "2drop", a_call, f_drop, 0
	.word f_drop
	.word f_drop
	.word f_exit

	wdef type, "type", a_call, f_2drop, 0
2:
	.word f_dup
	.word f_0branch
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

	wdef cr, "cr", a_call, f_type, 0
	.word f_2lit
	.word '\n'
	.word '\r'
	.word f_emit
	.word f_emit
	.word f_exit

	wdef ok, "ok", a_call, f_cr, 0
	.word f_2lit
	.word 'k'
	.word 'o'
	.word f_emit
	.word f_emit
	.word f_exit

	wdef motd, "motd", a_call, f_ok, 0
	.word f_cr
	.word f_2lit
	.word _binary_motd_txt_start
	.word _binary_motd_txt_size
	.word f_type
	.word f_cr
	.word f_exit

	wdef spload, "sp@", a_spload, f_motd, 0
a_spload:
	mv tos, sp
	call dpush
	next

	wdef stload, "st@", a_stload, f_spload, 0
a_stload:
	mv tos, st
	call dpush
	next

	wdef rshift, "rshift", a_rshift, f_stload, 0
a_rshift:
	call dpop
	mv wp, tos
	call dpop
	srl tos, tos, wp
	call dpush
	next

	wdef 4div, "4/", a_call, f_rshift, 0
	.word f_2
	.word f_rshift
	.word f_exit

	wdef depth, "depth", a_call, f_4div, 0
	.word f_spload
	.word f_stload
	.word f_swap
	.word f_sub
	.word f_4div
	.word f_exit

	wdef ssload, "ss@", a_ssload, f_depth, 0
a_ssload:
	mv tos, ss
	call dpush
	next

	wdef SS_DUND, "SS_DUND", a_doconst, f_ssload, 0
	.word SS_DUND

	wdef and, "and", a_and, f_SS_DUND, 0
a_and:
	call dpop
	mv wp, tos
	call dpop
	and tos, tos, wp
	call dpush
	next

	wdef sschk, "sschk", a_call, f_and, 0
	.word f_ssload
	.word f_SS_DUND
	.word f_and
	.word f_eqz
	.word f_exit

	wdef ssstore, "ss!", a_ssstore, f_sschk, 0
a_ssstore:
	call dpop
	mv ss, tos
	next

	wdef ssrst, "ssrst", a_call, f_ssstore, 0
	.word f_0
	.word f_ssstore
	.word f_exit

	wdef dzchk, "dzchk", a_call, f_ssrst, 0
	.word f_depth
	.word f_failnez
	.word f_sschk
	.word f_faileqz
	.word f_exit

	wdef xdigits, "xdigits", a_doconst, f_dzchk, 0
	.word _xdigits

_xdigits:
	.ascii "0123456789ABCDEF"
	.p2align 2, 0xFF

	wdef num2hex, "num2hex", a_call, f_xdigits, 0
	.word f_0xF
	.word f_and
	.word f_xdigits
	.word f_add
	.word f_cload
	.word f_exit

	wdef hex4, "hex4", a_call, f_num2hex, 0
	.word f_num2hex
	.word f_emit
	.word f_exit

	wdef hex8, "hex8", a_call, f_hex4, 0
	.word f_dup
	.word f_4
	.word f_rshift
	.word f_hex4
	.word f_hex4
	.word f_exit

	wdef hex16, "hex16", a_call, f_hex8, 0
	.word f_dup
	.word f_8
	.word f_rshift
	.word f_hex8
	.word f_hex8
	.word f_exit

	wdef hex32, "hex32", a_call, f_hex16, 0
	.word f_dup
	.word f_lit
	.word 16
	.word f_rshift
	.word f_hex16
	.word f_hex16
	.word f_exit


	wdef func_dot, "func-dot", a_doconst, f_hex32, 0
	.word func_dot

	wdef dot, ".", a_call, f_func_dot, 0
	.word f_func_dot
	.word f_loadexecute
	.word f_exit

	wdef quest, "?", a_call, f_dot, 0
	.word f_load
	.word f_dot
	.word f_exit

	wdef dsdump, ".s", a_call, f_quest, 0
	.word f_depth
	.word f_lit
	.word '('
	.word f_emit
	.word f_dot
	.word f_lit
	.word ')'
	.word f_emit

	.word f_depth
	.word f_stload
	.word f_swap
2:
	.word f_dup
	.word f_0branch
	.word 1f

	.word f_dec
	.word f_swap
	.word f_dup
	.word f_load
	.word f_dot
	.word f_lit
	.word ' '
	.word f_emit
	.word f_4
	.word f_sub
	.word f_swap
	.word f_branch
	.word 2b

1:
	.word f_2drop
	.word f_exit

	wdef ltz, "0<", a_ltz, f_dsdump, 0
a_ltz:
	call dpop
	bltz tos, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next
	
	wdef gez, "0>=", a_call, f_ltz, 0
	.word f_ltz
	.word f_invert
	.word f_exit

	wdef lt, "<", a_call, f_gez, 0
	.word f_sub
	.word f_ltz
	.word f_exit

	wdef gt, ">", a_call, f_lt, 0
	.word f_swap
	.word f_lt
	.word f_exit

	wdef tor, ">r", a_tor, f_gt, 0
a_tor:
	call dpop
	call rpush
	next

	wdef fromr, "r>", a_fromr, f_tor, 0
a_fromr:
	call rpop
	call dpush
	next

	wdef over, "over", a_call, f_fromr, 0
	.word f_tor
	.word f_dup
	.word f_fromr
	.word f_swap
	.word f_exit

	wdef 2dup, "2dup", a_call, f_over, 0
	.word f_over
	.word f_over
	.word f_exit

	wdef nip, "nip", a_call, f_2dup, 0
	.word f_swap
	.word f_drop
	.word f_exit

	wdef min, "min", a_call, f_nip, 0
	.word f_2dup
	.word f_lt
	.word f_0branch
	.word 1f
	.word f_drop
	.word f_exit
1:
	.word f_nip
	.word f_exit

	wdef rot, "rot", a_call, f_min, 0
	.word f_tor
	.word f_swap
	.word f_fromr
	.word f_swap
	.word f_exit

	wdef compare, "compare", a_call, f_rot, 0
	.word f_rot
	.word f_min

	.word f_dup
	.word f_0branch
	.word compare_fail
1:
	.word f_dup
	.word f_0branch
	.word compare_true
	.word f_dec
	.word f_rot
	.word f_dup
	.word f_cload
	.word f_tor
	.word f_rot
	.word f_dup
	.word f_cload
	.word f_fromr
	.word f_equ
	.word f_0branch
	.word compare_fail
	.word f_inc
	.word f_swap
	.word f_inc
	.word f_rot
	.word f_branch
	.word 1b
compare_true:
	.word f_2drop
	.word f_drop
	.word f_true
	.word f_exit
compare_fail:
	.word f_2drop
	.word f_drop
	.word f_false
	.word f_exit

	wdef wlinkload, "wlink@", a_call, f_compare, 0
	.word f_4
	.word f_sub
	.word f_load
	.word f_exit

	wdef NLEN_MASK, "NLEN_MASK", a_doconst, f_wlinkload, 0
	.word NLEN_MASK

	wdef wnlenload, "wnlen@", a_call, f_NLEN_MASK, 0
	.word f_4
	.word f_add
	.word f_cload
	.word f_NLEN_MASK
	.word f_and
	.word f_exit

	wdef wnameload, "wname@", a_call, f_wnlenload, 0
	.word f_5
	.word f_add
	.word f_exit

	wdef latest, "latest", a_doconst, f_wnameload, 0
	.word latest

	wdef latestload, "latest@", a_call, f_latest, 0
	.word f_latest
	.word f_load
	.word f_exit

	wdef words, "words", a_call, f_latestload, 0
	.word f_latestload

2:
	.word f_dup
	.word f_0branch
	.word 1f

	.word f_dup
	.word f_wnameload
	.word f_over
	.word f_wnlenload
	.word f_type
	.word f_lit
	.word ' '
	.word f_emit

	.word f_wlinkload
	.word f_branch
	.word 2b

1:
	.word f_drop
	.word f_exit

	wdef 2swap, "2swap", a_call, f_words, 0
	.word f_rot
	.word f_tor
	.word f_rot
	.word f_fromr
	.word f_exit

	wdef 2over, "2over", a_call, f_2swap, 0
	.word f_tor
	.word f_tor
	.word f_2dup
	.word f_fromr
	.word f_fromr
	.word f_2swap
	.word f_exit

	wdef ATTR_HIDEN, "ATTR_HIDEN", a_doconst, f_2over, 0
	.word ATTR_HIDEN

	wdef wishiden, "wishiden", a_call, f_ATTR_HIDEN, 0
	.word f_4
	.word f_add
	.word f_cload
	.word f_ATTR_HIDEN
	.word f_and
	.word f_nez
	.word f_exit

	wdef nothing, "nothing", 0, f_wishiden, ATTR_HIDEN 

	wdef find, "find", a_call, f_nothing, 0
	.word f_latestload

3:
	.word f_dup
	.word f_0branch
	.word 1f

	.word f_dup
	.word f_wishiden
	.word f_0branch
	.word 4f
	.word f_branch
	.word 2f
4:

	.word f_2dup
	.word f_wnlenload
	.word f_equ
	.word f_0branch
	.word 2f

	.word f_dup
	.word f_wnameload
	.word f_2over
	.word f_swap
	.word f_over
	.word f_compare
	.word f_0branch
	.word 2f

	.word f_nip
	.word f_nip
	.word f_exit

2:
	.word f_wlinkload
	.word f_branch
	.word 3b

1:
	.word f_nip
	.word f_nip
	.word f_exit

	wdef TIBSIZE, "TIBSIZE", a_doconst, f_find, 0
	.word TIBSIZE

	wdef tib, "tib", a_doconst, f_TIBSIZE, 0
	.word tib

	wdef toin, ">in", a_doconst, f_tib, 0
	.word toin

	wdef toinload, ">in@", a_call, f_toin, 0
	.word f_toin
	.word f_load
	.word f_exit

	wdef store, "!", a_call, f_toinload, 0
	.word f_dup
	.word f_VSPI_FLASH_BASE
	.word f_VSPI_FLASH_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VSPI_FLASH_BASE
	.word f_sub
	.word f_spi_flash_write4b
	.word f_exit
1:
	.word f_dup
	.word f_VI2C_FT24_BASE
	.word f_VI2C_FT24_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VI2C_FT24_BASE
	.word f_sub
	.word f_i2c_ft24_write4b
	.word f_exit
1:
	.word f_memstore
	.word f_exit

	wdef memstore, "mem!", a_memstore, f_store, 0
a_memstore:
	call dpop
	mv wp, tos
	call dpop
	sw tos, 0(wp)
	next

	wdef toinstore, ">in!", a_call, f_memstore, 0
	.word f_toin
	.word f_store
	.word f_exit

	wdef toinrst, ">inrst", a_call, f_toinstore, 0
	.word f_0
	.word f_toinstore
	.word f_exit

	wdef toinchk, ">inchk", a_call, f_toinrst, 0
	.word f_toinload
	.word f_dup
	.word f_gez
	.word f_swap
	.word f_TIBSIZE
	.word f_lt
	.word f_and
	.word f_exit

	wdef cstore, "c!", a_call, f_toinchk, 0
	.word f_dup
	.word f_VSPI_FLASH_BASE
	.word f_VSPI_FLASH_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VSPI_FLASH_BASE
	.word f_sub
	.word f_spi_flash_write1b
	.word f_exit
1:
	.word f_dup
	.word f_VI2C_FT24_BASE
	.word f_VI2C_FT24_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VI2C_FT24_BASE
	.word f_sub
	.word f_i2c_ft24_write1b
	.word f_exit
1:	
	.word f_memcstore
	.word f_exit
	
	wdef memcstore, "memc!", a_cstore, f_cstore, 0
a_cstore:
	call dpop
	mv wp, tos
	call dpop
	sb tos, 0(wp)
	next

	wdef tibpush, "tibpush", a_call, f_memcstore, 0
	.word f_tib
	.word f_toinload
	.word f_add
	.word f_cstore
	.word f_toinload
	.word f_inc
	.word f_toinstore
	.word f_toinchk
	.word f_0branch
	.word 1f
	.word f_exit
1:
	.word f_toinrst
	.word f_exit

	wdef tibdrop, "tibdrop", a_call, f_tibpush, 0
	.word f_toinload
	.word f_dec
	.word f_toinstore
	.word f_toinchk
	.word f_0branch
	.word 1f
	.word f_exit
1:
	.word f_toinrst
	.word f_exit

	wdef or, "or", a_or, f_tibdrop, 0
a_or:
	call dpop
	mv wp, tos
	call dpop
	or tos, tos, wp
	call dpush
	next

	wdef isnewline, "isnewline", a_call, f_or, 0
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

	wdef isdelete, "isdelete", a_call, f_isnewline, 0
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

	wdef isspace, "isspace", a_call, f_isdelete,  0
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

	wdef func_key, "func-key", a_doconst, f_isspace, 0
	.word func_key

	wdef key, "key", a_call, f_func_key, 0
	.word f_func_key
	.word f_loadexecute
	.word f_exit

	wdef early_rxc, "early-rxc", a_early_rxc, f_key, 0
a_early_rxc:
	call early_rxc
	call dpush
	next

	wdef _token, "_token", a_call, f_early_rxc, 0
1:
	.word f_key
	.word f_dup
	.word f_isspace
	.word f_0branch
	.word 2f
	.word f_emit
	.word f_exit
2:
	.word f_dup
	.word f_isnewline
	.word f_0branch
	.word 2f
	.word f_cr
	.word f_drop
	.word f_exit
2:
	.word f_dup
	.word f_isdelete
	.word f_0branch
	.word 2f
	.word f_emit
	.word f_tibdrop
	.word f_branch
	.word 1b
2:
	.word f_dup
	.word f_emit
	.word f_tibpush
	.word f_branch
	.word 1b

	wdef token, "token", a_call, f__token, 0
1:
	.word f_toinrst
	.word f__token
	.word f_toinload
	.word f_0branch
	.word 1b
	.word f_exit

	wdef le, "<=", a_call, f_token, 0
	.word f_2dup
	.word f_lt
	.word f_tor
	.word f_equ
	.word f_fromr
	.word f_or
	.word f_exit

	wdef within, "within", a_within, f_le, 0
a_within:
	call dpop
	mv wp, tos
	call dpop
	mv xp, tos
	call dpop
	blt tos, wp, 1f
	mv tos, zero
	call dpush
	next
1:
	bge tos, xp, 1f
	mv tos, zero
	call dpush
	next
1:
	li tos, -1
	call dpush
	next



	// (n) -- (flag)
	wdef isxdigit, "isxdigit", a_call, f_within, 0
	.word f_dup
	.word f_2lit
	.word '0'
	.word '9' + 1
	.word f_within
	.word f_swap
	.word f_2lit
	.word 'A'
	.word 'F' + 1
	.word f_within
	.word f_or
	.word f_exit

	// (addr u) -- (flag)
	wdef ishexstr, "ishexstr", a_call, f_isxdigit, 0
	.word f_dup
	.word f_2
	.word f_le
	.word f_0branch
	.word 1f
ishexstr_false:
	.word f_2drop
	.word f_false
	.word f_exit
1:

	.word f_2dup
	.word f_2lit
	.word hdr_hex
	.word 2
	.word f_compare
	.word f_0branch
	.word ishexstr_false

	.word f_dec
	.word f_dec
	.word f_swap
	.word f_inc
	.word f_inc
	.word f_swap

2:
	.word f_dup
	.word f_0branch
	.word 1f

	.word f_dec
	.word f_swap
	.word f_dup
	.word f_cload
	.word f_isxdigit
	.word f_0branch
	.word ishexstr_false

	.word f_inc
	.word f_swap
	.word f_branch
	.word 2b

1:
	.word f_2drop
	.word f_true
	.word f_exit

hdr_hex:
	.ascii "0x"
	.p2align 2, 0xFF

	wdef lshift, "lshift", a_lshift, f_ishexstr, 0
a_lshift:
	call dpop
	mv wp, tos
	call dpop
	sll tos, tos, wp
	call dpush
	next

	wdef 4mul, "4*", a_call, f_lshift, 0
	.word f_2
	.word f_lshift
	.word f_exit

	wdef hex2num, "hex2num", a_call, f_4mul, 0
	.word f_dup
	.word f_lit
	.word '9'
	.word f_le
	.word f_0branch
	.word 1f
	.word f_lit
	.word '0'
	.word f_sub
	.word f_exit
1:
	.word f_lit
	.word 'A'
	.word f_sub
	.word f_0xA
	.word f_add
	.word f_exit

	// (addr u) -- (n)
	wdef hexstr2num, "hexstr2num", a_call, f_hex2num, 0
	.word f_2dup
	.word f_ishexstr
	.word f_0branch
	.word hexstr2num_fail

	.word f_dec
	.word f_dec
	.word f_swap
	.word f_inc
	.word f_inc
	.word f_swap

	.word f_dup
	.word f_dec
	.word f_4mul
	.word f_0

1:
	// addr u shi out
	.word f_2swap
	.word f_dup
	.word f_0branch
	.word hexstr2num_over

	// shi out addr u
	.word f_over
	.word f_cload
	.word f_hex2num
	.word f_tor
	.word f_dec
	.word f_swap
	.word f_inc
	.word f_swap
	.word f_2swap

	// addr u shi out
	.word f_over
	.word f_fromr
	.word f_swap
	.word f_lshift
	.word f_or

	.word f_swap
	.word f_4
	.word f_sub
	.word f_swap

	.word f_branch
	.word 1b

hexstr2num_over:
	.word f_2drop
	.word f_nip
	.word f_exit

hexstr2num_fail:
	.word f_2drop
	.word f_false
	.word f_exit

	wdef func_isnumber, "func-number?", a_doconst, f_hexstr2num, 0
	.word func_isnumber

	wdef func_number, "func-number", a_doconst, f_func_isnumber, 0
	.word func_number

	wdef isnumber, "number?", a_call, f_func_number, 0
	.word f_func_isnumber
	.word f_loadexecute
	.word f_exit

	wdef number, "number", a_call, f_isnumber, 0
	.word f_func_number
	.word f_loadexecute
	.word f_exit

	wdef ATTR_IMMED, "ATTR_IMMED", a_doconst, f_number, 0
	.word ATTR_IMMED

	wdef wisimmed, "wisimmed", a_call, f_ATTR_IMMED, 0
	.word f_4
	.word f_add
	.word f_cload
	.word f_ATTR_IMMED
	.word f_and
	.word f_nez
	.word f_exit

	wdef nopimmed, "nopimmed", a_call, f_wisimmed, ATTR_IMMED
	.word f_exit

	wdef here, "here", a_doconst, f_nopimmed, 0
	.word here

	wdef hereload, "here@", a_call, f_here, 0
	.word f_here
	.word f_load
	.word f_exit

	wdef herestore, "here!", a_call, f_hereload, 0
	.word f_here
	.word f_store
	.word f_exit

	wdef comma, ",", a_call, f_herestore, 0
	.word f_hereload
	.word f_store
	.word f_hereload
	.word f_4
	.word f_add
	.word f_herestore
	.word f_exit

	.equ SS_COMP, (1 << 0)
	wdef SS_COMP, "SS_COMP", a_doconst, f_comma, 0
	.word SS_COMP

	wdef compon, "]", a_call, f_SS_COMP, 0
	.word f_ssload
	.word f_SS_COMP
	.word f_or
	.word f_ssstore
	.word f_exit

	wdef bic, "bic", a_call, f_compon, 0
	.word f_invert
	.word f_and
	.word f_exit

	wdef compoff, "[", a_call, f_bic, ATTR_IMMED
	.word f_ssload
	.word f_SS_COMP
	.word f_bic
	.word f_ssstore
	.word f_exit

	wdef iscomp, "comp?", a_call, f_compoff, 0
	.word f_ssload
	.word f_SS_COMP
	.word f_and
	.word f_nez
	.word f_exit

	wdef sprst, "sprst", a_sprst, f_iscomp, 0
a_sprst:
	mv sp, st
	next

	wdef ccomma, "c,", a_call, f_sprst, 0
	.word f_hereload
	.word f_cstore
	.word f_hereload
	.word f_inc
	.word f_herestore
	.word f_exit

	wdef aligned, "aligned", a_call, f_ccomma, 0
	.word f_3
	.word f_add
	.word f_4
	.word f_negate
	.word f_and
	.word f_exit

	wdef align, "align", a_call, f_aligned, 0
	.word f_hereload
	.word f_aligned
	.word f_herestore
	.word f_exit

	wdef lateststore, "latest!", a_call, f_align, 0
	.word f_latest
	.word f_store
	.word f_exit

	wdef cmove, "cmove", a_call, f_lateststore, 0
2:
	.word f_dup
	.word f_0branch
	.word 1f

	.word f_dec
	.word f_tor
	.word f_over
	.word f_cload
	.word f_over
	.word f_cstore
	.word f_inc
	.word f_swap
	.word f_inc
	.word f_swap
	.word f_fromr

	.word f_branch
	.word 2b
1:
	.word f_drop
	.word f_2drop
	.word f_exit

	// (addr u) --
	wdef defword, "defword", a_call, f_cmove, 0
	.word f_align

	.word f_latestload
	.word f_comma

	.word f_hereload
	.word f_2lit
	.word n_call
	.word 4
	.word f_find
	.word f_load // get word's entry
	.word f_comma

	.word f_over
	.word f_ATTR_HIDEN
	.word f_or
	.word f_ccomma

	.word f_lateststore

	.word f_dup
	.word f_tor
	
	.word f_hereload
	.word f_swap
	.word f_cmove

	.word f_hereload
	.word f_fromr
	.word f_add
	.word f_aligned
	.word f_herestore

	.word f_align
	.word f_exit

	wdef whidenclr, "whidenclr", a_call, f_defword, 0
	.word f_4
	.word f_add
	.word f_dup
	.word f_cload
	.word f_ATTR_HIDEN
	.word f_bic
	.word f_swap
	.word f_cstore
	.word f_exit

	// (value addr u)
	wdef defconst, "defconst", a_call, f_whidenclr, 0
	.word f_defword
	.word f_comma
	.word f_2lit
	.word n_doconst
	.word 7
	.word f_find
	.word f_load
	.word f_latestload
	.word f_store
	.word f_latestload
	.word f_whidenclr
	.word f_exit

	wdef tick, "'", a_call, f_defconst, 0
	.word f_token
	.word f_tib
	.word f_toinload
	.word f_find
	.word f_exit

	// (u) --
	wdef constant, "constant", a_call, f_tick, 0
	.word f_token
	.word f_tib
	.word f_toinload
	.word f_defconst
	.word f_exit

	wdef docom, ":", a_call, f_constant, 0
	.word f_token
	.word f_tib
	.word f_toinload
	.word f_defword
	.word f_compon
	.word f_exit

	wdef doend, ";", a_call, f_docom, ATTR_IMMED
	.word f_2lit
	.word n_exit
	.word 4
	.word f_find
	.word f_comma
	.word f_compoff
	.word f_latestload
	.word f_whidenclr
	.word f_exit

	wdef ifnez, "if", a_call, f_doend, ATTR_IMMED
	.word f_2lit
	.word n_0branch
	.word 7
	.word f_find
	.word f_comma
	.word f_hereload
	.word f_true
	.word f_comma
	.word f_exit

	wdef then, "then", a_call, f_ifnez, ATTR_IMMED
	.word f_hereload
	.word f_swap
	.word f_store
	.word f_exit

	wdef begin, "begin", a_call, f_then, ATTR_IMMED
	.word f_hereload
	.word f_exit

	wdef until, "until", a_call, f_begin, ATTR_IMMED
	.word f_2lit
	.word n_0branch
	.word 7
	.word f_find
	.word f_comma
	.word f_comma
	.word f_exit

	wdef again, "again", a_call, f_until, ATTR_IMMED
	.word f_2lit
	.word n_branch
	.word 6
	.word f_find
	.word f_comma
	.word f_comma
	.word f_exit

	wdef ycnt, "ycnt", a_doconst, f_again, 0
	.word ycnt

	.equ TNP, (0 * ADDRSIZE)
	.equ TIP, (1 * ADDRSIZE)
	.equ TSS, (2 * ADDRSIZE)
	.equ TSP, (3 * ADDRSIZE)
	.equ TST, (4 * ADDRSIZE)
	.equ TRP, (5 * ADDRSIZE)

taskload:
	lw ip, TIP(up)
	lw ss, TSS(up)
	lw sp, TSP(up)
	lw st, TST(up)
	lw rp, TRP(up)
	ret

tasksave:
	sw ip, TIP(up)
	sw ss, TSS(up)
	sw sp, TSP(up)
	sw st, TST(up)
	sw rp, TRP(up)
	ret
	
	wdef yield, "yield", a_yield, f_ycnt, 0
a_yield:
	la wp, ycnt
	lw xp, 0(wp)
	addi xp, xp, 1
	sw xp, 0(wp)

	call tasksave
	lw up, TNP(up)
	call taskload
	
	next

	wdef UART1, "UART1", a_doconst, f_yield, 0
	.word UART1_BASE

	wdef UART_STATR, "UART_STATR", a_doconst, f_UART1, 0
	.word UART_STATR

	wdef UART_DATAR, "UART_DATAR", a_doconst, f_UART_STATR, 0
	.word UART_DATAR

	wdef UART_TC, "UART_TC", a_doconst, f_UART_DATAR, 0
	.word UART_TC

	wdef UART_RXNE, "UART_RXNE", a_doconst, f_UART_TC, 0
	.word UART_RXNE

	// (uart-base-addr) -- (flag)
	wdef uart_rxstat, "uart-rx?", a_call, f_UART_RXNE, 0
	.word f_UART_STATR
	.word f_add
	.word f_load
	.word f_UART_RXNE
	.word f_and
	.word f_nez
	.word f_exit

	// (uart-base-addr) -- (flag)
	wdef uart_txstat, "uart-tx?", a_call, f_uart_rxstat, 0
	.word f_UART_STATR
	.word f_add
	.word f_load
	.word f_UART_TC
	.word f_and
	.word f_nez
	.word f_exit

	// (uart-base-addr) -- (data)
	wdef uart_load, "uart@", a_call, f_uart_txstat, 0
	.word f_UART_DATAR
	.word f_add
	.word f_load
	.word f_exit

	// (data uart-base-addr) --
	wdef uart_store, "uart!", a_call, f_uart_load, 0
	.word f_UART_DATAR
	.word f_add
	.word f_store
	.word f_exit

	// (uart-base-addr) --
	wdef uart_rxwait, "uart-rxwait", a_call, f_uart_store, 0
1:
	.word f_yield
	.word f_dup
	.word f_uart_rxstat
	.word f_0branch
	.word 1b
	.word f_drop
	.word f_exit

	// (uart-base-addr) --
	wdef uart_txwait, "uart-txwait", a_call, f_uart_rxwait, 0
1:
	.word f_yield
	.word f_dup
	.word f_uart_txstat
	.word f_0branch
	.word 1b
	.word f_drop
	.word f_exit

	// (data uart-base-addr) --
	wdef uart_tx, "uart-tx", a_call, f_uart_txwait, 0
	.word f_dup
	.word f_uart_txwait
	.word f_uart_store
	.word f_exit

	// (uart-base-addr) -- (data)
	wdef uart_rx, "uart-rx", a_call, f_uart_tx, 0
	.word f_dup
	.word f_uart_rxwait
	.word f_uart_load
	.word f_exit

	wdef uart1_tx, "uart1-tx", a_call, f_uart_rx, 0
	.word f_UART1
	.word f_uart_tx
	.word f_exit

	wdef uart1_rx, "uart1-rx", a_call, f_uart1_tx, 0
	.word f_UART1
	.word f_uart_rx
	.word f_exit

	wdef 16load, "16@", a_call, f_uart1_rx, 0
	.word f_dup
	.word f_VSPI_FLASH_BASE
	.word f_VSPI_FLASH_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VSPI_FLASH_BASE
	.word f_sub
	.word f_spi_flash_read2b
	.word f_exit
1:
	.word f_dup
	.word f_VI2C_FT24_BASE
	.word f_VI2C_FT24_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VI2C_FT24_BASE
	.word f_sub
	.word f_i2c_ft24_read2b
	.word f_exit
1:		
	.word f_mem16load
	.word f_exit

	wdef mem16load, "mem16@", a_mem16load, f_16load, 0
a_mem16load:
	call dpop
	lhu tos, 0(tos)
	call dpush
	next

	wdef 16store, "16!", a_call, f_mem16load, 0
	.word f_dup
	.word f_VSPI_FLASH_BASE
	.word f_VSPI_FLASH_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VSPI_FLASH_BASE
	.word f_sub
	.word f_spi_flash_write2b
	.word f_exit
1:
	.word f_dup
	.word f_VI2C_FT24_BASE
	.word f_VI2C_FT24_END
	.word f_within
	.word f_0branch
	.word 1f
	.word f_VI2C_FT24_BASE
	.word f_sub
	.word f_i2c_ft24_write2b
	.word f_exit
1:
	.word f_mem16store
	.word f_exit

	wdef mem16store, "mem16!", a_mem16store, f_16store, 0
a_mem16store:
	call dpop
	mv wp, tos
	call dpop
	sh tos, 0(wp)
	next

	.equ IWDG_BASE, 0x40003000
	wdef IWDG, "IWDG", a_doconst, f_16store, 0
	.word IWDG_BASE

	.equ IWDG_CTLR, 0x00
	wdef IWDG_CTLR, "IWDG_CTLR", a_doconst, f_IWDG, 0
	.word IWDG_CTLR

	.equ IWDG_PSCR, 0x04
	wdef IWDG_PSCR, "IWDG_PSCR", a_doconst, f_IWDG_CTLR, 0
	.word IWDG_PSCR

	.equ IWDG_RLDR, 0x08
	wdef IWDG_RLDR, "IWDG_RLDR", a_doconst, f_IWDG_PSCR, 0
	.word IWDG_RLDR

	.equ IWDG_STATR, 0x0C
	wdef IWDG_STATR, "IWDG_STATR", a_doconst, f_IWDG_RLDR, 0
	.word IWDG_STATR

	.equ IWDG_KEY_UNLOCK, 0x5555
	.equ IWDG_KEY_FEED, 0xAAAA
	.equ IWDG_KEY_START, 0xCCCC

	wdef IWDG_KEY_UNLOCK, "IWDG_KEY_UNLOCK", a_doconst, f_IWDG_STATR, 0
	.word IWDG_KEY_UNLOCK
	wdef IWDG_KEY_FEED, "IWDG_KEY_FEED", a_doconst, f_IWDG_KEY_UNLOCK, 0
	.word IWDG_KEY_FEED
	wdef IWDG_KEY_START, "IWDG_KEY_START", a_doconst, f_IWDG_KEY_FEED, 0
	.word IWDG_KEY_START

	wdef iwdg_wait, "iwdg-wait", a_call, f_IWDG_KEY_START, 0
1:
	.word f_IWDG
	.word f_IWDG_STATR
	.word f_add
	.word f_16load
	.word f_3
	.word f_and
	.word f_eqz
	.word f_0branch
	.word 1b
	.word f_exit

	wdef iwdg_on, "iwdg-on", a_call, f_iwdg_wait, 0
	.word f_IWDG_KEY_UNLOCK
	.word f_IWDG
	.word f_IWDG_CTLR
	.word f_add
	.word f_16store

	.word f_iwdg_wait

	.word f_lit
	.word 0xFFFF
	.word f_IWDG
	.word f_IWDG_RLDR
	.word f_add
	.word f_16store

	.word f_iwdg_wait

	.word f_lit
	.word 3
	.word f_IWDG
	.word f_IWDG_PSCR
	.word f_add
	.word f_16store

	.word f_iwdg_wait

	.word f_IWDG_KEY_START
	.word f_IWDG
	.word f_IWDG_CTLR
	.word f_add
	.word f_16store

	.word f_iwdg_wait

	.word f_exit

	wdef iwdg_feed, "iwdg-feed", a_call, f_iwdg_on, 0
	.word f_IWDG_KEY_FEED
	.word f_IWDG
	.word f_IWDG_CTLR
	.word f_add
	.word f_16store
	.word f_exit

	.macro defconst name, value, link
	wdef \name, \name, a_doconst, \link, 0
	.word \value
	.endm
	
	.equ SPI1_BASE, 0x40013000
	defconst SPI1, SPI1_BASE, f_iwdg_feed
	.equ SPI_CTLR1, 0x00
	defconst SPI_CTLR1, SPI_CTLR1, f_SPI1
	.equ SPI_SSM_SOFT, (1 << 9)
	defconst SPI_SSM_SOFT, SPI_SSM_SOFT, f_SPI_CTLR1
	.equ SPI_SSI_HIGH, (1 << 8)
	defconst SPI_SSI_HIGH, SPI_SSI_HIGH, f_SPI_SSM_SOFT
	.equ SPI_MASTER, (1 << 2)
	defconst SPI_MASTER, SPI_MASTER, f_SPI_SSI_HIGH
	.equ SPI_ENABLE, (1 << 6)
	defconst SPI_ENABLE, SPI_ENABLE, f_SPI_MASTER
	.equ SPI_DATAR, 0xC
	defconst SPI_DATAR, SPI_DATAR, f_SPI_ENABLE
	.equ SPI_BAUD_6M, (0x3 << 3)
	defconst SPI_BAUD_6M, SPI_BAUD_6M, f_SPI_DATAR

	// (spi-base-addr) --
	wdef spim_init, "spim-init", a_call, f_SPI_BAUD_6M, 0
	.word f_SPI_CTLR1
	.word f_add
	.word f_SPI_SSM_SOFT
	.word f_SPI_SSI_HIGH
	.word f_or
	.word f_SPI_MASTER
	.word f_or
	.word f_SPI_BAUD_6M
	.word f_or
	.word f_SPI_ENABLE
	.word f_or
	.word f_swap
	.word f_16store
	.word f_exit

	// (data spi-base-addr)
	wdef spi_store, "spi!", a_call, f_spim_init, 0
	.word f_SPI_DATAR
	.word f_add
	.word f_16store
	.word f_exit

	// (spi-base-addr) -- (data)
	wdef spi_load, "spi@", a_call, f_spi_store, 0
	.word f_SPI_DATAR
	.word f_add
	.word f_16load
	.word f_exit

	.equ SPI_STATR, 0x08
	defconst SPI_STATR, SPI_STATR, f_spi_load

	.equ SPI_BUSY, (1 << 7)
	defconst SPI_BUSY, SPI_BUSY, f_SPI_STATR

	.equ SPI_TXE, (1 << 1)
	defconst SPI_TXE, SPI_TXE, f_SPI_BUSY

	.equ SPI_RXNE, (1 << 0)
	defconst SPI_RXNE, SPI_RXNE, f_SPI_TXE

	// (spi-base-addr) -- (flag)
	wdef spi_busy, "spi-busy", a_call, f_SPI_RXNE, 0
	.word f_SPI_STATR
	.word f_add
	.word f_16load
	.word f_SPI_BUSY
	.word f_and
	.word f_nez
	.word f_exit

	// (spi-base-addr) -- (flag)
	wdef spi_txstat, "spi-tx?", a_call, f_spi_busy, 0
	.word f_SPI_STATR
	.word f_add
	.word f_16load
	.word f_SPI_TXE
	.word f_and
	.word f_nez
	.word f_exit

	// (spi-base-addr) -- (flag)
	wdef spi_rxstat, "spi-rx?", a_call, f_spi_txstat, 0
	.word f_SPI_STATR
	.word f_add
	.word f_16load
	.word f_SPI_RXNE
	.word f_and
	.word f_nez
	.word f_exit

	// (spi-base-addr)
	wdef spi_txwait, "spi-txwait", a_call, f_spi_rxstat, 0
1:
	.word f_yield
	.word f_dup
	.word f_spi_txstat
	.word f_0branch
	.word 1b
	.word f_drop
	.word f_exit

	// (spi-base-addr)
	wdef spi_rxwait, "spi-rxwait", a_call, f_spi_txwait, 0
1:
	.word f_yield
	.word f_dup
	.word f_spi_rxstat
	.word f_0branch
	.word 1b
	.word f_drop
	.word f_exit

	// (data spi-base-addr)
	wdef spi_tx, "spi-tx", a_call, f_spi_rxwait, 0
	.word f_dup
	.word f_spi_txwait
	.word f_spi_store
	.word f_exit

	// (spi-base-addr) -- (data)
	wdef spi_rx, "spi-rx", a_call, f_spi_tx, 0
	.word f_dup
	.word f_spi_rxwait
	.word f_spi_load
	.word f_exit

	// (txdata spi-base-addr) -- (rxdata)
	wdef spi_tx_rx, "spi-tx-rx", a_call, f_spi_rx, 0
	.word f_dup
	.word f_tor
	.word f_spi_tx
	.word f_fromr
	.word f_spi_rx
	.word f_exit

	defconst GPIOA, GPIOA_BASE, f_spi_tx_rx
	defconst GPIO_OUTDR, GPIO_OUTDR, f_GPIOA

	# (mask addr)
	wdef membic, "membic", a_call, f_GPIO_OUTDR, 0
	.word f_dup
	.word f_load
	.word f_rot
	.word f_bic
	.word f_swap
	.word f_store
	.word f_exit

	# (mask addr)
	wdef memor, "memor", a_call, f_membic, 0
	.word f_dup
	.word f_load
	.word f_rot
	.word f_or
	.word f_swap
	.word f_store
	.word f_exit

	wdef gpio_outset, "gpio-outset", a_call, f_memor, 0
	.word f_GPIO_OUTDR
	.word f_add
	.word f_memor
	.word f_exit

	// (mask gpio-base-addr)
	wdef gpio_outclr, "gpio-outclr", a_call, f_gpio_outset, 0
	.word f_GPIO_OUTDR
	.word f_add
	.word f_membic
	.word f_exit

	wdef func_spi_flash_cs "func-spi-flash-cs", a_doconst, f_gpio_outclr, 0
	.word func_spi_flash_cs

	wdef spi_flash_cs_pa2, "spi-flash-cs-pa2", a_call, f_func_spi_flash_cs, 0
	.word f_0branch
	.word 1f
	.word f_1
	.word f_2
	.word f_lshift
	.word f_GPIOA
	.word f_gpio_outclr
	.word f_exit
1:
	.word f_1
	.word f_2
	.word f_lshift
	.word f_GPIOA
	.word f_gpio_outset
	.word f_exit

	// (flag)
	wdef spi_flash_cs, "spi-flash-cs", a_call, f_spi_flash_cs_pa2, 0
	.word f_func_spi_flash_cs
	.word f_loadexecute
	.word f_exit

	wdef func_spi_flash_bus, "func-spi-flash-bus", a_doconst, f_spi_flash_cs, 0
	.word func_spi_flash_bus

	wdef spi_flash_bus, "spi-flash-bus", a_call, f_func_spi_flash_bus, 0
	.word f_func_spi_flash_bus
	.word f_loadexecute
	.word f_exit

	wdef spi_flash_send_start, "spi-flash-send-start", a_call, f_spi_flash_bus, 0
	.word f_true
	.word f_spi_flash_cs
	.word f_exit

	wdef spi_flash_send_end, "spi-flash-send-end", a_call, f_spi_flash_send_start, 0
	.word f_false
	.word f_spi_flash_cs
	.word f_exit

	defconst W25QXX_READ_JEDEC_ID, 0x9F, f_spi_flash_send_end

	// 0xEF 0x40 0x16 : w25q32
	// () -- (manufacturer-id device-id-0-7 device-id-8-15)
	wdef spi_flash_id, "spi-flash-id", a_call, f_W25QXX_READ_JEDEC_ID, 0
	.word f_spi_flash_send_start
	
	.word f_W25QXX_READ_JEDEC_ID
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	// Manufacturer ID
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx

	// Device ID
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx

	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx

	.word f_spi_flash_send_end
	.word f_exit

	defconst W25QXX_WRITE_ENABLE, 0x06, f_spi_flash_id
	defconst W25QXX_WRITE_DISABLE, 0x04, f_W25QXX_WRITE_ENABLE

	wdef spi_flash_wren, "spi-flash-wren", a_call, f_W25QXX_WRITE_DISABLE, 0
	.word f_spi_flash_send_start

	.word f_W25QXX_WRITE_ENABLE
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_spi_flash_send_end
	.word f_exit

	wdef spi_flash_wrdis, "spi-flash-wrdis", a_call, f_spi_flash_wren, 0
	.word f_spi_flash_send_start

	.word f_W25QXX_WRITE_DISABLE
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_spi_flash_send_end
	.word f_exit

	.equ W25QXX_SR1_READ, 0x05
	defconst W25QXX_SR1_READ, W25QXX_SR1_READ, f_spi_flash_wrdis
	
	wdef spi_flash_sr1load, "spi-flash-sr1@", a_call, f_W25QXX_SR1_READ ,0
	.word f_spi_flash_send_start

	.word f_W25QXX_SR1_READ
	.word f_spi_flash_bus
	.word f_spi_tx_rx

	.word f_spi_flash_bus
	.word f_spi_tx_rx

	.word f_spi_flash_send_end
	.word f_exit

	# (addr3b)
	wdef spi_flash_addrsend, "spi-flash-addrsend", a_call, f_spi_flash_sr1load, 0
	.word f_dup
	.word f_lit
	.word 16
	.word f_rshift
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_dup
	.word f_8
	.word f_rshift
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_exit

	.equ W25QXX_READ_DATA, 0x03
	defconst W25QXX_READ_DATA, W25QXX_READ_DATA, f_spi_flash_addrsend

	# (addr3b)
	wdef spi_flash_read_start, "spi-flash-read-start", a_call, f_W25QXX_READ_DATA, 0
	.word f_spi_flash_send_start
	.word f_W25QXX_READ_DATA
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop
	.word f_spi_flash_addrsend
	.word f_exit

	wdef spi_flash_read_end, "spi-flash-read-end", a_call, f_spi_flash_read_start, 0
	.word f_spi_flash_send_end
	.word f_exit

	# (addr3b) -- (data1b)
	wdef spi_flash_read1b, "spi-flash-read1b", a_call, f_spi_flash_read_end, 0
	.word f_spi_flash_read_start
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_spi_flash_read_end
	.word f_exit

	# (addr3b) -- (data2b)
	wdef spi_flash_read2b, "spi-flash-read2b", a_call, f_spi_flash_read1b, 0
	.word f_spi_flash_read_start
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_8
	.word f_lshift
	.word f_or
	.word f_spi_flash_read_end
	.word f_exit

	# (addr3b) -- (data4b)
	wdef spi_flash_read4b, "spi-flash-read4b", a_call, f_spi_flash_read2b, 0
	.word f_spi_flash_read_start

	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_lit
	.word 8
	.word f_lshift
	.word f_or
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_lit
	.word 16
	.word f_lshift
	.word f_or
	.word f_0
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_lit
	.word 24
	.word f_lshift
	.word f_or
	
	.word f_spi_flash_read_end
	.word f_exit

	.equ W25QXX_BUSY, (1 << 0)
	defconst W25QXX_BUSY, W25QXX_BUSY, f_spi_flash_read4b

	wdef spi_flash_busy, "spi-flash-busy", a_call, f_W25QXX_BUSY, 0
	.word f_spi_flash_sr1load
	.word f_W25QXX_BUSY
	.word f_and
	.word f_nez
	.word f_exit

	wdef spi_flash_wait, "spi-flash-wait", a_call, f_spi_flash_busy, 0
1:
	.word f_yield
	.word f_spi_flash_busy
	.word f_invert
	.word f_0branch
	.word 1b
	.word f_exit

	.equ W25QXX_PROGRAM, 0x02
	defconst W25QXX_PROGRAM, W25QXX_PROGRAM, f_spi_flash_wait

	# (addr3b)
	wdef spi_flash_write_start, "spi-flash-write-start", a_call, f_W25QXX_PROGRAM, 0
	.word f_spi_flash_wren
	.word f_spi_flash_wait
	.word f_spi_flash_send_start
	.word f_W25QXX_PROGRAM
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop
	.word f_spi_flash_addrsend
	.word f_exit

	wdef spi_flash_write_end, "spi-flash-write-end", a_call, f_spi_flash_write_start, 0
	.word f_spi_flash_send_end
	.word f_spi_flash_wait
	.word f_spi_flash_wrdis
	.word f_exit

	# (data1b addr3b)
	wdef spi_flash_write1b, "spi-flash-write1b", a_call, f_spi_flash_write_end, 0
	.word f_spi_flash_write_start
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop
	.word f_spi_flash_write_end
	.word f_exit

	# (data2b addr3b)
	wdef spi_flash_write2b, "spi-flash-write2b", a_call, f_spi_flash_write1b, 0
	.word f_spi_flash_write_start
	.word f_dup
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_8
	.word f_rshift
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_spi_flash_write_end
	.word f_exit

	# (data4b addr3b)
	wdef spi_flash_write4b, "spi-flash-write4b", a_call, f_spi_flash_write2b, 0
	.word f_spi_flash_write_start
	.word f_dup
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_dup
	.word f_8
	.word f_rshift
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_dup
	.word f_lit
	.word 16
	.word f_rshift
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_lit
	.word 24
	.word f_rshift
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_spi_flash_write_end
	.word f_exit

	# virtual memory map, for spi flash access
	defconst VSPI_FLASH_BASE, 0x20010000, f_spi_flash_write4b
	defconst VSPI_FLASH_SIZE, (4 * 1024 * 1024), f_VSPI_FLASH_BASE
	wdef VSPI_FLASH_END, "VSPI_FLASH_END", a_call, f_VSPI_FLASH_SIZE, 0
	.word f_VSPI_FLASH_BASE
	.word f_VSPI_FLASH_SIZE
	.word f_add
	.word f_exit

	wdef move, "move", a_call, f_VSPI_FLASH_END, 0
2:
	.word f_dup
	.word f_0branch
	.word 1f
	.word f_dec
	.word f_tor
	.word f_over
	.word f_load
	.word f_over
	.word f_store
	.word f_4
	.word f_add
	.word f_swap
	.word f_4
	.word f_add
	.word f_swap
	.word f_fromr
	.word f_branch
	.word 2b

1:
	.word f_drop
	.word f_2drop
	.word f_exit

	defconst W25QXX_ERASE4K, 0x20, f_move

	# (addr)
	wdef spi_flash_erase4k, "spi-flash-erase4k", a_call, f_W25QXX_ERASE4K, 0
	.word f_spi_flash_wren
	.word f_spi_flash_wait
	.word f_spi_flash_send_start

	.word f_W25QXX_ERASE4K
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_spi_flash_addrsend

	.word f_spi_flash_send_end
	.word f_spi_flash_wren
	.word f_spi_flash_wait
	.word f_exit

	defconst W25QXX_ERASEALL, 0xC7, f_spi_flash_erase4k

	wdef spi_flash_eraseall, "spi-flash-eraseall", a_call, f_W25QXX_ERASEALL, 0
	.word f_spi_flash_wren
	.word f_spi_flash_wait
	.word f_spi_flash_send_start

	.word f_W25QXX_ERASEALL
	.word f_spi_flash_bus
	.word f_spi_tx_rx
	.word f_drop

	.word f_spi_flash_send_end
	.word f_spi_flash_wren
	.word f_spi_flash_wait
	.word f_exit

	defconst ESIG_BASE, 0x1FFFF700, f_spi_flash_eraseall
	defconst ESIG_UID1, 0xE8, f_ESIG_BASE
	defconst ESIG_UID2, 0xEC, f_ESIG_UID1
	defconst ESIG_UID3, 0xF0, f_ESIG_UID2

	wdef chipuid, "chipuid", a_call, f_ESIG_UID3, 0
	.word f_ESIG_BASE
	.word f_ESIG_UID3
	.word f_add
	.word f_load

	.word f_ESIG_BASE
	.word f_ESIG_UID2
	.word f_add
	.word f_load

	.word f_ESIG_BASE
	.word f_ESIG_UID1
	.word f_add
	.word f_load

	.word f_exit

	defconst I2C2, 0x40005800, f_chipuid
	defconst I2C_CTLR1, 0x00, f_I2C2
	defconst I2C_CTLR2, 0x04, f_I2C_CTLR1
	defconst I2C_OADDR1, 0x08, f_I2C_CTLR2
	defconst I2C_OADDR2, 0x0C, f_I2C_OADDR1
	defconst I2C_DATAR, 0x10, f_I2C_OADDR2
	defconst I2C_STAR1, 0x14, f_I2C_DATAR
	defconst I2C_STAR2, 0x18, f_I2C_STAR1
	defconst I2C_CKCFGR, 0x1C, f_I2C_STAR2
	defconst I2C_RTR, 0x20, f_I2C_CKCFGR

	defconst I2C_FREQ_60M, 60, f_I2C_RTR
	defconst I2C_CCR_100K, 840, f_I2C_FREQ_60M
	wdef I2C_TRISE, "I2C_TRISE", a_call, f_I2C_CCR_100K, 0
	.word f_I2C_FREQ_60M
	.word f_inc
	.word f_exit

	defconst I2C_PE, (1 << 0), f_I2C_TRISE
	defconst I2C_ACK, (1 << 10), f_I2C_PE

	# (mask addr)
	wdef mem16bic, "mem16bic", a_call, f_I2C_PE, 0
	.word f_dup
	.word f_16load
	.word f_rot
	.word f_bic
	.word f_swap
	.word f_16store
	.word f_exit

	# (mask addr)
	wdef mem16or, "mem16or", a_call, f_mem16bic, 0
	.word f_dup
	.word f_16load
	.word f_rot
	.word f_or
	.word f_swap
	.word f_16store
	.word f_exit

	# (i2c-base-addr)
	wdef i2cm_init, "i2cm-init", a_call, f_mem16or, 0
	.word f_I2C_FREQ_60M
	.word f_over
	.word f_I2C_CTLR2
	.word f_add
	.word f_16store

	.word f_dup
	.word f_i2c_dis

	.word f_I2C_TRISE
	.word f_over
	.word f_I2C_RTR
	.word f_add
	.word f_16store

	.word f_I2C_CCR_100K
	.word f_over
	.word f_I2C_CKCFGR
	.word f_add
	.word f_16store

	.word f_dup
	.word f_i2c_en

	.word f_i2c_ack_en

	.word f_exit

	# (i2c-base-addr)
	wdef i2c_en, "i2c-en", a_call, f_i2cm_init, 0
	.word f_I2C_CTLR1
	.word f_add
	.word f_I2C_PE
	.word f_swap
	.word f_mem16or
	.word f_exit

	wdef i2c_dis, "i2c-dis", a_call, f_i2c_en, 0
	.word f_I2C_CTLR1
	.word f_add
	.word f_I2C_PE
	.word f_swap
	.word f_mem16bic
	.word f_exit

	wdef i2c_ack_en, "i2c-ack-en", a_call, f_i2c_dis, 0
	.word f_I2C_CTLR1
	.word f_add
	.word f_I2C_ACK
	.word f_swap
	.word f_mem16or
	.word f_exit

	wdef i2c_ack_dis, "i2c-ack-dis", a_call, f_i2c_ack_en, 0
	.word f_I2C_CTLR1
	.word f_add
	.word f_I2C_ACK
	.word f_swap
	.word f_mem16bic
	.word f_exit

	defconst I2C_START, (1 << 8), f_i2c_ack_dis
	defconst I2C_STOP,  (1 << 9), f_I2C_START

	wdef i2c_start, "i2c-start", a_call, f_I2C_STOP, 0
	.word f_I2C_CTLR1
	.word f_add
	.word f_I2C_START
	.word f_swap
	.word f_mem16or
	.word f_exit

	wdef i2c_stop, "i2c-stop", a_call, f_i2c_start, 0
	.word f_I2C_CTLR1
	.word f_add
	.word f_I2C_STOP
	.word f_swap
	.word f_mem16or
	.word f_exit

	wdef i2c_statload, "i2c-stat@", a_call, f_i2c_stop, 0
	.word f_dup
	.word f_I2C_STAR1
	.word f_add
	.word f_16load
	.word f_swap
	.word f_I2C_STAR2
	.word f_add
	.word f_16load
	.word f_lit
	.word 16
	.word f_lshift
	.word f_or
	.word f_exit

	wdef i2c_statstore, "i2c-stat!", a_call, f_i2c_statload, 0
	.word f_2dup
	.word f_I2C_STAR1
	.word f_add
	.word f_16store
	.word f_I2C_STAR2
	.word f_add
	.word f_swap
	.word f_lit
	.word 16
	.word f_rshift
	.word f_swap
	.word f_16store
	.word f_exit

	wdef i2c_statrst, "i2c-statrst", a_call, f_i2c_statstore, 0
	.word f_0
	.word f_swap
	.word f_i2c_statstore
	.word f_exit

	.equ I2C_BUSY, (1 << (1 + 16))
	defconst I2C_BUSY, I2C_BUSY, f_i2c_statrst

	wdef i2c_bus_busy, "i2c-bus-busy", a_call, f_I2C_BUSY, 0
	.word f_i2c_statload
	.word f_I2C_BUSY
	.word f_and
	.word f_nez
	.word f_exit

	wdef i2c_bus_wait, "i2c-bus-wait", a_call, f_i2c_bus_busy, 0
1:
	.word f_yield
	.word f_dup
	.word f_i2c_bus_busy
	.word f_invert
	.word f_0branch
	.word 1b
	.word f_drop
	.word f_exit

	.equ I2C_MSL, (1 << (0 + 16))
	.equ I2C_SB,  (1 << 0)
	.equ I2C_COMM_START, (I2C_BUSY | I2C_MSL | I2C_SB)
	defconst I2C_COMM_START, I2C_COMM_START, f_i2c_bus_wait
	
	wdef i2c_start_stat, "i2c-start-stat", a_call, f_I2C_COMM_START, 0
	.word f_i2c_statload
	.word f_I2C_COMM_START
	.word f_and
	.word f_I2C_COMM_START
	.word f_equ
	.word f_exit

	wdef i2c_start_wait, "i2c-start-wait", a_call, f_i2c_start_stat, 0
1:
	.word f_yield
	.word f_dup
	.word f_i2c_start_stat
	.word f_0branch
	.word 1b
	.word f_drop
	.word f_exit

	defconst I2C_ADDR_7BIT_MODE, 0x4000, f_i2c_start_wait

	wdef i2c_addr_set, "i2c-addr-set", a_call, f_I2C_ADDR_7BIT_MODE, 0
	.word f_swap
	.word f_I2C_ADDR_7BIT_MODE
	.word f_or
	.word f_swap
	.word f_I2C_OADDR1
	.word f_add
	.word f_16store
	.word f_exit

	wdef i2c_cstore, "i2c-c!", a_call, f_i2c_addr_set, 0
	.word f_swap
	.word f_lit
	.word 0xFF
	.word f_and
	.word f_swap
	.word f_I2C_DATAR
	.word f_add
	.word f_16store
	.word f_exit

	defconst I2C_ADDR_MTOK, 0x00070082, f_i2c_cstore
	defconst I2C_AF, (1 << 10), f_I2C_ADDR_MTOK

	# (i2c-base-addr) -- (flag)
	wdef i2c_mtaddr_wait, "i2c-mtaddr-wait", a_call, f_I2C_AF, 0
2:
	.word f_yield
	.word f_dup
	.word f_i2c_statload

	.word f_dup
	.word f_I2C_AF
	.word f_and
	.word f_0branch
	.word 1f
	.word f_2drop
	.word f_false
	.word f_exit
1:
	.word f_I2C_ADDR_MTOK
	.word f_and
	.word f_I2C_ADDR_MTOK
	.word f_equ
	.word f_0branch
	.word 2b
	.word f_drop
	.word f_true
	.word f_exit

	defconst I2C_ADDR_MROK, 0x00030002, f_i2c_mtaddr_wait

	# (i2c-base-addr) -- (flag)
	wdef i2c_mraddr_wait, "i2c-mraddr-wait", a_call, f_i2c_mtaddr_wait, 0
2:
	.word f_yield
	.word f_dup
	.word f_i2c_statload

	.word f_dup
	.word f_I2C_AF
	.word f_and
	.word f_0branch
	.word 1f
	.word f_2drop
	.word f_false
	.word f_exit
1:
	.word f_I2C_ADDR_MROK
	.word f_and
	.word f_I2C_ADDR_MROK
	.word f_equ
	.word f_0branch
	.word 2b
	.word f_drop
	.word f_true
	.word f_exit

	wdef i2c_mtscan, "i2c-mtscan", a_call, f_i2c_mraddr_wait, 0
	.word f_dup
	.word f_i2c_bus_wait
	.word f_dup
	.word f_i2c_start
	.word f_dup
	.word f_i2c_start_wait

	.word f_swap
	.word f_over
	.word f_i2c_cstore

	.word f_dup
	.word f_i2c_mtaddr_wait
	.word f_swap
	.word f_i2c_stop
	.word f_exit

	wdef i2c_mrscan, "i2c-mrscan", a_call, f_i2c_mtscan, 0
	.word f_dup
	.word f_i2c_bus_wait
	.word f_dup
	.word f_i2c_start
	.word f_dup
	.word f_i2c_start_wait

	.word f_swap
	.word f_1
	.word f_or
	.word f_over
	.word f_i2c_cstore

	.word f_dup
	.word f_i2c_mraddr_wait
	.word f_swap

	.word f_dup
	.word f_i2c_ack_dis
	.word f_i2c_stop
	.word f_exit

	defconst I2C_MTX1B_OK, 0x00070084, f_i2c_mrscan

	wdef i2c_mtx_wait, "i2c-mtx-wait", a_call, f_I2C_MTX1B_OK, 0
2:
	.word f_yield
	.word f_dup
	.word f_i2c_statload

	.word f_dup
	.word f_I2C_AF
	.word f_and
	.word f_0branch
	.word 1f
	.word f_2drop
	.word f_false
	.word f_exit
1:
	.word f_I2C_MTX1B_OK
	.word f_and
	.word f_I2C_MTX1B_OK
	.word f_equ
	.word f_0branch
	.word 2b
	.word f_drop
	.word f_true
	.word f_exit

	defconst I2C_RXNE, (1 << 6), f_i2c_mtx_wait
	
	wdef i2c_mrx_wait, "i2c-mrx-wait", a_call, f_I2C_RXNE, 0
1:
	.word f_yield
	.word f_dup
	.word f_i2c_statload

	.word f_I2C_RXNE
	.word f_and
	.word f_nez
	.word f_0branch
	.word 1b
	.word f_drop
	.word f_true
	.word f_exit

	wdef i2c_cload, "i2c-c@", a_call, f_i2c_mrx_wait, 0
	.word f_I2C_DATAR
	.word f_add
	.word f_16load
	.word f_exit

	wdef func_i2c_ft24_bus, "func-i2c-ft24-bus", a_doconst, f_i2c_cload, 0
	.word func_i2c_ft24_bus

	defconst 0xA0, 0xA0, f_func_i2c_ft24_bus

	wdef func_i2c_ft24_addr, "func-i2c-ft24-addr", a_doconst, f_0xA0, 0
	.word func_i2c_ft24_addr

	wdef i2c_ft24_bus, "i2c-ft24-bus", a_call, f_func_i2c_ft24_addr, 0
	.word f_func_i2c_ft24_bus
	.word f_loadexecute
	.word f_exit

	wdef i2c_ft24_addr, "i2c-ft24-addr", a_call, f_i2c_ft24_bus, 0
	.word f_func_i2c_ft24_addr
	.word f_loadexecute
	.word f_exit

	# (addr2b) -- (data1b)
	wdef i2c_ft24_r1b, "i2c-ft24-r1b", a_call, f_i2c_ft24_addr, 0
	.word f_i2c_ft24_bus
	.word f_dup
	.word f_i2c_bus_wait
	.word f_dup
	.word f_i2c_start
	.word f_dup
	.word f_i2c_start_wait

	# send dev addr: send
	.word f_i2c_ft24_addr
	.word f_over
	.word f_i2c_cstore
	.word f_i2c_mtaddr_wait
	.word f_0branch
	.word ft24_r1b_fail

	# send data address bit8~15
	.word f_dup
	.word f_8
	.word f_rshift
	.word f_i2c_ft24_bus
	.word f_i2c_cstore
	.word f_i2c_ft24_bus
	.word f_i2c_mtx_wait
	.word f_0branch
	.word ft24_r1b_fail

	# send data address bit0~7
	.word f_dup
	.word f_i2c_ft24_bus
	.word f_i2c_cstore
	.word f_i2c_ft24_bus
	.word f_i2c_mtx_wait
	.word f_0branch
	.word ft24_r1b_fail

	.word f_i2c_ft24_bus
	.word f_dup
	.word f_i2c_start
	.word f_dup
	.word f_i2c_start_wait

	# send dev addr: recv
	.word f_i2c_ft24_addr
	.word f_1
	.word f_or
	.word f_over
	.word f_i2c_cstore
	.word f_i2c_mraddr_wait
	.word f_0branch
	.word ft24_r1b_fail

	.word f_i2c_ft24_bus
	.word f_dup
	.word f_i2c_ack_dis
	.word f_i2c_mrx_wait
	.word f_0branch
	.word ft24_r1b_fail

	.word f_i2c_ft24_bus
	.word f_i2c_cload
	.word f_nip

	.word f_i2c_ft24_bus
	.word f_dup
	.word f_i2c_stop
	.word f_i2c_ack_en
	.word f_exit

ft24_r1b_fail:
	.word f_drop
	.word f_i2c_ft24_bus
	.word f_dup
	.word f_i2c_stop
	.word f_dup
	.word f_i2c_statrst
	.word f_i2c_ack_en
	.word f_true
	.word f_lit
	.word 0xFF
	.word f_bic
	.word f_exit

	# (data addr2b) -- (flag)
	wdef i2c_ft24_w1b, "i2c-ft24-w1b", a_call, f_i2c_ft24_r1b, 0
	.word f_i2c_ft24_bus
	.word f_dup
	.word f_i2c_bus_wait
	.word f_dup
	.word f_i2c_start
	.word f_dup
	.word f_i2c_start_wait

	# send dev addr: send
	.word f_i2c_ft24_addr
	.word f_over
	.word f_i2c_cstore
	.word f_i2c_mtaddr_wait
	.word f_0branch
	.word ft24_w1b_fail

	# send data address bit8~15
	.word f_dup
	.word f_8
	.word f_rshift
	.word f_i2c_ft24_bus
	.word f_i2c_cstore
	.word f_i2c_ft24_bus
	.word f_i2c_mtx_wait
	.word f_0branch
	.word ft24_w1b_fail

	# send data address bit0~7
	.word f_dup
	.word f_i2c_ft24_bus
	.word f_i2c_cstore
	.word f_i2c_ft24_bus
	.word f_i2c_mtx_wait
	.word f_0branch
	.word ft24_w1b_fail

	# send data bit0~7
	.word f_over
	.word f_i2c_ft24_bus
	.word f_i2c_cstore
	.word f_i2c_ft24_bus
	.word f_i2c_mtx_wait
	.word f_0branch
	.word ft24_w1b_fail

	.word f_i2c_ft24_bus
	.word f_i2c_stop
	.word f_2drop
	.word f_true
	.word f_exit

ft24_w1b_fail:
	.word f_2drop
	.word f_i2c_ft24_bus
	.word f_dup
	.word f_i2c_statrst
	.word f_i2c_stop
	.word f_false
	.word f_exit

	# virtual memory map, for i2c ft24c32 eeprom access
	wdef VI2C_FT24_BASE, "VI2C_FT24_BASE", a_call, f_i2c_ft24_w1b, 0
	.word f_VSPI_FLASH_END
	.word f_aligned
	.word f_exit

	defconst VI2C_FT24_SIZE, ((32 * 1024) / 8), f_VI2C_FT24_BASE

	wdef VI2C_FT24_END, "VI2C_FT24_END", a_call, f_VI2C_FT24_SIZE, 0
	.word f_VI2C_FT24_BASE
	.word f_VI2C_FT24_SIZE
	.word f_add
	.word f_exit

	wdef i2c_ft24_read1b, "i2c-ft24-read1b", a_call, f_VI2C_FT24_END, 0
2:
	.word f_dup
	.word f_i2c_ft24_r1b
	.word f_dup
	.word f_lit
	.word (1 << 8)
	.word f_and
	.word f_nez
	.word f_0branch
	.word 1f
	.word f_drop
	.word f_branch
	.word 2b
1:
	.word f_nip
	.word f_exit

	wdef i2c_ft24_write1b, "i2c-ft24-write1b", a_call, f_i2c_ft24_read1b, 0
1:
	.word f_2dup
	.word f_i2c_ft24_w1b
	.word f_0branch
	.word 1b
	.word f_2drop
	.word f_exit

	wdef i2c_ft24_read2b, "i2c-ft24-read2b", a_call, f_i2c_ft24_write1b, 0
	.word f_dup
	.word f_i2c_ft24_read1b
	.word f_swap
	.word f_inc
	.word f_i2c_ft24_read1b
	.word f_8
	.word f_lshift
	.word f_or
	.word f_exit

	wdef i2c_ft24_write2b, "i2c-ft24-write2b", a_call, f_i2c_ft24_read2b, 0
	.word f_2dup
	.word f_i2c_ft24_write1b
	.word f_inc
	.word f_swap
	.word f_8
	.word f_rshift
	.word f_swap
	.word f_i2c_ft24_write1b
	.word f_exit

	wdef i2c_ft24_read4b, "i2c-ft24-read4b", a_call, f_i2c_ft24_write2b, 0
	.word f_dup
	.word f_i2c_ft24_read2b
	.word f_swap
	.word f_2
	.word f_add
	.word f_i2c_ft24_read2b
	.word f_lit
	.word 16
	.word f_lshift
	.word f_or
	.word f_exit

	wdef i2c_ft24_write4b, "i2c-ft24-write4b", a_call, f_i2c_ft24_read4b, 0
	.word f_2dup
	.word f_i2c_ft24_write2b
	.word f_2
	.word f_add
	.word f_swap
	.word f_lit
	.word 16
	.word f_rshift
	.word f_swap
	.word f_i2c_ft24_write2b
	.word f_exit

	wdef interpret, "interpret", a_call, f_i2c_ft24_write4b, 0
	.word f_token
	.word f_tib
	.word f_toinload

	.word f_dup
	.word f_0branch
	.word interpret_exit

	.word f_2dup
	.word f_find
	.word f_dup
	.word f_0branch
	.word interpret_noword
	.word f_dup
	.word f_wisimmed
	.word f_0branch
	.word 1f
	.word f_branch
	.word interpret_execute
1:
	.word f_iscomp
	.word f_0branch
	.word interpret_execute
	.word f_comma
	.word f_branch
	.word interpret_exit

interpret_execute:
	.word f_nip
	.word f_nip
	.word f_execute
	.word f_sschk
	.word f_0branch
	.word 1f
	.word f_exit
1:
	.word f_2lit
	.word _msg_dstk_err
	.word _msg_end_dstk_err - _msg_dstk_err
	.word f_type
	.word f_cr
	.word f_ssrst
	.word f_sprst
	.word f_exit

_msg_dstk_err:
	.ascii " data stack error"
_msg_end_dstk_err:
	.p2align 2, 0xFF
	
interpret_noword:
	.word f_drop
	.word f_2dup
	.word f_isnumber
	.word f_0branch
	.word interpret_notfound
	.word f_number
	.word f_iscomp
	.word f_0branch
	.word 1f
	.word f_2lit
	.word n_lit
	.word 3
	.word f_find
	.word f_comma
	.word f_comma
1:
	.word f_exit
interpret_notfound:
	.word f_type
	.word f_2lit
	.word _msg_notfound
	.word _msg_end_notfound - _msg_notfound
	.word f_type
	.word f_cr
	.word f_exit
interpret_exit:
	.word f_2drop
	.word f_exit	

_msg_notfound:
	.ascii " not found"
_msg_end_notfound:
	.p2align 2, 0xFF

	.p2align 2, 0xFF
boot_human:
	.macro display label, msg
		.word f_2lit
		.word _str_\label
		.word _str_end_\label - _str_\label
		.word f_type
		.word f_branch
		.word 1f
	_str_\label:
		.ascii "\msg"
	_str_end_\label:
		.p2align 2, 0xFF
	1:
	.endm

	display test_0branch, "0branch."
	.word f_branch
	.word 1f
	.word f_fail
1:

	.word f_0
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_faileqz
	.word f_dzchk

	.word f_0
	.word f_1
	.word f_failequ
	.word f_dzchk

	display test_xor, "xor."
	.word f_0
	.word f_0
	.word f_xor
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_1
	.word f_xor
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_0
	.word f_xor
	.word f_faileqz
	.word f_dzchk

	.word f_0
	.word f_1
	.word f_xor
	.word f_faileqz
	.word f_dzchk

	display test_add, "+."
	.word f_0
	.word f_1
	.word f_add
	.word f_1
	.word f_failneq
	.word f_dzchk

	.word f_1
	.word f_0
	.word f_add
	.word f_1
	.word f_failneq
	.word f_dzchk

	.word f_0
	.word f_0
	.word f_add
	.word f_failnez
	.word f_dzchk

	.word f_2lit
	.word -1
	.word 1
	.word f_add
	.word f_failnez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word -1
	.word f_add
	.word f_failnez
	.word f_dzchk

	.word f_2lit
	.word 1
	.word -1
	.word f_add
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_1
	.word f_add
	.word f_2
	.word f_failneq
	.word f_dzchk

	.word f_1
	.word f_2
	.word f_add
	.word f_3
	.word f_failneq
	.word f_dzchk

	display test_inc, "1+."
	.word f_lit
	.word -1
	.word f_inc
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_inc
	.word f_2
	.word f_failneq
	.word f_dzchk

	display test_negate, "negate."
	.word f_1
	.word f_negate
	.word f_lit
	.word -1
	.word f_failneq
	.word f_dzchk

	.word f_0
	.word f_negate
	.word f_0
	.word f_failneq
	.word f_dzchk

	.word f_lit
	.word -1
	.word f_negate
	.word f_1
	.word f_failneq
	.word f_dzchk

	.word f_2
	.word f_negate
	.word f_lit
	.word -2
	.word f_failneq
	.word f_dzchk

	.word f_lit
	.word -2
	.word f_negate
	.word f_2
	.word f_failneq
	.word f_dzchk

	display test_sub, "-."
	.word f_1
	.word f_1
	.word f_sub
	.word f_failnez
	.word f_dzchk

	.word f_2
	.word f_1
	.word f_sub
	.word f_1
	.word f_failneq
	.word f_dzchk

	.word f_0
	.word f_1
	.word f_sub
	.word f_lit
	.word -1
	.word f_failneq
	.word f_dzchk

	.word f_2lit
	.word -2
	.word 1
	.word f_sub
	.word f_lit
	.word -3
	.word f_failneq
	.word f_dzchk

	display test_execute, "execute."
	.word f_lit
	.word f_noop
	.word f_execute
	.word f_dzchk

	.word f_lit
	.word f_false
	.word f_execute
	.word f_failnez
	.word f_dzchk

	display test_load, "@."
	.word f_lit
	.word 1f
	.word f_load
	.word f_lit
1:
	.word 0xFF00AA55
	.word f_failneq
	.word f_dzchk

	display test_loadexecute, "@execute."
	.word f_lit
	.word 1f
	.word f_loadexecute
1:
	.word f_noop
	.word f_dzchk

	display test_emit, ""
	.word f_lit
	.word 'e'
	.word f_emit
	.word f_lit
	.word 'm'
	.word f_emit
	.word f_lit
	.word 'i'
	.word f_emit
	.word f_lit
	.word 't'
	.word f_emit
	.word f_lit
	.word '.'
	.word f_emit
	.word f_dzchk

	.word f_lit
	.word 0x20
2:
	.word f_dup
	.word f_lit
	.word 0x7F
	.word f_neq
	.word f_0branch
	.word 1f
	.word f_dup
	.word f_emit
	.word f_inc
	.word f_branch
	.word 2b
1:
	.word f_drop
	.word f_dzchk

	display test_dec, "1-."
	.word f_1
	.word f_dec
	.word f_failnez
	.word f_dzchk

	.word f_0
	.word f_dec
	.word f_lit
	.word -1
	.word f_equ
	.word f_faileqz
	.word f_dzchk

	.word f_2
	.word f_dec
	.word f_1
	.word f_equ
	.word f_faileqz
	.word f_dzchk

	display test_cload, "c@."
	.word f_lit
	.word 1f
	.word f_cload
	.word f_lit
1:
	.word 0x00000055
	.word f_equ
	.word f_faileqz
	.word f_dzchk

	display test_swap, "swap."
	.word f_2lit
	.word 0x55
	.word 0xAA
	.word f_swap
	.word f_lit
	.word 0x55
	.word f_equ
	.word f_faileqz
	.word f_lit
	.word 0xAA
	.word f_equ
	.word f_faileqz
	.word f_dzchk

	display test_dup, "dup."
	.word f_lit
	.word 0x33
	.word f_dup
	.word f_lit
	.word 0x33
	.word f_equ
	.word f_faileqz
	.word f_lit
	.word 0x33
	.word f_equ
	.word f_faileqz
	.word f_dzchk

	display test_drop, "drop."
	.word f_lit
	.word 0x44
	.word f_drop
	.word f_dzchk

	display test_2drop, "2drop."
	.word f_2lit
	.word 0x22
	.word 0x11
	.word f_2drop
	.word f_dzchk

	display test_type, ""
	.word f_2lit
	.word __str_test_type
	.word 5
	.word f_type
	.word f_dzchk

	.word f_branch
	.word 1f
__str_test_type:
	.ascii "type."
	.p2align 2, 0xFF
1:

	display test_spload, "sp@."
	.word f_spload
	.word f_lit
	.word dstk_top_human
	.word f_failneq
	.word f_dzchk

	display test_stload, "st@."
	.word f_stload
	.word f_lit
	.word dstk_top_human
	.word f_failneq
	.word f_dzchk

	display test_rshift, "rshift."
	.word f_1
	.word f_0
	.word f_rshift
	.word f_1
	.word f_failneq
	.word f_dzchk

	.word f_2
	.word f_1
	.word f_rshift
	.word f_1
	.word f_failneq
	.word f_dzchk

	.word f_4
	.word f_1
	.word f_rshift
	.word f_2
	.word f_failneq
	.word f_dzchk

	.word f_4
	.word f_2
	.word f_rshift
	.word f_1
	.word f_failneq
	.word f_dzchk

	display test_4div, "4/."
	.word f_4
	.word f_4div
	.word f_1
	.word f_equ
	.word f_faileqz
	.word f_dzchk

	.word f_lit
	.word 8
	.word f_4div
	.word f_2
	.word f_failneq
	.word f_dzchk

	display test_depth, "depth."
	.word f_depth
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_depth
	.word f_failneq
	.word f_dzchk

	display test_ssload, "ss@."
	.word f_ssload
	.word f_failnez
	.word f_dzchk

	display test_and, "and."
	.word f_0
	.word f_1
	.word f_and
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_0
	.word f_and
	.word f_failnez
	.word f_dzchk

	.word f_0
	.word f_0
	.word f_and
	.word f_failnez
	.word f_dzchk

	.word f_1
	.word f_1
	.word f_and
	.word f_1
	.word f_failneq
	.word f_dzchk

	display test_sschk_ssrst, "sschk.ssrst."
	.word f_sschk
	.word f_faileqz

	.word f_drop
	.word f_0
	.word f_sschk
	.word f_failnez

	.word f_ssrst
	.word f_sschk
	.word f_faileqz
	.word f_dzchk

	display test_num2hex, "num2hex."
	.word f_0
	.word f_num2hex
	.word f_lit
	.word '0'
	.word f_failneq

	.word f_9
	.word f_num2hex
	.word f_lit
	.word '9'
	.word f_failneq

	.word f_0xA
	.word f_num2hex
	.word f_lit
	.word 'A'
	.word f_failneq

	.word f_0xF
	.word f_num2hex
	.word f_lit
	.word 'F'
	.word f_failneq

	display test_09AF_hex4, "hex4."
	.word f_0
	.word f_hex4
	.word f_1
	.word f_hex4
	.word f_2
	.word f_hex4
	.word f_3
	.word f_hex4
	.word f_4
	.word f_hex4
	.word f_5
	.word f_hex4
	.word f_6
	.word f_hex4
	.word f_7
	.word f_hex4
	.word f_8
	.word f_hex4
	.word f_9
	.word f_hex4
	.word f_0xA
	.word f_hex4
	.word f_0xB
	.word f_hex4
	.word f_0xC
	.word f_hex4
	.word f_0xD
	.word f_hex4
	.word f_0xE
	.word f_hex4
	.word f_0xF
	.word f_hex4
	.word f_lit
	.word '.'
	.word f_emit
	.word f_dzchk

	display test_09AF_hex8, "hex8."
	.word f_lit
	.word 0x01
	.word f_hex8
	.word f_lit
	.word 0x23
	.word f_hex8
	.word f_lit
	.word 0x45
	.word f_hex8
	.word f_lit
	.word 0x67
	.word f_hex8
	.word f_lit
	.word 0x89
	.word f_hex8
	.word f_lit
	.word 0xAB
	.word f_hex8
	.word f_lit
	.word 0xCD
	.word f_hex8
	.word f_lit
	.word 0xEF
	.word f_hex8
	.word f_lit
	.word '.'
	.word f_emit
	.word f_dzchk

	display test_09AF_hex16, "hex16."
	.word f_lit
	.word 0x0123
	.word f_hex16
	.word f_lit
	.word 0x4567
	.word f_hex16
	.word f_lit
	.word 0x89AB
	.word f_hex16
	.word f_lit
	.word 0xCDEF
	.word f_hex16
	.word f_lit
	.word '.'
	.word f_emit
	.word f_dzchk

	display test_09AF_hex32, "hex32."
	.word f_lit
	.word 0x01234567
	.word f_hex32
	.word f_lit
	.word 0x89ABCDEF
	.word f_hex32
	.word f_lit
	.word '.'
	.word f_emit
	.word f_dzchk

	display test_dot, ".."
	.word f_lit
	.word 0x01234567
	.word f_dot
	.word f_lit
	.word 0x89ABCDEF
	.word f_dot
	.word f_dzchk

	display test_dsdump, ".s."
	.word f_dsdump
	.word f_1
	.word f_2
	.word f_3
	.word f_4
	.word f_dsdump
	.word f_2drop
	.word f_dsdump
	.word f_2drop
	.word f_dsdump
	.word f_dzchk

	display test_ltz, "0<."
	.word f_lit
	.word -1
	.word f_ltz
	.word f_faileqz

	.word f_lit
	.word -2
	.word f_ltz
	.word f_faileqz

	.word f_lit
	.word 0
	.word f_ltz
	.word f_failnez

	.word f_lit
	.word 1
	.word f_ltz
	.word f_failnez
	.word f_dzchk

	display test_gez, "0>=."
	.word f_lit
	.word -1
	.word f_gez
	.word f_failnez

	.word f_0
	.word f_gez
	.word f_faileqz

	.word f_1
	.word f_gez
	.word f_faileqz
	.word f_dzchk

	display test_lt, "<."
	.word f_0
	.word f_0
	.word f_lt
	.word f_failnez

	.word f_0
	.word f_1
	.word f_lt
	.word f_faileqz

	.word f_1
	.word f_2
	.word f_lt
	.word f_faileqz

	.word f_2lit
	.word -1
	.word 0
	.word f_lt
	.word f_faileqz

	.word f_2lit
	.word -2
	.word -1
	.word f_lt
	.word f_faileqz

	.word f_2lit
	.word -1
	.word -2
	.word f_lt
	.word f_failnez

	.word f_2lit
	.word 0
	.word -1
	.word f_lt
	.word f_failnez

	.word f_2lit
	.word 0
	.word -1
	.word f_lt
	.word f_failnez

	.word f_2lit
	.word 1
	.word 0
	.word f_lt
	.word f_failnez

	.word f_2lit
	.word 2
	.word 1
	.word f_lt
	.word f_failnez

	.word f_2lit
	.word 1
	.word -1
	.word f_lt
	.word f_failnez
	.word f_dzchk

	display gt, ">."
	.word f_0
	.word f_0
	.word f_gt
	.word f_failnez

	.word f_1
	.word f_0
	.word f_gt
	.word f_faileqz

	.word f_2
	.word f_1
	.word f_gt
	.word f_faileqz

	.word f_2lit
	.word 0
	.word -1
	.word f_gt
	.word f_faileqz

	.word f_2lit
	.word -1
	.word -2
	.word f_gt
	.word f_faileqz

	.word f_2lit
	.word 1
	.word -1
	.word f_gt
	.word f_faileqz

	.word f_2lit
	.word -1
	.word 0
	.word f_gt
	.word f_failnez

	.word f_2lit
	.word -2
	.word -1
	.word f_gt
	.word f_failnez

	.word f_2lit
	.word 1
	.word 2
	.word f_gt
	.word f_failnez

	.word f_2lit
	.word 0
	.word 1
	.word f_gt
	.word f_failnez
	.word f_dzchk

	display test_tor_fromr, ">r.r>."
	.word f_5
	.word f_tor
	.word f_fromr
	.word f_5
	.word f_failneq
	.word f_dzchk

	display test_over, "over."
	.word f_0
	.word f_1
	.word f_over
	.word f_failnez
	.word f_1
	.word f_failneq
	.word f_failnez
	.word f_dzchk

	display test_2dup, "2dup."
	.word f_0
	.word f_1
	.word f_2dup
	.word f_1
	.word f_failneq
	.word f_failnez
	.word f_1
	.word f_failneq
	.word f_failnez
	.word f_dzchk

	display test_nip, "nip."
	.word f_0
	.word f_1
	.word f_nip
	.word f_1
	.word f_failneq
	.word f_dzchk

	display test_min, "min."
	.word f_0
	.word f_1
	.word f_min
	.word f_failnez

	.word f_1
	.word f_0
	.word f_min
	.word f_failnez

	.word f_2lit
	.word -1
	.word 0
	.word f_min
	.word f_lit
	.word -1
	.word f_failneq

	.word f_2lit
	.word 1
	.word -1
	.word f_min
	.word f_lit
	.word -1
	.word f_failneq
	.word f_dzchk

	display test_rot, "rot."
	.word f_0
	.word f_1
	.word f_2
	.word f_rot
	.word f_failnez
	.word f_2
	.word f_failneq
	.word f_1
	.word f_failneq
	.word f_dzchk

	display test_compare, "compare."
	.word f_2lit
	.word _tcmp0
	.word 0
	.word f_2lit
	.word _tcmp1
	.word 5
	.word f_compare
	.word f_failnez

	.word f_2lit
	.word _tcmp0
	.word 5
	.word f_2lit
	.word _tcmp1
	.word 5
	.word f_compare
	.word f_faileqz

	.word f_2lit
	.word _tcmp0
	.word 5
	.word f_2lit
	.word _tcmp1
	.word 6
	.word f_compare
	.word f_faileqz

	.word f_2lit
	.word _tcmp0
	.word 6
	.word f_2lit
	.word _tcmp1
	.word 6
	.word f_compare
	.word f_failnez
	.word f_dzchk

	.word f_branch
	.word 1f
_tcmp0:
	.ascii "HelloWorld"
_tcmp1:
	.ascii "HelloAlice"
	.p2align 2, 0xFF
1:

	display test_wlinkload, "wlink@.",
	.word f_lit
	.word f_okay
	.word f_wlinkload
	.word f_failnez

	.word f_lit
	.word f_fail
	.word f_wlinkload
	.word f_lit
	.word f_okay
	.word f_failneq

	.word f_dzchk

	display test_wnlenload, "wnlen@."
	.word f_lit
	.word f_okay
	.word f_wnlenload
	.word f_4
	.word f_failneq

	.word f_lit
	.word f_lit
	.word f_wnlenload
	.word f_3
	.word f_failneq

	.word f_dzchk

	display test_wnameload, "wname@."
	.word f_lit
	.word f_okay
	.word f_wnameload
	.word f_lit
	.word n_okay
	.word f_failneq

	.word f_lit
	.word f_wnameload
	.word f_wnameload
	.word f_lit
	.word n_wnameload
	.word f_failneq

	.word f_dzchk

	display test_latestload, "latest@."
	.word f_latestload
	.word f_lit
	.word lastword
	.word f_failneq
	.word f_dzchk

	display test_words, "words."
	.word f_words
	.word f_dzchk

	display test_2swap, "2swap."
	.word f_0
	.word f_1
	.word f_2
	.word f_3
	.word f_2swap
	.word f_1
	.word f_failneq
	.word f_failnez
	.word f_3
	.word f_failneq
	.word f_2
	.word f_failneq
	.word f_dzchk

	display test_2over, "2over."
	.word f_0
	.word f_1
	.word f_2
	.word f_3
	.word f_2over
	.word f_1
	.word f_failneq
	.word f_failnez
	.word f_3
	.word f_failneq
	.word f_2
	.word f_failneq
	.word f_1
	.word f_failneq
	.word f_failnez
	.word f_dzchk

	display test_find, "find."
	.word f_2lit
	.word n_fail
	.word 4
	.word f_find
	.word f_lit
	.word f_fail
	.word f_failneq
	.word f_dzchk

	.word f_2lit
	.word n_nothing
	.word 7
	.word f_find
	.word f_failnez
	.word f_dzchk

	.word f_2lit
	.word n_2over
	.word 5
	.word f_find
	.word f_lit
	.word f_2over
	.word f_failneq
	.word f_dzchk

	.word f_latestload
	.word f_wnameload
	.word f_latestload
	.word f_wnlenload
	.word f_find
	.word f_latestload
	.word f_failneq
	.word f_dzchk

	display test_toin, ">in.>in@.>in!.>inrst.>inchk."
	.word f_toinrst
	.word f_toinload
	.word f_failnez

	.word f_toinchk
	.word f_faileqz

	.word f_TIBSIZE
	.word f_toinstore
	.word f_toinchk
	.word f_failnez

	.word f_lit
	.word -1
	.word f_toinstore
	.word f_toinchk
	.word f_failnez

	.word f_toinrst
	.word f_toinchk
	.word f_faileqz

	.word f_dzchk

	display test_tibpush_tibdrop, "tibpush.tibdrop."
	.word f_lit
	.word 0x55
	.word f_tibpush
	.word f_tib
	.word f_cload
	.word f_lit
	.word 0x55
	.word f_failneq

	.word f_toinload
	.word f_1
	.word f_failneq

	.word f_tibdrop
	.word f_toinload
	.word f_failnez

	.word f_dzchk

	display test_or, "or."
	.word f_0
	.word f_0
	.word f_or
	.word f_failnez

	.word f_1
	.word f_0
	.word f_or
	.word f_1
	.word f_failneq

	.word f_0
	.word f_1
	.word f_or
	.word f_1
	.word f_failneq

	.word f_1
	.word f_1
	.word f_or
	.word f_1
	.word f_failneq

	.word f_dzchk

	display test_isnewline, "isnewline."
	.word f_lit
	.word '\n'
	.word f_isnewline
	.word f_faileqz

	.word f_lit
	.word '\r'
	.word f_isnewline
	.word f_faileqz

	.word f_lit
	.word 'M'
	.word f_isnewline
	.word f_failnez

	.word f_dzchk

	display test_isdelete, "isdelete."
	.word f_lit
	.word '\b'
	.word f_isdelete
	.word f_faileqz

	.word f_lit
	.word 0x7F
	.word f_isdelete
	.word f_faileqz

	.word f_lit
	.word 'M'
	.word f_isdelete
	.word f_failnez

	.word f_dzchk

	display test_isspace, "isspace."
	.word f_lit
	.word ' '
	.word f_isspace
	.word f_faileqz

	.word f_lit
	.word '\t'
	.word f_isspace
	.word f_faileqz

	.word f_lit
	.word 'M'
	.word f_isspace
	.word f_failnez

	.word f_dzchk

	display test_le, "<=."
	.word f_0
	.word f_0
	.word f_le
	.word f_faileqz

	.word f_0
	.word f_1
	.word f_le
	.word f_faileqz

	.word f_1
	.word f_0
	.word f_le
	.word f_failnez
	.word f_dzchk

	display test_within, "within."
	.word f_0
	.word f_1
	.word f_2
	.word f_within
	.word f_failnez

	.word f_1
	.word f_1
	.word f_2
	.word f_within
	.word f_faileqz

	.word f_2
	.word f_1
	.word f_2
	.word f_within
	.word f_failnez

	.word f_dzchk

	display test_isxdigit, "isxdigit."
	.word f_lit
	.word '0'
	.word f_isxdigit
	.word f_faileqz

	.word f_lit
	.word '9'
	.word f_isxdigit
	.word f_faileqz

	.word f_lit
	.word 'A'
	.word f_isxdigit
	.word f_faileqz

	.word f_lit
	.word 'F'
	.word f_isxdigit
	.word f_faileqz

	.word f_lit
	.word '0' - 1
	.word f_isxdigit
	.word f_failnez

	.word f_lit
	.word '9' + 1
	.word f_isxdigit
	.word f_failnez

	.word f_lit
	.word 'A' - 1
	.word f_isxdigit
	.word f_failnez

	.word f_lit
	.word 'F' + 1
	.word f_isxdigit
	.word f_failnez

	.word f_dzchk

	display test_ishexstr, "ishexstr."
	.word f_0
	.word f_0
	.word f_ishexstr
	.word f_failnez

	.word f_2lit
	.word _hx0
	.word 6
	.word f_ishexstr
	.word f_faileqz

	.word f_2lit
	.word _hx1
	.word 3
	.word f_ishexstr
	.word f_faileqz

	.word f_2lit
	.word _hx2
	.word 6
	.word f_ishexstr
	.word f_failnez

	.word f_2lit
	.word _hx3
	.word 4
	.word f_ishexstr
	.word f_failnez

	.word f_dzchk

	.word f_branch
	.word 1f
_hx0:
	.ascii "0x1234"
_hx1:
	.ascii "0x0"
_hx2:
	.ascii "0x123M"
_hx3:
	.ascii "1234"
	.p2align 2, 0xFF
1:

	display test_lshift, "lshift."
	.word f_1
	.word f_0
	.word f_lshift
	.word f_1
	.word f_failneq

	.word f_1
	.word f_1
	.word f_lshift
	.word f_2
	.word f_failneq

	.word f_1
	.word f_2
	.word f_lshift
	.word f_4
	.word f_failneq

	.word f_1
	.word f_3
	.word f_lshift
	.word f_8
	.word f_failneq

	.word f_dzchk

	display test_4mul, "4*."
	.word f_0
	.word f_4mul
	.word f_failnez

	.word f_1
	.word f_4mul
	.word f_4
	.word f_failneq

	.word f_2
	.word f_4mul
	.word f_8
	.word f_failneq

	.word f_dzchk

	display test_hex2num, "hex2num."
	.word f_lit
	.word '0'
	.word f_hex2num
	.word f_failnez

	.word f_lit
	.word '1'
	.word f_hex2num
	.word f_1
	.word f_failneq

	.word f_lit
	.word '9'
	.word f_hex2num
	.word f_9
	.word f_failneq

	.word f_lit
	.word 'A'
	.word f_hex2num
	.word f_0xA
	.word f_failneq

	.word f_lit
	.word 'F'
	.word f_hex2num
	.word f_0xF
	.word f_failneq

	.word f_dzchk

	display test_hexstr2num, "hexstr2num."
	.word f_0
	.word f_0
	.word f_hexstr2num
	.word f_failnez
	.word f_dzchk

	.word f_2lit
	.word _hex0
	.word 3
	.word f_hexstr2num
	.word f_1
	.word f_failneq
	.word f_dzchk

	.word f_2lit
	.word _hex1
	.word 5
	.word f_hexstr2num
	.word f_lit
	.word 0x123
	.word f_failneq
	.word f_dzchk

	.word f_2lit
	.word _hex2
	.word 10
	.word f_hexstr2num
	.word f_lit
	.word 0x01234567
	.word f_failneq
	.word f_dzchk

	.word f_2lit
	.word _hex3
	.word 10
	.word f_hexstr2num
	.word f_lit
	.word 0x89ABCDEF
	.word f_failneq
	.word f_dzchk

	.word f_branch
	.word 1f
_hex0:
	.ascii "0x1"
_hex1:
	.ascii "0x123"
_hex2:
	.ascii "0x01234567"
_hex3:
	.ascii "0x89ABCDEF"
	.p2align 2, 0xFF
1:

	display test_isnumber, "number?."
	.word f_2lit
	.word _hex1
	.word 3
	.word f_isnumber
	.word f_faileqz
	.word f_dzchk

	display test_number, "number."
	.word f_2lit
	.word _hex1
	.word 3
	.word f_number
	.word f_1
	.word f_failneq
	.word f_dzchk

	display test_wishiden, "wishiden."
	.word f_lit
	.word f_nothing
	.word f_wishiden
	.word f_faileqz
	.word f_dzchk

	.word f_lit
	.word f_call
	.word f_wishiden
	.word f_failnez
	.word f_dzchk

	display test_wisimmed, "wisimmed."
	.word f_lit
	.word f_nopimmed
	.word f_wisimmed
	.word f_faileqz
	.word f_dzchk

	.word f_lit
	.word f_lit
	.word f_wisimmed
	.word f_failnez
	.word f_dzchk

	display test_here, "here.here@.here!.,."
	.word f_hereload
	.word f_lit
	.word 0x55AA
	.word f_comma
	.word f_dup
	.word f_load
	.word f_lit
	.word 0x55AA
	.word f_failneq
	.word f_hereload
	.word f_swap
	.word f_sub
	.word f_4
	.word f_failneq
	.word f_dzchk

	display test_bic, "bic."
	.word f_0
	.word f_1
	.word f_bic
	.word f_failnez

	.word f_1
	.word f_1
	.word f_bic
	.word f_failnez

	.word f_1
	.word f_0
	.word f_bic
	.word f_1
	.word f_failneq

	.word f_3
	.word f_1
	.word f_bic
	.word f_2
	.word f_failneq

	.word f_dzchk

	display test_comp, "].[.comp?."
	.word f_compon
	.word f_iscomp
	.word f_faileqz
	.word f_dzchk

	.word f_compoff
	.word f_iscomp
	.word f_failnez
	.word f_dzchk

	display test_ccomma_aligned_align, "c,.aligned.align."
	.word f_hereload
	.word f_lit
	.word 0x55
	.word f_ccomma
	.word f_cload
	.word f_lit
	.word 0x55
	.word f_failneq

	.word f_hereload
	.word f_align
	.word f_hereload
	.word f_swap
	.word f_sub
	.word f_3
	.word f_failneq

	.word f_dzchk

	display test_cmove, "cmove."
	.word f_2lit
	.word 1f
	.word 5
	.word f_hereload
	.word f_swap
	.word f_cmove
	.word f_2lit
	.word 1f
	.word 5
	.word f_hereload
	.word f_5
	.word f_compare
	.word f_faileqz
	.word f_dzchk

	.word f_branch
	.word 2f
1:	
	.word 0x55AA4477
2:
	display test_defword, "defword."
	.word f_hereload
	.word f_2lit
	.word n_noop
	.word 4
	.word f_defword
	.word f_hereload
	.word f_swap
	.word f_sub
	.word f_lit
	.word 16
	.word f_failneq

	.word f_2lit
	.word n_noop
	.word 4
	.word f_find
	.word f_comma
	
	.word f_2lit
	.word n_exit
	.word 4
	.word f_find
	.word f_comma

	.word f_latestload
	.word f_whidenclr

	.word f_2lit
	.word n_noop
	.word 4
	.word f_find
	.word f_latestload
	.word f_failneq

	.word f_dzchk

	display test_defconst, "defconst."
	.word f_0
	.word f_2lit
	.word n_0
	.word 1
	.word f_defconst

	.word f_2lit
	.word n_0
	.word 1
	.word f_find
	.word f_latestload
	.word f_failneq

	.word f_dzchk


	//.word f_src
	.word f_motd

	.word f_spi_flash_bus
	.word f_spim_init

	.word f_i2c_ft24_bus
	.word f_i2cm_init

1:
	.word f_interpret
	.word f_branch
	.word 1b

	display test_echo, "echo."
1:
	.word f_key
	.word f_emit
	.word f_dzchk
	.word f_branch
	.word 1b

	.word f_noop
	.word f_okay
fail:
	.word f_fail

boot_back:
1:
	.word f_yield
	.word f_iwdg_feed
	.word f_branch
	.word 1b

forth:
	li ss, 0

	la wp, ycnt
	sw zero, 0(wp)
	
	la wp, func_emit
	la xp, f_uart1_tx
	sw xp, 0(wp)

	la wp, func_key
	la xp, f_uart1_rx
	sw xp, 0(wp)

	la wp, func_dot
	la xp, f_hex32
	sw xp, 0(wp)

	la wp, func_isnumber
	la xp, f_ishexstr
	sw xp, 0(wp)

	la wp, func_number
	la xp, f_hexstr2num
	sw xp, 0(wp)

	la wp, func_spi_flash_cs
	la xp, f_spi_flash_cs_pa2
	sw xp, 0(wp)

	la wp, func_spi_flash_bus
	la xp, f_SPI1
	sw xp, 0(wp)

	la wp, func_i2c_ft24_addr
	la xp, f_0xA0
	sw xp, 0(wp)

	la wp, func_i2c_ft24_bus
	la xp, f_I2C2
	sw xp, 0(wp)

	la wp, latest
	la xp, lastword
	sw xp, 0(wp)

	la wp, here
	la xp, ram_dict
	sw xp, 0(wp)

	la wp, toin
	sw zero, 0(wp)

	la up, task_human
	la wp, task_back
	sw wp, TNP(up)
	la sp, dstk_top_human
	mv st, sp
	la rp, rstk_top_human
	la ip, boot_human
	call tasksave

	la up, task_back
	la wp, task_human
	sw wp, TNP(up)
	la sp, dstk_top_back
	mv st, sp
	la rp, rstk_top_back
	la ip, boot_back
	call tasksave

	next

	.section .bss
	.p2align 2
_ram_vector:
	.fill _vector_size, 1, 0
_ram_vector_end:
	.p2align 2
ycnt:
	.fill 1, ADDRSIZE, 0
	.p2align 2
tib:
	.fill TIBSIZE, 1, 0
tib_end:
	.fill 1, 1, 0
	.p2align 2
toin:
	.fill 1, ADDRSIZE, 0
	.p2align 2
task_human:
	.fill TASKSIZE, ADDRSIZE, 0
	.p2align 2
dstk_human:
	.fill STKSIZE, ADDRSIZE, 0
dstk_top_human:
	.fill 1, ADDRSIZE, 0
	.p2align 2
rstk_human:
	.fill STKSIZE, ADDRSIZE, 0
rstk_top_human:
	.fill 1, ADDRSIZE, 0
	.p2align 2
task_back:
	.fill TASKSIZE, ADDRSIZE, 0
	.p2align 2
dstk_back:
	.fill STKSIZE, ADDRSIZE, 0
dstk_top_back:
	.fill 1, ADDRSIZE, 0
	.p2align 2
rstk_back:
	.fill STKSIZE, ADDRSIZE, 0
rstk_top_back:
	.fill 1, ADDRSIZE, 0
	.p2align 12
save_start:
func_emit:
	.fill 1, ADDRSIZE, 0
func_key:
	.fill 1, ADDRSIZE, 0
func_dot:
	.fill 1, ADDRSIZE, 0
func_isnumber:
	.fill 1, ADDRSIZE, 0
func_number:
	.fill 1, ADDRSIZE, 0
func_spi_flash_cs:
	.fill 1, ADDRSIZE, 0
func_spi_flash_bus:
	.fill 1, ADDRSIZE, 0
func_i2c_ft24_addr:
	.fill 1, ADDRSIZE, 0
func_i2c_ft24_bus:
	.fill 1, ADDRSIZE, 0
latest:
	.fill 1, ADDRSIZE, 0
here:
	.fill 1, ADDRSIZE, 0
	.p2align 2
ram_dict:
