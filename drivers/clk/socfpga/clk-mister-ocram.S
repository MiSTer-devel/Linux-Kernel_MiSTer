#include <linux/linkage.h>
#include "clk-mister-ocram.h"

	.arch armv7-a
	.arm
	.text
	.balign 8

	.macro stage value
	mov r2, #\value
	str r2, [r1, #MISTER_STAGE]
	.endm

	.macro counter regoff, saveoff, targetoff
	ldr r2, [r1, #\saveoff]
	movw r3, #511
	bic r2, r2, r3
	ldr r3, [r1, #\targetoff]
	orr r2, r2, r3
	str r2, [r0, #\regoff]
	.endm

ENTRY(socfpga_mister_ocram)
	push {r4-r11, lr}
	ldr r4, [r0, #0x04]
	ldr r5, [r0, #0x40]
	ldr r6, [r0, #0x60]
	ldr r7, [r0, #0xa0]
	ldr r2, [r0, #0x48]
	str r2, [r1, #MISTER_OLD_MPU]
	ldr r2, [r0, #0xe4]
	str r2, [r1, #MISTER_OLD_MAIN]
	ldr r2, [r0, #0xe8]
	str r2, [r1, #MISTER_OLD_DBG]
	ldr r2, [r0, #0x5c]
	str r2, [r1, #MISTER_OLD_USER]
	ubfx r8, r5, #3, #13
	ldr r9, [r1, #MISTER_TARGET_NUMER]
	mov r11, #0
	stage 1
	bl .Lwait_idle
	cmp r2, #0
	bne .Lbefore_gate_error

	dsb sy
	stage 2
	bic r2, r6, #0x300
	str r2, [r0, #0x60]
	ldr r2, [r1, #MISTER_FLASH_GATES]
	bic r2, r7, r2
	str r2, [r0, #0xa0]
	dsb sy
	stage 3
	orr r2, r4, #1
	str r2, [r0, #0x04]
	bl .Lwait_idle
	cmp r2, #0
	bne .Lrollback
	stage 4
	counter 0x48, MISTER_OLD_MPU, MISTER_TARGET_MPU
	counter 0xe4, MISTER_OLD_MAIN, MISTER_TARGET_MAIN
	counter 0xe8, MISTER_OLD_DBG, MISTER_TARGET_MAIN
	counter 0x5c, MISTER_OLD_USER, MISTER_TARGET_USER
	stage 5
	bl .Lramp
	cmp r2, #0
	bne .Lrollback
	stage 6
	bl .Lreset_outputs
	str r4, [r0, #0x04]
	bl .Lwait_idle
	cmp r2, #0
	bne .Lrollback
	stage 7
	b .Lrestore_gates

.Lrollback:
	mvn r11, #109
	stage 16
	orr r2, r4, #1
	str r2, [r0, #0x04]
	bl .Lwait_idle
	cmp r2, #0
	bne .Lhalt
	ldr r2, [r1, #MISTER_OLD_MPU]
	str r2, [r0, #0x48]
	ldr r2, [r1, #MISTER_OLD_MAIN]
	str r2, [r0, #0xe4]
	ldr r2, [r1, #MISTER_OLD_DBG]
	str r2, [r0, #0xe8]
	ldr r2, [r1, #MISTER_OLD_USER]
	str r2, [r0, #0x5c]
	ubfx r9, r5, #3, #13
	bl .Lramp
	cmp r2, #0
	bne .Lsafe_bypass
	bl .Lreset_outputs
	str r4, [r0, #0x04]
	bl .Lwait_idle
	cmp r2, #0
	bne .Lsafe_bypass
	stage 17
	b .Lrestore_gates

.Lsafe_bypass:

	orr r2, r4, #1
	str r2, [r0, #0x04]
	bl .Lwait_idle
	cmp r2, #0
	bne .Lhalt
	mvn r11, #4
	stage 254
	b .Lrestore_gates

.Lhalt:

	stage 255
	mvn r2, #4
	str r2, [r1, #MISTER_ERROR]
1:
	b 1b

.Lrestore_gates:
	str r7, [r0, #0xa0]
	str r6, [r0, #0x60]
	dsb sy
	mov r2, r11
.Lbefore_gate_error:
	str r2, [r1, #MISTER_ERROR]
	mov r0, r2
	pop {r4-r11, pc}

.Lwait_idle:
	movw r3, #4096
1:
	ldr r2, [r0, #0x14]
	tst r2, #1
	beq 2f
	subs r3, r3, #1
	bne 1b
	mvn r2, #109
	bx lr
2:
	mov r2, #0
	bx lr

.Lreset_outputs:

	ldr r2, [r0, #0x40]
	orr r3, r2, #0x1000000
	str r3, [r0, #0x40]
	dsb sy
	str r2, [r0, #0x40]
	dsb sy
	bx lr

.Lramp:

1:
	cmp r8, r9
	beq 4f
	addlo r8, r8, #8
	subhi r8, r8, #8
	movw r3, #0xfff8
	bic r2, r5, r3
	orr r2, r2, r8, lsl #3
	str r2, [r0, #0x40]

	movw r3, #2048
2:
	ldr r2, [r0, #0x14]
	subs r3, r3, #1
	bne 2b
4:
	movw r3, #4096
3:
	ldr r2, [r0, #0x08]
	tst r2, #64
	bne 5f
	subs r3, r3, #1
	bne 3b
	mvn r2, #109
	bx lr
5:
	cmp r8, r9
	bne 1b
	mov r2, #0
	bx lr
ENDPROC(socfpga_mister_ocram)
	.balign 4
	.globl socfpga_mister_ocram_sz
socfpga_mister_ocram_sz:
	.long . - socfpga_mister_ocram
