; Simple ZX Spectrum CPU clock frequency measurement
; (c) 2017 gyurco@freemail.hu
;
; compile with:
; z80asm -b bench.asm
; appmake +zx -b bench.bin  --org 60000 -o bench.tap
;
; running:
; let ticks=usr 60000
; let f=5055390/ticks*50/1000
; print "freq=";f;" kHz"


	defc	irqhandler=$FEFE

	org	60000

	di
	push	af
	push	de
	push	hl
	ld	a,$FE
	ld	($FC00),a
	ld	hl,$FC00
	ld	de,$FC01
	ld	bc,$100
	ldir
	ld	hl,irqh
	ld	de,irqhandler
	ld	bc,irqh_end-irqh
	ldir
	xor	a
	ld	h,a
	ld	l,a
	ld	(counter),hl
	ld	bc,$ffff
	ld	a,$FC
	ld	i,a
	im	2
	ei
;loop total T=5055390
loop:	nop	;4
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop	;64 * 65535
	djnz	loop; 65535*13+8
	ld	a,c; 4 * 255
	or	a; 4 * 255
	jr	z,loopend; 255*12+7
	dec	c; 4 * 255
	jr	loop ;12 * 255
loopend:di
	ld	a,$3F
	ld	i,a
	im	1
	ei
	ld	bc,(counter)
	pop	hl
	pop	de
	pop	af
	ret


counter: defw 0
irqh:	push	af
	push	hl
	ld	hl,(counter)
	inc	hl
	ld	(counter),hl
	pop	hl
	pop	af
	ei
	reti
irqh_end:
