; read the optical rotary encoder's gray code to get the angle
.equ control_address, 0xfe23
.equ porta, 0xfe20
.equ portb, 0xfe21
.equ portc, 0xfe22
.equ gray_bits, 0x07
.equ theta_bits, 0x3f
.equ max_skip, 0x02

.equ s_theta1, 0x7000
.equ s_theta2, 0x7100
.equ s_dtheta1, 0x7200
.equ s_dtheta2, 0x7300

; registers
; r1 = decrement to output to lcd
; r2 = theta_1
; r3 = theta_2
; r4 = dtheta_1
; r5 = dtheta_2
; r6 = last gray code
; r7 = current gray code

.org 0
ljmp start

.org 0bh
ljmp t0isr

.org 100h
start:
lcall init
loop:
	sjmp loop

init:
	; set timer 0 in mode 2 (8 bit autoreload)
	mov tmod, #02h
	mov th0, #0xff
	mov ie, #82h

	; set up 8255
	mov dptr, #control_address
	mov a, #0x90; port A is input, ports B and C are output
	movx @dptr, a

	; set initial state
	mov dptr, #portc
	movx a, @dptr
	lcall gray2bin
	mov r6, a

	lcall init_lcd

	setb tr0 ; start timer
	ret

t0isr:
	; read input pins
	mov dptr, #portc
	movx a, @dptr
	lcall gray2bin
	mov r7, a

	clr c
	subb a, r6
	anl a, #gray_bits
	clr c
	subb a, #max_skip
	jc ignore

	mov a, r6
	clr c
	subb a, r7
	anl a, #gray_bits
	clr c
	subb a, #max_skip
	jc ignore

	; difference is in range, switch to new angle
	mov a, r7
	clr c
	subb a, r6 ; get offset
	add a, r2 ; add to current angle
	anl a, #theta_bits
	mov r2, a ; store
	mov r6, 7 ; move new decoded gray code to old

	ignore:

	mov p1, r6
	djnz r1, skip_lcd


	skip_lcd:

	reti

lcd:
	lcall reset_lcd

	; print theta1 to LCD
	mov dptr, #s_theta1
	lcall print
	mov a, r2
	lcall print_value

	; print space to LCD
	mov a, #' '
	lcall print_chr

	; print theta2 to LCD
	mov dptr, #s_theta2
	lcall print
	mov a, r3
	lcall print_value

	ret

gray2bin:
	; convert gray code in acc to binary
	g2b7:
		jnb acc.7, g2b6
		cpl acc.6
	g2b6:
		jnb acc.6, g2b5
		cpl acc.5
	g2b5:
		jnb acc.5, g2b4
		cpl acc.4
	g2b4:
		jnb acc.4, g2b3
		cpl acc.3
	g2b3:
		jnb acc.3, g2b2
		cpl acc.2
	g2b2:
		jnb acc.2, g2b1
		cpl acc.1
	g2b1:
		jnb acc.1, g2b0
		cpl acc.0
	g2b0:
	ret

init_lcd:
	; set display for 8 bit communication, 5x7 character set
	mov dptr, #portc
	mov a, #0x00
	movx @dptr, a ; lower enable line
	mov dptr, #porta
	mov a, #0x38
	movx @dptr, a
	lcall latch

	; turn display on, hide cursor
	mov dptr, #porta
	mov a, #0x0c
	movx @dptr, a
	lcall latch

	lcall reset_lcd

	ret

reset_lcd:
	; clear display
	mov dptr, #porta
	mov a, #0x01
	movx @dptr, a
	lcall latch

	; set RAM address to zero
	mov dptr, #porta
	mov a, #0x80
	movx @dptr, a
	lcall latch

	; switch to write mode
	mov dptr, #portc
	mov a, #0x10
	lcall wait

	ret


latch:
	mov dptr, #portc
	mov a, #0x40
	movx @dptr, a ; raise enable line
	lcall wait
	mov a, #0x00
	movx @dptr, a ; lower enable line
	ret

print_chr:
	mov dptr, #porta
	movx @dptr, a ; write charater to LCD data port
	mov dptr, #portc
	mov a, #0x50
	movx @dptr, a ; raise enable line
	lcall wait
	mov a, #0x10
	movx @dptr, a ; lower enable line
	ret

; print the value of acc
print_value:
	; print hundred's place
	mov b, #100d
	div ab
	orl a, #0x30 ; convert to ascii
	lcall print_chr
	; print ten's place
	mov a, b
	mov b, #10d
	div ab
	orl a, #0x30 ; convert to ascii
	lcall print_chr
	; print one's place
	mov a, b
	orl a, #0x30 ; convert to ascii
	lcall print_chr
	ret

; print the string at dptr to the LCD
print:
	mov r0, #0x00
	print_loop:
		mov a, r0
		movc a, @dptr+a
		inc r0
		jz print_done ; null-terminated string

		lcall print_chr

		mov a, r0
		subb a, #24d ; max string length
		jnz print_loop
	print_done:
	ret

wait:
mov a, #0x00
wait_loop:
	djnz acc, wait_loop
ret

; 0xf2 is theta
.org s_theta1
.db 0xf2, "1=", 0x00
.org s_theta2
.db 0xf2, "2=", 0x00
.org s_dtheta1
.db 0xf2, "1'=", 0x00
.org s_dtheta2
.db 0xf2, "2'=", 0x00
