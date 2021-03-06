; read the optical rotary encoder's gray code to get the angle
.equ control_address, 0xfe23
.equ porta, 0xfe20
.equ portb, 0xfe21
.equ portc, 0xfe22
.equ num_gray_bits, 0x03
.flag gray_msb, acc.2
.equ gray_bits, (1 << num_gray_bits) - 1
.equ theta_bits, 0x7f
.equ max_skip, 0x03
.equ stack, 0x2f

.equ s_theta1, 0x7000
.equ s_theta2, 0x7100
.equ s_dtheta1, 0x7200
.equ s_dtheta2, 0x7300

; registers
; r0 = general register
; r1 = decrement to output to lcd
; registers r2-r7 are different for theta_1 and theta_2
; theta_1 is in register bank 0, theta_2 is in register bank 1
; r2 = low byte of counter since last change
; r3 = high byte of counter since last change, r3.7 determines whether to increment/decrement
; r4 = theta
; r5 = 1/dtheta
; r6 = last gray code
; r7 = current gray code

.org 0
ljmp start

.org 0bh
ljmp t0isr

.org 100h
start:
mov sp, #stack ; reinitialize stack pointer
lcall init
loop:
	mov a, r1
	jnz loop
	lcall lcd
	sjmp loop

init:
	; set timer 0 in mode 2 (8 bit autoreload)
	; set timer 1 in mode 2 (8 bit autoreload)
	mov tmod, #22h
	mov th0, #0x80
	mov th1, #0xff ; set 57600 baud
	mov pcon, #0x80
	mov scon, #0x52 ; set serial for 8 bit data, ready to send initially
	setb it0 ; set tcon.0 for edge triggered interrupts
	setb tr1 ; start timer 1 for serial communication
	mov ie, #82h

	; set up 8255
	mov dptr, #control_address
	mov a, #0x82; port B is input, ports A and C are output
	movx @dptr, a

	lcall init_lcd

	; set initial state
	mov dptr, #portb
	movx a, @dptr
	push acc
	; theta1 bits are in PB.0-PB.2, theta2 bits ae in PB.4-PB.6
	anl a, #gray_bits
	lcall init_rotary
	setb rs0 ; switch to register bank 1
	pop acc
	swap a
	anl a, #gray_bits
	lcall init_rotary
	clr rs0

	; tell PSoC to restart
	mov a, #0x00
	lcall sndchr
	mov a, #0x80
	lcall sndchr

	setb tr0 ; start timer
	ret

; initialize variables for the rotary encoder specified by the register bank
init_rotary:
	mov r2, #0x00
	mov r3, #0x00
	mov r4, #0x00
	mov r5, #0x00

	lcall gray2bin
	mov r6, a

	ret

t0isr:
	push dph
	push dpl
	push acc
	push b

	; read input pins
	mov dptr, #portb
	movx a, @dptr
	mov p1, a
	setb p1.7
	push acc
	anl a, #gray_bits
	lcall update_theta
	setb rs0
	pop acc
	swap a
	anl a, #gray_bits
	lcall update_theta
	clr rs0

	djnz r1, skip_lcd
	;lcall lcd
	skip_lcd:

	pop b
	pop acc
	pop dpl
	pop dph
	clr p1.7

	reti

; expects gray code in acc and rs0 to select which theta to update
update_theta:
	lcall gray2bin
	mov r7, a
	sjmp done_dangle_counter

	mov a, r3
	anl a, #0x80
	jnz decrement_counter
	increment_counter:
		; increment counter for time since last change
		mov a, r2
		add a, #0x01
		mov r2, a
		jnc done_dangle_counter
		mov a, r3
		add a, #0x01
		jb ov, done_dangle_counter
		mov r3, a
		sjmp done_dangle_counter
	decrement_counter:
		; decrement counter for time since last change
		mov a, r2
		clr c
		subb a, #0x01
		mov r2, a
		jnc done_dangle_counter
		mov a, r3
		subb a, #0x00 ; 0 because c is set
		jb ov, done_dangle_counter
		mov r3, a
	done_dangle_counter:

	mov a, r7
	clr c
	subb a, r6
	jz done_angle ; short circuit when no change
	anl a, #gray_bits
	clr c
	subb a, #max_skip+1
	jc dangle_within_limits

	mov a, r6
	clr c
	subb a, r7
	anl a, #gray_bits
	clr c
	subb a, #max_skip+1
	jc dangle_within_limits
	sjmp done_angle

	dangle_within_limits:
		; difference is in range, switch to new angle
		mov a, r7
		clr c
		subb a, r6 ; get offset
		jnb gray_msb, unsign_extend
		sign_extend: ; moving in negative direction
			orl a, #-gray_bits-1 ; logical inversion of gray_bits
			mov r5, #0x80 ; moving in negative direction
			sjmp sign_extend_done
		unsign_extend: ; moving in positive direction
			anl a, #gray_bits
			mov r5, #0x00 ; moving in positive direction
		sign_extend_done:

		setb p1.3
		add a, r4 ; add to current angle
		anl a, #theta_bits
		mov r4, a ; store
		mov a, r7
		mov r6, a ; move new decoded gray code to old

		jb rs0, angle1
		angle0:
			mov a, #0x00
			sjmp angle_done ; send which rotary to PSoC
		angle1:
			mov a, #0x80
		angle_done:
			orl a, r4
			lcall sndchr ; send rotary number and theta to PSoC
		clr p1.3

		; compute dtheta
		mov a, r3
		xrl a, r5
		jnb acc.7, dangle_same_direction
		dangle_reverse_direction:
			mov a, r5
			mov r5, #0x00
			jz positive_counter
			sjmp negative_counter
		dangle_same_direction:
			; calculate 1/dtheta as sign bit, upper 7 bits of r2
			mov a, r3
			anl a, #0xff
			jz positive_dangle
			jnb acc.7, positive_dangle_max
			xrl a, #0xff
			jz negative_dangle
			negative_dangle_max:
				mov r5, #0x80
				sjmp done_dangle
			positive_dangle_max:
				mov r5, #0x7f
				sjmp done_dangle
			negative_dangle:
			positive_dangle:
			mov a, r3
			rlc a ; put sign bit into c
			mov a, r2
			rrc a ; shift and put sign bit into acc.7
			mov r5, a

			done_dangle:
				mov a, r3
				jb acc.7, negative_counter
				positive_counter:
					mov r3, #0x00
					mov r2, #0x01
					sjmp done_reset_counter
				negative_counter:
					mov r3, #0xff
					mov r2, #0xfe
				done_reset_counter:
		mov a, r5
		; would send 1/dtheta*C for some C, but the PSoC now calculates this to reduce serial usage
		;lcall sndchr

	done_angle:
	ret

lcd:
	lcall reset_lcd

	; print theta1 to LCD
	mov dptr, #s_theta1
	lcall print
	mov a, r4
	lcall print_value

	; print space to LCD
	mov a, #' '
	lcall print_chr

	; print dtheta1 to LCD
	; mov dptr, #s_dtheta1
	; lcall print
	; mov a, r5
	; lcall print_value

	; print theta2 to LCD
	mov dptr, #s_theta2
	lcall print
	mov a, 12 ; r4 for register bank 1
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

; send character over serial port
sndchr:
	txloop:
		jnb scon.1, txloop ; wait until previous character has been sent
	clr scon.1
	mov sbuf, a
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
	push dph
	push dpl
	mov dptr, #porta
	movx @dptr, a ; write charater to LCD data port
	mov dptr, #portc
	mov a, #0x50
	movx @dptr, a ; raise enable line
	lcall wait
	mov a, #0x10
	movx @dptr, a ; lower enable line
	pop dpl
	pop dph
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
	push 0
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
	pop 0
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
