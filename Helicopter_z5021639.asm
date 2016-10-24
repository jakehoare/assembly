; Helicopter project AVR code
; Controls a model helicopter using keypad and pushbuttons.
; Position information is displayed on the LCD.
; Motor simulates the rotor speed.
; Refer to project spec, user manual and design manual.

; Author : Harold Jacob Hoare
; Student ID : z5021639
; Date : 30th October 2014

; PIN DESCRIPTIONS
; Connect AVR PC0-PC7 to LCD D0-D7
; Connect AVR PB0-PB3 to LCD RW, DS, RS, BL
; Connect LCD BE pin to ground.

; Connect keypad to AVR A
; Connect LED bar to AVR D
; Connect AVR PB4 to Mot

; Connect AVR PE7 (labelled PE2) to PB0
; Connect AVR PE6 (labelled PE3) to PB1

.include "m64def.inc"

.equ LCD_RW = 0				; LCD Read/Write
.equ LCD_E = 1				; LCD Enable (DS)
.equ LCD_RS = 2				; LCD Register select
.equ LCD_BF = 7				; busy flag
.equ LCD_ID = 1				; increment flag
.equ LCD_N = 3				; lines of display flag
.equ LCD_C = 1				; cursor flag
.equ LCD_B = 0				; cursor blink
.equ LCD_DISP_ON = 0b00001100
.equ LCD_ENTRY_SET = 0b00000100
.equ LCD_DISP_CLR = 0b00000001
.equ LCD_DISP_OFF = 0b00001000
.equ LCD_FUNC_SET = 0b00110000
.equ LCD_READ_MASK = 0b01111111
.equ LCD_TOP_LINE = 0b10000000
.equ LCD_NEXT_LINE = 0b11000000
.equ LCD_DIRECTION = 0b11001010

.equ MIN_X = 0				; room boundary
.equ MAX_X = 50
.equ MIN_Y = 0
.equ MAX_Y = 50
.equ MIN_Z = 0
.equ MAX_Z = 10
.equ MAX_SPEED = 4			; maximum helicopter speed

.equ DIR_LEFT = 0b00000001	; flight directions
.equ DIR_FORWARD = 0b00000010
.equ DIR_UP = 0b00000011
.equ DIR_RIGHT = 0b00000100
.equ DIR_BACK = 0b00000101
.equ DIR_DOWN = 0b00000110

.equ PORTADIR = 0xF0		; PA7-4: output (columns), PA3-0, input (rows)
.equ STARCOLMASK = 0xEF		; scan from left column (eg for the * key)
.equ INITROWMASK = 0x01		; scan from the top row
.equ TAKEOFFHOVERROW = 0x08	; scan the bottom row
.equ HASHCOLMASK = 0xBF		; scan from 3rd left column scan (eg for the # key)
.equ ROWMASK = 0x0F			; for obtaining input from Port A

.equ INITIAL_DUTY_CYCLE = 0x64
.equ DUTY_CYCLE_FACTOR = 0x32
.equ MAX_DUTY_CYCLE = 0xFA

.def del_lo = r16			; delay loop counter
.def del_hi = r17			; delay loop counter
.def row = r18				; current row number (0 to 3)
.def col = r19				; current column number (0 to 3)
.def rmask = r20			; mask for current row during scan
.def cmask = r21			; mask for current column during scan
.def timer_counter = r22	; number of timer2 interrupts
.def tenth_seconds = r23	; number of 0.1 seconds	
.def temp1 = r24			; multi-purpose temporary register
.def temp2 = r25			; multi-purpose temporary register
.def path_time_l = r26		; time on current trajectory in tenth of a second
.def path_time_h = r27		


;************ LCD UTILITIES ********************************************

.MACRO delay
loop:
	ldi temp2, 18
inner_loop:
	dec temp2
	cpi temp2, 0
	brne inner_loop		; taken branch takes two cycles
	subi del_lo, 1		; the del_hi:del_lo register pair store the loop counts
	sbci del_hi, 0
	brne loop 			; taken branch takes two cycles
.ENDMACRO 				; one loop time is 76 cycles = ~10.3us

.MACRO lcd_write_com
	out PORTC, temp2 	; set the data port's value up
	ldi temp1, (0<<LCD_RS)|(0<<LCD_RW)
	out PORTB, temp1 	; RS = 0, RW = 0 for a command write
	nop 				; delay to meet timing (Set up time)
	sbi PORTB, LCD_E 	; turn on the enable pin
	nop 				; delay to meet timing (Enable pulse width)
	nop
	nop
	cbi PORTB, LCD_E 	; turn off the enable pin
	nop 				; delay to meet timing (Enable cycle time)
	nop
	nop
.ENDMACRO

.MACRO lcd_write_data
	out PORTC, temp2 	; set the data port's value up
	ldi temp1, (1 << LCD_RS)|(0<<LCD_RW)
	out PORTB, temp1	; RS = 1, RW = 0 for a data write
	nop 				; delay to meet timing (Set up time)
	sbi PORTB, LCD_E 	; turn on the enable pin
	nop 				; delay to meet timing (Enable pulse width)
	nop
	nop
	cbi PORTB, LCD_E 	; turn off the enable pin
	nop 				; delay to meet timing (Enable cycle time)
	nop
	nop
.ENDMACRO

.MACRO lcd_wait_busy
	clr temp1
	out DDRC, temp1 	; make PORTC be an input port for now
	out PORTC, temp1
	ldi temp1, 1 << LCD_RW
	out PORTB, temp1 	; RS = 0, RW = 1 for a command port read
busy_loop:
	nop 				; delay to meet set-up time
	sbi PORTB, LCD_E 	; turn on the enable pin
	nop 				; delay to meet timing (Data delay time)
	nop
	nop
	in temp1, PINC 		; read value from LCD
	cbi PORTB, LCD_E 	; turn off the enable pin
	sbrc temp1, LCD_BF 	; if the busy flag is set
	rjmp busy_loop 		; repeat command read
	clr temp1 			; else
	out PORTB, temp1 	; turn off read mode,
	ser temp1 ;
	out DDRC, temp1 	; make PORTC an output port again
.ENDMACRO

.MACRO lcd_initialise
	ldi del_lo, low(1500) ;delay (>15ms)
	ldi del_hi, high(1500)
	delay

	ldi temp2, LCD_FUNC_SET | (1 << LCD_N) ; function set 2 line display and 5*7 font. 1st command
	lcd_write_com
	ldi del_lo, low(410) ; delay (>4.1 ms)
	ldi del_hi, high(410)
	delay

	lcd_write_com ; 2nd function set command
	ldi del_lo, low(10) ; delay (>100 ns)
	ldi del_hi, high(10)
	delay
	
	lcd_write_com ; 3rd function set command
	lcd_write_com ; Final function set command

	lcd_wait_busy ; Wait until the LCD is ready
	ldi temp2, LCD_DISP_OFF
	lcd_write_com ; Turn Display off

	lcd_wait_busy ; Wait until the LCD is ready
	ldi temp2, LCD_DISP_CLR
	lcd_write_com ; Clear Display

	lcd_wait_busy ; Wait until the LCD is ready
	ldi temp2, LCD_ENTRY_SET | (1 << LCD_ID)	; Set entry mode: Increment = yes and Shift = no
	lcd_write_com

	lcd_wait_busy ; Wait until the LCD is ready
	ldi temp2, LCD_DISP_ON | (0 << LCD_C)	; Display On command with C = 0 and B = 0
	lcd_write_com
.ENDMACRO

;************ LCD DISPLAY ********************************************

.MACRO lcd_display
lcd_display_loop:
	lcd_wait_busy
	lpm temp2, Z+				; ZH:ZL points to each character of the string in turn
	cpi temp2, 0				; end if we find end of string character
	breq end_of_display
	lcd_write_data				; write the character to the LCD
	rjmp lcd_display_loop
end_of_display:
.ENDMACRO

.MACRO display_new_position
	lcd_wait_busy
	ldi temp2, LCD_NEXT_LINE
	lcd_write_com				; shift to start of second line of LCD display

	ldi ZL, low(clear_posn<<1)
	ldi ZH, high(clear_posn<<1)
	lcd_display					; clear previous position information from the LCD

	lcd_wait_busy
	ldi temp2, LCD_NEXT_LINE
	lcd_write_com				; shift to start of second line of LCD display

	lcd_wait_busy
	ldi temp2, '('
	lcd_write_data				; display opening bracket

	ldi YL, low(x_new)
	ldi YH, high(x_new)
	ld temp1, Y+				; load x_new position

	display_triple_digit		; display x position

	lcd_wait_busy
	ldi temp2, ','
	lcd_write_data				; display comma

	ld temp1, Y+				; load y_new

	display_triple_digit		; display y position

	lcd_wait_busy
	ldi temp2, ','
	lcd_write_data				; display comma

	ld temp1, Y+				; load z_new

	display_triple_digit		; display z position

	lcd_wait_busy
	ldi temp2, ')'
	lcd_write_data				; display closing bracket
.ENDMACRO

.MACRO display_dir_speed
	lcd_wait_busy
	ldi temp2, LCD_DIRECTION	; move the to the LCD memory showing the direction
	lcd_write_com				


	ldi YL, low(direction)		; get direction from data memory
	ldi YH, high(direction)	
	ld temp1, Y
	
display_l:						; display the appropriate letter
	cpi temp1, DIR_LEFT
	brne display_r
	lcd_wait_busy
	ldi temp2, 'L'
	lcd_write_data
	rjmp display_speed
display_r:
	cpi temp1, DIR_FORWARD
	brne display_f
	lcd_wait_busy
	ldi temp2, 'F'
	lcd_write_data
	rjmp display_speed
display_f:
	cpi temp1, DIR_UP
	brne display_b
	lcd_wait_busy
	ldi temp2, 'U'
	lcd_write_data
	rjmp display_speed
display_b:
	cpi temp1, DIR_RIGHT
	brne display_u
	lcd_wait_busy
	ldi temp2, 'R'
	lcd_write_data
	rjmp display_speed
display_u:
	cpi temp1, DIR_BACK
	brne display_d
	lcd_wait_busy
	ldi temp2, 'B'
	lcd_write_data
	rjmp display_speed
display_d:
	cpi temp1, DIR_DOWN
	brne display_speed
	lcd_wait_busy
	ldi temp2, 'D'
	lcd_write_data

display_speed:
	lcd_wait_busy
	ldi temp2, ' '				; display a space
	lcd_write_data

	lcd_wait_busy
	ldi YL, low(speed)
	ldi YH, high(speed)	
	ld temp1, Y
	ldi temp2, '0'				; add ASCII '0' to the integer speed
	add temp2, temp1
	lcd_write_data				; display the ASCII speed
.ENDMACRO


.MACRO display_triple_digit
	clr temp2					; temp2 is high byte input, temp1 is low byte input,
	rcall divide_by_10			; temp2 contains the result, temp1 contains the remainder

	clr r15						; r15 is used for the units if input was 3 digits, else zero
	cpi temp2, 10				; if result is less than 10 then input was 2 decimal digits
	brlo double_digit
	mov r15, r0					; 
	mov temp1, r1				; move first 2 digits back to temp1
	clr temp2					; clear upper byte
	rcall divide_by_10			; and divide by 10 again

double_digit:
	cpi temp2, 0				; do not display tens if they are zero
	breq single_digit

	lcd_wait_busy				; else display tens of double digit
	ldi temp2, '0'
	add temp2, r1
	lcd_write_data

single_digit:
	lcd_wait_busy				; display units of double or single digit
	ldi temp2, '0'
	add temp2, r0
	lcd_write_data

	mov temp1, r15
	cpi temp1, 0				; if we had a 3 digit input then display units or else end
	breq end_triple_display
	lcd_wait_busy
	ldi temp2, '0'
	add temp2, r15
	lcd_write_data
end_triple_display:
.ENDMACRO

;************ KEYPAD UTILITIES ********************************************

.MACRO scan_hash
	ldi cmask, HASHCOLMASK	; column mask for third column
hash_poll:
	out PORTA, cmask		; scan the column

	ldi temp1, 0xFF			; slow down the scan operation
slow_scan:
	dec temp1
	brne slow_scan

	in temp1, PINA			; read PORTA
	andi temp1, TAKEOFFHOVERROW	; get the keypad output value
	cpi temp1, 0			; check if the row is low
	breq end_hash_scan
jmp hash_poll
end_hash_scan:
.ENDMACRO

;************ HELICOPTER UTILITIES ********************************************

.MACRO takeoff_position
	ldi YL, low(x_last)		; pointer to x position	
	ldi YH, high(x_last)	
	ldi temp1, 25
	st Y+, temp1			; initial x position = 25
	st Y+, temp1			; initial y position = 25
	clr temp1
	st Y+, temp1			; initial z position = 0
	ldi temp1, 1
	st Y+, temp1			; initial speed = 1
	ldi temp1, DIR_UP
	st Y, temp1				; initial direction
	ldi YL, low(flight_time)	; pointer to total flight time
	ldi YH, high(flight_time)	
	clr temp1
	st Y+, temp1			; initial time zero
	st Y, temp1				; initial distance zero
.ENDMACRO

.MACRO path_distance
	ldi YL, low(speed)		; pointer to speed
	ldi YH, high(speed)	
	ld temp1, Y
	mul temp1, path_time_h	; multiply speed by high byte number of tenth seconds
	mov r2, r0				; store low byte of result in r2
	mul temp1, path_time_l	; multiply speed by low byte number of tenth seconds
	add r1, r2
	mov temp2, r1
	mov temp1, r0
	call divide_by_10		; divide by 10 to get distance on path
	mov r0, r1				; move result into r0
.ENDMACRO

.MACRO copy_positions
	ldi YL, low(x_last)		; Y points to the last position
	ldi YH, high(x_last)	
	ldi ZL, low(x_new)		; Z points to the new position
	ldi ZH, high(x_new)	
	clr temp2
copy_positions:
	cpi temp2, 3			; loop 3 times for x, y, z
	breq positions_copied
	ld temp1, Y+			; load the last position
	st Z+, temp1			; and store it as the new position
	inc temp2
	rjmp copy_positions
positions_copied:
.ENDMACRO

.MACRO copy_back
	ldi YL, low(x_last)		; Y points to the last position
	ldi YH, high(x_last)	
	ldi ZL, low(x_new)		; Z points to the new position
	ldi ZH, high(x_new)	
	clr temp2
back_copy:
	cpi temp2, 3			; loop 3 times for x, y, z
	breq copied_back
	ld temp1, Z+			; load the new position
	st Y+, temp1			; and store it as the last position
	inc temp2
	rjmp back_copy
copied_back:
.ENDMACRO

.MACRO update_new_positions
	ldi YL, low(direction)	; get direction from data memory
	ldi YH, high(direction)	
	ld temp1, Y
	
	cpi temp1, 1			; add or subtract path distance from relevant x, y, or z
	breq flying_x_plus
	cpi temp1, 2
	breq flying_y_plus
	cpi temp1, 3
	breq flying_z_plus
	cpi temp1, 4
	breq flying_x_minus
	cpi temp1, 5
	breq flying_y_minus
	cpi temp1, 6
	breq flying_z_minus
	rjmp end_position_update

flying_x_plus:
	ldi YL, low(x_new)
	ldi YH, high(x_new)
	ld temp1, Y
	add temp1, r0
	st Y,temp1
	rjmp end_position_update
flying_x_minus:
	ldi YL, low(x_new)
	ldi YH, high(x_new)
	ld temp1, Y
	sub temp1, r0
	st Y,temp1
	rjmp end_position_update
flying_y_plus:
	ldi YL, low(y_new)
	ldi YH, high(y_new)
	ld temp1, Y
	add temp1, r0
	st Y,temp1
	rjmp end_position_update
flying_y_minus:
	ldi YL, low(y_new)
	ldi YH, high(y_new)
	ld temp1, Y
	sub temp1, r0
	st Y,temp1
	rjmp end_position_update
flying_z_plus:
	ldi YL, low(z_new)
	ldi YH, high(z_new)
	ld temp1, Y
	add temp1, r0
	st Y,temp1
	rjmp end_position_update
flying_z_minus:
	ldi YL, low(z_new)
	ldi YH, high(z_new)
	ld temp1, Y
	sub temp1, r0
	st Y,temp1
end_position_update:
.ENDMACRO

.MACRO update_total_distance
	ldi YL, low(flight_distance)	; load the previous flight_distance
	ldi YH, high(flight_distance)	
	ld temp1, Y
	add temp1, r0			; add distance on the path that has just finished (r0)
	st Y, temp1				; and save back to memory
.ENDMACRO

.MACRO fly_to_land
	path_distance			; calc distance on current path (in r0)
	update_total_distance	; update the overall flight distance
	copy_positions			; copy x_last etc to x_new etc
	update_new_positions	; update x_new etc for latest position (adding r0 the current direction)
	copy_back				; copy x_new etc to x_last etc

	ldi YL, low(speed)		; set speed to 1
	ldi YH, high(speed)
	ldi temp1, 1
	st Y, temp1
	ldi YL, low(direction)	; set direction to down
	ldi YH, high(direction)
	ldi temp1, DIR_DOWN
	st Y, temp1
	ldi temp1, INITIAL_DUTY_CYCLE	; adjust PWM duty cycle corresponding to 1m/s speed
	out OCR0, temp1
	clr path_time_l			; restart timer for new path
	clr path_time_h
.ENDMACRO


;************ INTERRUPT VECTOR ****************************************

	jmp RESET		; initialisation
	jmp DEFAULT		; no handling for Ext Int0
	jmp DEFAULT		; no handling for Ext Int1	 
	jmp DEFAULT		; no handling for Ext Int2	 
	jmp DEFAULT		; no handling for Ext Int3	 
	jmp DEFAULT		; no handling for Ext Int4
	jmp DEFAULT		; no handling for Ext Int5	 
	jmp DEFAULT		; no handling for Ext Int6	 
	jmp DEFAULT		; no handling for Ext Int7	 
	jmp DEFAULT		; no handling for Timer2 compare/match
	jmp T2_OVERFLOW	; handling for Timer2 overflow, to count 0.5 seconds

DEFAULT:
	reti

;************ INITIALISATION ****************************************

RESET:
	ldi temp1, low(RAMEND)	; Initialize the stack
	out SPL, temp1
	ldi temp1, high(RAMEND)
	out SPH, temp1

	ser temp1				; Set port directions			
	out DDRB, temp1			; PORTB is output for LCD controls and pin4 for PWM
	out DDRC, temp1			; PORTC is output for LCD display
	out DDRD, temp1			; PORTD is output for LED bar
	ldi temp1, PORTADIR		; PORTA7:4 columns output, PORTA3:0 rows input
	out DDRA, temp1
	clr temp1
	out DDRE, temp1			; PORTE is input for pushbuttons

	lcd_initialise

	ldi ZL, low(start_message<<1)
	ldi ZH, high(start_message<<1)
	lcd_display				; display the start message on the LCD

	scan_hash				; check for # key pressed to indicate takeoff

	ldi temp1, MAX_DUTY_CYCLE		; start the motor at max duty (to overcome starting inertia)
	out OCR0, temp1
	ldi temp1, (1<< WGM00)|(1<<COM01)|(1<<CS00)	; set Timer0 to Phase Correct PWM mode.
	out TCCR0, temp1

	takeoff_position		; set the helicopter initial location and speed
	clr path_time_l
	clr path_time_h

	lcd_wait_busy
	ldi temp2, LCD_DISP_CLR
	lcd_write_com

	ldi ZL, low(flight_message<<1)
	ldi ZH, high(flight_message<<1)
	lcd_display				; display the LCD header

	lcd_wait_busy
	ldi temp2, LCD_NEXT_LINE
	lcd_write_com			; shift to start of second line of LCD display

	ldi ZL, low(flight_start<<1)
	ldi ZH, high(flight_start<<1)
	lcd_display				; display the initial speed and direction

	ldi temp1, 0b00000011	; Initilaise timer2 interrupts
	out TCCR2, temp1 		; Prescaling value = 64 = 450 interrupts per second
	ldi temp1, 1<<TOIE2 
	out TIMSK, temp1 		; T/C2 interrupt enable to refresh every 0.1 seconds

	ldi temp1, INITIAL_DUTY_CYCLE	; adjust PWM duty cycle corresponding to 1m/s speed
	out OCR0, temp1
	sei						; enable global interrupt
	clr timer_counter
	clr tenth_seconds

	ldi del_lo, low(30000) 	; delay to stop double keypress being read
	ldi del_hi, high(30000)	
	delay	
	
;************ MAIN ROUTINE ****************************************

; POLLING FOR SPEED CHANGE OR HOVERING OR LANDING
main:
	ldi cmask, STARCOLMASK	; column mask for first column
	out PORTA, cmask		; scan the column

	ldi temp1, 0xFF			; slow down the scan operation
slower_scan:
	dec temp1
	brne slower_scan

	in temp1, PINA			; read PORTA
	andi temp1, TAKEOFFHOVERROW	; get the keypad output value
	cpi temp1, 0			; check if the row is low
	brne check_landing

	path_distance			; if * (hover) pressed calc distance on current path
	clr temp1				; set speed to zero
	st Y, temp1				
	rjmp speed_change		

check_landing:
	ldi cmask, HASHCOLMASK	; column mask for third column
	out PORTA, cmask		; scan the column

	ldi temp1, 0xFF			; slow down the scan operation
slowing_scan:
	dec temp1
	brne slowing_scan

	in temp1, PINA			; read PORTA
	andi temp1, TAKEOFFHOVERROW	; get the keypad output value
	cpi temp1, 0			; check if the row is low
	brne check_PB0

	rjmp landing_trajectory

check_PB0:
	sbic PINE, 7			; if PB0 not pressed check PB1
	rjmp check_PB1
	ldi YL, low(speed)		; if PB0 pressed load speed
	ldi YH, high(speed)	
	ld temp1, Y
	cpi temp1, MAX_SPEED	; compare to max speed
	brsh check_PB1			; if same or higher then do not increase
	path_distance			; if less then calc distance on path in r0
	ldi YL, low(speed)		; then increase speed
	ldi YH, high(speed)	
	ld temp1, Y
	inc temp1				 
	st Y, temp1				; and save back to memory
	rjmp speed_change

check_PB1:
	sbic PINE, 6			; if PB1 not pressed, start loop again
	rjmp main
	path_distance			; if PB1 pressed calc distance on path in r0
	ldi YL, low(speed)		; then decrease speed
	ldi YH, high(speed)	
	ld temp1, Y
	dec temp1
	st Y, temp1				; and save back to memory

; A BUTTON OR KEY HAS BEEN PRESSED

speed_change:

	update_total_distance	; update total flight distance
	copy_positions			; copy x_last etc to x_new etc
	update_new_positions	; update x_new etc for latest position (adding r0 the current direction)
	copy_back				; copy x_new etc to x_last etc
	clr path_time_l			; restart timer for new path
	clr path_time_h

	ldi del_lo, low(19400) 	; 0.2s is the time taken to change speed
	ldi del_hi, high(19400)	; = 19400 loops of 10.308 microseconds
	delay		
	
	ldi YL, low(speed)		; if speed is zero then go to hovering routine
	ldi YH, high(speed)	
	ld temp1, Y
	cpi temp1, 0
	breq hovering
							
	ldi temp2, DUTY_CYCLE_FACTOR	; speed is not zero so adjust duty cycle for motor
	inc temp1				; Duty cycle = (1 + speed) * factor
	mul temp1, temp2			 
	out OCR0, r0
							
	rjmp main

; HOVERING - POLL FOR NEW DIRECTION INPUT OR START FLYING

hovering:
	ldi cmask, STARCOLMASK	; initial column mask
	clr col					; initial column

check_PB0_rotor:
	sbic PINE, 7				; if PB0 not pressed check PB1
	rjmp check_PB1_rotor
	in temp1, OCR0				; PB0 pressed, load current rotor speed
	cpi temp1, MAX_DUTY_CYCLE	; compare to maximum duty cycle
	breq check_PB1_rotor		; if same then do not increase
	ldi temp2, DUTY_CYCLE_FACTOR; if less, then increase duty cycle
	add temp1, temp2
	out OCR0, temp1 			; update Timer0 duty cycle
	ldi del_lo, low(30000) 	; delay to stop double keypress being read
	ldi del_hi, high(30000)
	delay

check_PB1_rotor:
	sbic PINE, 6				; if PB1 not pressed, check keypad
	rjmp colloop
	in temp1, OCR0				; PB1 pressed, load current rotor speed
	cpi temp1, INITIAL_DUTY_CYCLE	; compare to minimum duty cycle
	breq colloop				; if same then move on to check keypad
	subi temp1, DUTY_CYCLE_FACTOR	; if more, then decrease duty cycle
	out OCR0, temp1 			; update Timer0 duty cycle
	ldi del_lo, low(30000) 	; delay to stop double keypress being read
	ldi del_hi, high(30000)
	delay

colloop:
	cpi col, 3				; scan 3 columns only ignoring the RHS column
	breq hovering			; if all keys are scanned, repeat
	out PORTA, cmask		; otherwise, scan a column

	ldi temp1, 0xFF			; slow down the scan operation
wait:
	dec temp1
	brne wait

	in temp1, PINA			; read PORTA
	andi temp1, ROWMASK		; get the keypad output value
	cpi temp1, 0xF			; check if any row is low
	breq nextcol			; if yes, find which row is low

	ldi rmask, INITROWMASK	; initialize for row check
	clr row

rowloop:
	cpi row, 4
	breq nextcol			; the row scan is over.
	mov temp2, temp1
	and temp2, rmask		; check un-masked bit
	breq keypress			; if bit is clear, the key is pressed
	inc row					; else move to the next row
	lsl rmask
	jmp rowloop

nextcol:					; if row scan is over
	lsl cmask
	inc col					; increase column value
	jmp colloop				; go to the next column

keypress:
	mov temp1, row			; for numbers from 1-9
	lsl temp1
	add temp1, row
	add temp1, col			; temp1 = row*3 + col + 1
	inc temp1

	cpi temp1, 10			; check if * is pressed
	breq restart_flight		; if so restart flight with current settings

	cpi temp1 , 12			; check if # is pressed
	breq landing_trajectory	; if so restart flight downwards at 1 m/s

	cpi temp1, 7			; if number is from 1 to 6
	brlo new_direction		; then set the new direction
	rjmp hovering			; else continue hovering

new_direction:
	ldi YL, low(direction)	; else set new direction
	ldi YH, high(direction)	
	st Y, temp1

	rjmp hovering			; and check again for direction or * to start moving or rotor change


restart_flight:
	clr temp2
find_rotor_speed:			; convert the rotor speed into a speed level for the flight restart
	ldi temp1, DUTY_CYCLE_FACTOR
	inc temp2				; temp2 will be the flight speed
	mul temp1, temp2
	add r0, temp1
	in temp1, OCR0			; load current rotor speed
	cp r0, temp1
	breq set_new_speed		; flight speed corresponds to current rotor speed
	rjmp find_rotor_speed	; else go back an increment the flight speed to be checked again

set_new_speed:
	ldi YL, low(speed)
	ldi YH, high(speed)
	st Y, temp2
	
	clr path_time_l			; restart timer for new path
	clr path_time_h
	ldi del_lo, low(30000) 	; delay to stop double keypress (back to hover)
	ldi del_hi, high(30000)
	delay		

	jmp main				; jump back to start of main routine

landing_trajectory:
	fly_to_land				; fly down at 1m/s

	jmp main				; jump back to start of main routine


;************ TIMER2 INTERRUPT ****************************************

T2_OVERFLOW:
	push temp1				; save conflict registers onto the stack
	in temp1, SREG
	push temp1
	push temp2
	push ZH
	push ZL
	push YH
	push YL
	push r1
	push r0

	inc timer_counter

	cpi timer_counter, 45	; count 45 timer2 interrupts for 0.1 seconds
	breq tenth_second_passed
	jmp t2_epilogue

tenth_second_passed:
	adiw path_time_h:path_time_l, 1
	inc tenth_seconds
	clr timer_counter		; reset time_counter

	path_distance			; store distance on current path in r0

	copy_positions			; copy x_last etc to x_new etc

	update_new_positions	; update x_new etc for latest position
	
test_if_crash:
	ldi YL, low(x_new)		; test if we have reached any of the 6 room boundaries
	ldi YH, high(x_new)
	ld temp1, Y+
	cpi temp1, MIN_X
	brlt crashed
	breq crashed
	cpi temp1, MAX_X
	brsh crashed

	ld temp1, Y+
	cpi temp1, MIN_Y
	brlt crashed
	breq crashed
	cpi temp1, MAX_Y
	brsh crashed

	ld temp1, Y
	cpi temp1, MAX_Z
	brsh crashed
	cpi temp1, MIN_Z		; test if reached floor
	brlt crash_or_land		; potentially crash if speed too high else landed
	breq crash_or_land

	rjmp check_second		; if not crashed or landed, test if we need to update the LCD display

crash_or_land:
	ldi YL, low(direction)
	ldi YH, high(direction)
	ld temp1, Y
	cpi temp1, 6
	breq test_speed			; if travelling down, then check how fast
	rjmp check_second		; if not travelling downwards
test_speed:
	ldi YL, low(speed)
	ldi YH, high(speed)
	ld temp1, Y
	cpi temp1, 3			
	brsh crashed			; if speed >= 3 then crash
	rjmp landed				; else helicopter has landed

crashed:
	display_new_position

	lcd_wait_busy
	ldi temp2, LCD_TOP_LINE
	lcd_write_com				; shift to start of first line of LCD display

	ldi ZL, low(crash_message<<1)
	ldi ZH, high(crash_message<<1)
	lcd_display					; display the crash message

	lcd_wait_busy
	ldi temp2, LCD_DIRECTION
	lcd_write_com				; shift to display the current direction

	ldi ZL, low(blanks<<1)
	ldi ZH, high(blanks<<1)
	lcd_display					; clears direction and speed

	ldi temp1, 0<<TOIE2 
	out TIMSK, temp1 			; cancel T/C2 interrupt

	clr temp1					; disable Timer0 Phase Correct PWM
	out TCCR0, temp1

	ser temp1
	out PORTD, temp1			; set LED bars

led_flash:
	ldi del_lo, low(48505)		; 97010 cycles of 10.308 microseconds = 1 second
	ldi del_hi, high(48505)
	delay
	ldi del_lo, low(48505)
	ldi del_hi, high(48505)
	delay

	com temp1
	out PORTD, temp1			; switch LED bar pattern
	rjmp led_flash				; loop forever to flash LED bar

landed:
	ldi temp1, 0<<TOIE2 
	out TIMSK, temp1 			; cancel T/C2 interrupt

	clr temp1					; disable Timer0 Phase Correct PWM
	out TCCR0, temp1

	lcd_wait_busy
	ldi temp2, LCD_DISP_CLR
	lcd_write_com

	ldi ZL, low(landed_distance<<1)	; display the LCD header
	ldi ZH, high(landed_distance<<1)
	lcd_display

	path_distance			; get distance on path, keep in r0
	update_total_distance

	display_triple_digit

	lcd_wait_busy				; display 'm' for meters
	ldi temp2, 'm'
	lcd_write_data

	lcd_wait_busy
	ldi temp2, LCD_NEXT_LINE
	lcd_write_com			; shift to start of second line of LCD display

	ldi ZL, low(landed_duration<<1)	; display the LCD header
	ldi ZH, high(landed_duration<<1)
	lcd_display

	ldi YL, low(flight_time)	; pointer to total flight time in seconds
	ldi YH, high(flight_time)
	ld temp1, Y

	display_triple_digit

	lcd_wait_busy				; display 's' for seconds
	ldi temp2, 's'
	lcd_write_data

landed_loop:
	rjmp landed_loop


check_second:
	cpi tenth_seconds, 10		; test if one second has passed
	brne check_half_second
	ldi YL, low(flight_time)	; if so increase count of seconds total flight time
	ldi YH, high(flight_time)	
	ld temp1, Y
	inc temp1
	st Y, temp1
	clr tenth_seconds
	rjmp refresh_lcd_posn

check_half_second:
	cpi tenth_seconds, 5		; if 0.5 seconds has passed we update the LCD
	breq refresh_lcd_posn
	rjmp t2_epilogue			; otherwise end the interrupt

refresh_lcd_posn:
	display_new_position
	display_dir_speed


t2_epilogue:
	pop r0
	pop r1
	pop YL
	pop YH
	pop ZL
	pop ZH
	pop temp2		
	pop temp1
	out SREG, temp1
	pop temp1
	reti

;************ DIVIDE FUNCTION ****************************************

divide_by_10: 			; temp2:temp1 is the 2 byte input
	push r28 			; prologue, save r29:r28 in the stack
	push r29
	push r20 			; save conflict registers used in the function body
	push r21

	in r28, SPL 		; initialize the stack frame pointer
	in r29, SPH
	out SPH, r29 		; update the stack pointer to the new stack top
	out SPL, r28		; end of prologue

	clr r20				; count the 10s
	clr r21				; zero
start_division:
	cpi temp1, 10  		; compare low byte to 10
	cpc temp2, r21		; compare high byte with carry to zero
	brlo end_division	; finish if lower
	sbiw temp2:temp1, 10	; else subtract another 10
	inc r20				; and increment the 10s count
	rjmp start_division
end_division:
	mov temp2, r20		; result in stored temp2, remainder is in temp1
	mov r0, temp1		; copy of the remainder
	mov r1, temp2		; copy of the result

	out SPH, r29		; epilogue, restore registers
	out SPL, r28
	pop r21
	pop r20				
	pop r29
	pop r28
	ret 

;************ CODE MEMORY STATIC DATA *************************************

start_message: .DB "Start:", 0
flight_message: .DB "pos      dir spd", 0
flight_start: .DB "(25,25,0) U 1m/s", 0
crash_message: .DB "Crash position: ", 0
blanks: .DB "      ", 0
clear_posn: .DB "          ", 0
landed_distance: .DB "Distance: ", 0
landed_duration: .DB "Duration: ", 0

;************ DATA MEMORY VARIABLES ***************************************

.DSEG
x_last:  .BYTE 1
y_last:  .BYTE 1
z_last:  .BYTE 1
speed:	.BYTE 1
direction:	.BYTE 1
x_new:	.BYTE 1
y_new:	.BYTE 1
z_new:	.BYTE 1
flight_time:	.BYTE 1
flight_distance:.BYTE 1
