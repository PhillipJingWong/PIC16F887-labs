; operating BUILD CONFIGURATIONS drop down menu in the DEBUG toolbar
; FOR SIMULATIONS with MPLAB SIM: select "Debug" this will switch off delays that take thousands of instructions
; HARDWARE: select "Release" all delays will be on

; Provided code - do not edit  
; This include setups up various configuration bits for the microcontroller
; we are using, and importantly reserves space for variables.
; If you need to use more variables, please place them in VAR.INC, which you
; can find under the "Header Files" folder. The variables listed there will be
; placed in the first memory bank.
; This code has been provided for you to simplify your work, but you should be
; aware that you cannot ignore it.
#include	ECH_1.inc
		
;NOTE: THE PIC16 DOESNT HAVE BRANCHES SO GOTOS HAVE BEEN USED THROUGHOUT
; Place your SUBROUTINE(S) (if any) here ...  
;{ 
ISR	;CODE	H'20'

;usually it is recommended to keep isr code as small as possible - but we want to lose repeated interrupts in this case
	
;low priority interrupt for adc readings to minimise wait times	
;high priority interrupt that enables configuring for the current mode 
;interrupts already disabled in entering
	
	;disable interrupts
	BCF INTCON,ADIE
	BCF INTCON,RBIE
	BCF INTCON,GIE
	
	;set registers and choose bank
	;save W and Status
	BANKSEL Temp_W
	MOVWF Temp_W
	SWAPF STATUS,W
	MOVWF Temp_Status
	
I_L1 ;high priority button config interrupt
	BTFSS INTCON,RBIF ;check if button flag set
	GOTO I_L2
	BCF INTCON,RBIF
	BSF value,0 ;set a variable to go into config mode next loop
	
I_L2 ;low priority adc interrupt
	BTFSS PIR1,ADIF ;check if adc flag set
	GOTO I_L3
	BCF PIR1,ADIF
	;CALL ServeAD
	BSF value,1 ;set a variable to change mode to adc value next loop
I_L3
	;restore registers	
	BANKSEL INTCON
	BCF INTCON,0
	BCF PIR1,6 ;clear ADC flag bit
	BCF INTCON,RBIF ;clear button flag
	;reset flag
	BANKSEL PORTD
	
	;restore W and status
	SWAPF Temp_Status,W
	MOVWF STATUS
	SWAPF Temp_W, F
	SWAPF Temp_W, W
	
	retfie	

PRNG ;pseudorandom number generator function
	BCF STATUS,0	;clear carry before shifting
	RRF LFSR_STATE,f
	BTFSS STATUS,0	;test output bit
	return
	
	movfw LFSR_MASK
	XORWF LFSR_STATE
	BSF LFSR_STATE,7  ;insert 1 into msb as lsb was 1
	return

ServeAD ;get adc reading and change display mode (state) if needed
	
	BANKSEL ADRESH
	MOVFW ADRESH
	BANKSEL ADC_HI	
	MOVWF ADC_HI ;store adc reading
	
	subwf ADC_PREV,w ;subtract previous adc reading
	
	btfsc STATUS,Z ;check if previous reading is same as the current
	goto SameReading
	
	;code for if status changed
	movlw h'80'
	subwf ADC_HI,w ;4 -2.5=+1.5
	
	btfss STATUS,C	;negative, new value is less than original
	movlw b'00000010'
	btfsc STATUS,C	;polarity reversed because of 2s complement
	movlw b'00001000' ;assign relevant program state for new value
	btfsc STATUS,Z
	movlw b'00000100' 
	movwf Program_State
	
	movfw ADC_HI
	movwf ADC_PREV ;store new reading as the previous reading
	
	;disable PWM 	
	movlw b'01100001' ;01 P1D, 10 LSB, 11xx chooses pwm
	movwf CCP1CON
	
	;reset LEDS
	movlw d'0'
	movwf PORTD
	
	SameReading ;same reading is the same exit used by not same reading
	
	
	BCF value,1 ;clear variable telling loop to serve the adc reading
	BCF INTCON,ADIF	;clear adc interrupt flag
	BSF INTCON,RBIE ;reenable interrupts
	BSF INTCON,ADIE
	BSF INTCON,GIE
	return
	
	
ConfigSSStrobe ;configure side to side strobe function
	clrf PORTD
	movlw b'01000000' ;set LED to tell user what they are configure
	movwf PORTD
	
	banksel Strobe_Current
	;set starting LED to RD7
	movlw b'10000000'
	movwf Strobe_Current
	
	movlw d'5' ;small delay to ensure program works as intended
	call DelWds
	
	;choose speed of motion first - delay between changing LEDs
	;0.1s,0.5s,1s,5s
	;call Select4
	call Select4
	movwf Strobe_Speed
	
	BTFSC Strobe_Speed,0
	movlw d'1'
	BTFSC Strobe_Speed,1
	movlw d'5'
	BTFSC Strobe_Speed,2
	movlw d'10'
	BTFSC Strobe_Speed,3
	movlw d'50'
	
	movwf Strobe_Speed ;store selected strobe speed
	
	;choose between LED moving back and forth, or one direction only
	call Select4
	movwf Strobe_Direction
	;0 is back and forth, 1 is left to right (forward), 2 is right to left(backwards), 3 is back and forth
	;chosen in the strobe code based on the variable value
	
	movlw d'0'
	movwf value ;set variable so loop doesnt go to configure again
	movlw d'1' ;debounce button
	movwf PORTB
	
	BCF INTCON,RBIF	;clear configure interrupt flag
	BSF INTCON,RBIE ;reenable interrupts
	BSF INTCON,GIE
	
	return		
	
ConfigVPWM ;configure variable pwm function
	;disable PWM while configuring	
	movlw b'01000001' ;01 P1D, 00 clear LSB, 11xx chooses pwm
	movwf CCP1CON
	
	clrf PORTD
	BSF PORTD,7 ;set LED to what is being configured
	
	;solid LED during configuring
	;first choose speed
	;speed increases the delay between each iteration between the brightness values
	;19.61khz
	
	;ensure 2nd brightness is larger: subtract and see if carry is set - if set then need to be subtracted the other way round
	;no difference between both brightness values - doesnt matter what value is set, so use 0.1s

	;diff of 3: 408-0=408 iterations
	;diff of 2: 408-136=272 iterations
	;diff of 1: 408-272=136 iterations
	
	;for 408 iterations in 1s= 0.002s per iteration (0.82s), 3s=0.007s per iteration(2.9s), 5s=0.012s per iteration(4.9s), 10s= 0.025s per iteration (10.2s)
	;for 272 iterations in 1s= 0.004s per iteration (1.088s), 3s=0.011s per iteration(2.992s), 5s=0.018s per iteration(4.9s), 10s= 0.037s per iteration (10.1s)
	;for 136 iterations in 1s= 0.008s per iteration (1.088s), 3s=0.022s per iteration(2.992s), 5s=0.036s per iteration(4.9s), 10s= 0.074s per iteration (10.1s)
	
	;increases by a factor of 4 (408=102 iterations,272=68,136=36 iterations)
	
	;brightness will be increasing when first time selecting new configuration
	movlw d'1'
	movwf Bright_DI ;sets whether to increase or decrease brightness
	
	;user selects speed first but this isnt used until the end when the user has chosen the 
	;brightnesses as speed depends on the brightness difference
	movlw d'5'
	call DelWds ;little delay to ensure works as expected
	
	call Select4 ;get input
	movfw VPWM_Speed ;store
	
	movlw d'5'
	call DelWds
	
	;second: choose brightness of first state
	;Duty cycle ratio= duty cycle value / (4(Pr2+1)), PR2 is 0x65=101
	;Duty cycle percentage/values for brightnesses (set in CCPR1L and LSB of CCP1CON):
	;0%:0 (need to use 1)
	;33.3%:136 = 0b10001000 (136/408)
	;66.6%:272 = 0b100010000 (272/408)
	;100%:408= 0b110011000 (408/408)
	
	call Select4
	movwf Bright_Choice1 ;get the users brightness choice and set the relevant brightness
	
	btfsc Bright_Choice1,0
	goto B1_0
	btfsc Bright_Choice1,1
	goto B1_33
	btfsc Bright_Choice1,2
	goto B1_66
	btfsc Bright_Choice1,3
	goto B1_100
	
	;assign values required for duty cycle percentages that correlate to brightness
	B1_0 ;0 = 0b00000000
	MOVLW b'00000001' ;cant be set to 0 exactly so set to 1
	movwf VPWM_Bright1
	
	movlw d'0'
	movwf Bright_Choice1 ;save denary number of choice
	
	goto NEXT_BRIGHT
	B1_33 ;136 = 0b10001000
	MOVLW b'00100010'
	movwf VPWM_Bright1
	movlw d'1'
	movwf Bright_Choice1 ;save denary number of choice
	goto NEXT_BRIGHT
	B1_66 ;272 = 0b100010000
	MOVLW b'01000100'
	movwf VPWM_Bright1
	movlw d'2'
	movwf Bright_Choice1 ;save denary number of choice
	goto NEXT_BRIGHT
	B1_100 ;408= 0b110011000
	MOVLW b'01100110'
	movwf VPWM_Bright1
	movlw d'3'
	movwf Bright_Choice1 ;save denary number of choice
	NEXT_BRIGHT ;ready to choose brightness 2
	
	;third: choose brightness of second brightness
	call Select4
	movwf Bright_Choice2
	
	btfsc Bright_Choice2,0
	goto B2_0
	btfsc Bright_Choice2,1
	goto B2_33
	btfsc Bright_Choice2,2
	goto B2_66
	btfsc Bright_Choice2,3
	goto B2_100
	
	;assign values required for duty cycle percentages that correlate to brightness
	B2_0 ;0 = 0b00000000
	MOVLW b'00000001'
	movwf VPWM_Bright2
	movlw d'0'
	movwf Bright_Choice2 ;save denary number of choice
	goto BRIGHTS_CHOSEN
	B2_33 ;136 = 0b10001000
	MOVLW b'00100010'
	movwf VPWM_Bright2
	movlw d'1'
	movwf Bright_Choice2 ;save denary number of choice
	goto BRIGHTS_CHOSEN
	B2_66 ;272 = 0b100010000
	MOVLW b'01000100'
	movwf VPWM_Bright2
	movlw d'2'
	movwf Bright_Choice2 ;save denary number of choice
	goto BRIGHTS_CHOSEN
	B2_100 ;408= 0b110011000
	MOVLW b'01100110'
	movwf VPWM_Bright2
	
	movlw d'3'
	movwf Bright_Choice2 ;save denary number of choice
	
	;goto BRIGHTS_CHOSEN ;no need for this code here
	
	BRIGHTS_CHOSEN ;both brightnesses chosen
	
	;code to allocate the speed that the user chose (speed is dependent on brightness values)
	;first calculate the difference between the two brightnesses
	;see comment lines xyz above *quote it
	
	;see if bright2 or bright1 was larger (needed to calculate difference between the two incase of negative)
	movfw Bright_Choice1
	subwf Bright_Choice2,w
	
	BTFSS STATUS,C ;negative polarity due to 2s complement
	goto negative
	
	;save the brightness difference
	movwf Bright_Difference
	goto Allocate_Speed
	
	negative
	movfw VPWM_Bright2
	subwf VPWM_Bright1,w
	
	movfw Bright_Choice2
	subwf Bright_Choice1,w
	
	movwf Bright_Difference
	
	;make it so that the larger value is held in Bright2 variable
	movfw VPWM_Bright1
	movwf Temp
	movfw VPWM_Bright2
	movwf VPWM_Bright1
	movfw Temp
	movwf VPWM_Bright2
	
	movfw Bright_Choice1
	movwf Temp
	movfw Bright_Choice2
	movwf Bright_Choice1
	movfw Temp
	movwf Bright_Choice2
	
	;Set the speed with the known brightness difference
	;diff of 3: 408-0=408 iterations
	;diff of 2: 408-136=272 iterations
	;diff of 1: 408-272=136 iterations
	
	;for 408 iterations in 1s= 0.002s per iteration (0.82s), 3s=0.007s per iteration(2.9s), 5s=0.012s per iteration(4.9s), 10s= 0.025s per iteration (10.2s)
	;for 272 iterations in 1s= 0.004s per iteration (1.088s), 3s=0.011s per iteration(2.992s), 5s=0.018s per iteration(4.9s), 10s= 0.037s per iteration (10.1s)
	;for 136 iterations in 1s= 0.008s per iteration (1.088s), 3s=0.022s per iteration(2.992s), 5s=0.036s per iteration(4.9s), 10s= 0.074s per iteration (10.1s)
	
	Allocate_Speed
	btfsc VPWM_Speed,0
	goto Speed0
	btfsc VPWM_Speed,1
	goto Speed1
	btfsc VPWM_Speed,2
	goto Speed2
	btfsc VPWM_Speed,3
	goto Speed3
	
	;set the speed required depending on the difference between brightnesses (speed is delay between iteration that increments brightness value)
	;speed not allocated if Bright_difference is 0 (delay is set to 1ms)
	
	Speed0
	;brightness difference will only be 1,2 or 3
	;subtract 2 from brightness difference, carry/borrow set means 3(polarity reversed), carry/borrow not set means 1, zero set means 2
	movlw d'2'
	subwf Bright_Difference,w
	btfss STATUS,C
	movlw d'8' ;borrow clear and hence negative (bright diff is 1)
	btfsc STATUS,C
	movlw d'2' ;borrow set and hence positive (bright diff is 3)
	btfsc STATUS,Z
	movlw d'4' ;zero set and hence bright diff is 2
	movwf VPWM_Speed
	goto endconfigpwm
	
	Speed1
	movlw d'2'
	subwf Bright_Difference,w
	btfss STATUS,C
	movlw d'22' ;borrow clear and hence negative (bright diff is 1)
	btfsc STATUS,C
	movlw d'7' ;borrow set and hence positive (bright diff is 3)
	btfsc STATUS,Z
	movlw d'11' ;zero set and hence bright diff is 2
	movwf VPWM_Speed
	goto endconfigpwm
	
	Speed2
	movlw d'2'
	subwf Bright_Difference,w
	btfss STATUS,C
	movlw d'36' ;borrow clear and hence negative (bright diff is 1)
	btfsc STATUS,C
	movlw d'12' ;borrow set and hence positive (bright diff is 3)
	btfsc STATUS,Z
	movlw d'18' ;zero set and hence bright diff is 2
	movwf VPWM_Speed
	goto endconfigpwm
	
	Speed3
	movlw d'2'
	subwf Bright_Difference,w
	btfss STATUS,C
	movlw d'74' ;borrow clear and hence negative (bright diff is 1)
	btfsc STATUS,C
	movlw d'25' ;borrow set and hence positive (bright diff is 3)
	btfsc STATUS,Z
	movlw d'37' ;zero set and hence bright diff is 2
	movwf VPWM_Speed
	goto endconfigpwm
	
endconfigpwm
	;reenable PWM
	movlw b'01101100' ;01 P1D, 10 LSB, 11xx chooses pwm
	movwf CCP1CON
	
	movfw VPWM_Bright1
	movwf CCPR1L ;initialise the initial brightness value
	movlw d'1'
	movwf Bright_DI ;brightness is increasing to begin with
	
	
	movlw d'0'
	movwf value ;dont enter configuration next loop iteration
	movlw d'1' ;debounce button
	movwf PORTB

	BCF INTCON,RBIF	;clear button interrupt flag
	BSF INTCON,RBIE ;reenable interrupts
	BSF INTCON,GIE
	
	return	
	
ConfigLFSR ;configure lfsr
	clrf PORTD
	movlw b'11000000' ;let user know they are configuring lfsr
	movwf PORTD
	
	;choose speed that new values are displayed (set with a delay)
	;0.5s,1s,5s,10s
	call Select4
	movwf LFSR_Speed
	
	BTFSC LFSR_Speed,0
	movlw d'5'
	BTFSC LFSR_Speed,1
	movlw d'10'
	BTFSC LFSR_Speed,2
	movlw d'50'
	BTFSC LFSR_Speed,3
	movlw d'100'
	
	movwf LFSR_Speed
	
	movlw d'0' ;dont enter configuration loop
	movwf value
	movlw d'1' ;debounce button
	movwf PORTB
	
	BCF INTCON,RBIF	;clear button interrupt flag
	BSF INTCON,RBIE ;reenable interrupts
	BSF INTCON,GIE
	
	return	
	
;10 microsecond counter - each cycle is 1us	
Del_us movlw D'1'
	    movwf US_CNT
	us_loop nop
		decfsz US_CNT
		nop
		return	
	
;} end of your subroutines


; Provided code - do not edit  
Main	nop
#include ECH_INIT.inc

; Place your INITIALISATION code (if any) here ...   
;{ ***		***************************************************
; e.g.,		movwf	Ctr1 ; etc

;set starting strobe LED state and direction		
banksel Strobe_Current
movlw b'10000000'
movwf Strobe_Current
movlw d'0'
movwf Strobe_Direction

;enable RB0 interrupt through IOCB register	
BANKSEL IOCB
MOVLW B'00000001'
MOVWF IOCB	
	
BANKSEL ANSEL
BSF INTCON,RBIE ;3 enable portb change interrupt
BSF INTCON,GIE ;enable interrupts
BSF INTCON,PEIE
BSF ANSEL,0 ;a0 as analogue input
	
BANKSEL ADCON0
MOVLW B'01000001'
;MOVLW B'10000001' ;Fosc/32 conv clock select bits
MOVWF ADCON0

;acquisition delay
movlw d'10'
call DelWds
	
movlw b'00000010' ;set the starting program state
movwf Program_State
movlw h'80'
movwf ADC_PREV ;initialise previous detected adc reading for changing between states

movlw d'1'
movwf Strobe_FB	;initialise strobe direction
movlw d'10'
movwf Strobe_Speed ;initialise strobe speed
		
;bcf PIE1,ADIE ;temp disable adc interrupt for testing		
banksel PORTD
movlw b'10000000' ;set starting LED
movwf PORTD
		
movlw d'0'
movfw value ;set starting value for deciding whether to go into configure mode 

movlw b'11110000'     ; set initial value of taps
;movlw b'10001110'
movwf LFSR_MASK

movlw b'10011001'   ;set start value
movwf LFSR_STATE		

movlw d'10'
movwf LFSR_Speed ;set start lfsr speed
	
		
;config pwm for 19.61 khz
BANKSEL TRISC
BCF TRISC,2	

movlw h'65'	
movwf PR2	
	
BANKSEL T2CON
movlw b'00000111' ;at end 1 sets timer2 on, 11 sets prescalar to 16	
movwf T2CON
	
banksel CCPR1L
movlw b'00011000'	
movwf CCPR1L ;MSB for the PWM
	
movlw b'01101100' ;01 P1D, 10 LSB, 11xx chooses pwm
movwf CCP1CON		

;ADC setup
BANKSEL ADCON1
MOVLW B'00000000' ;left justify	
MOVWF ADCON1
BANKSEL TRISA
BSF TRISA,0
			
BSF INTCON,PEIE	
BANKSEL PIE1
bsf PIE1,ADIE		;enable interrupts
		
BANKSEL ADCON0 ;(needs to be set again to function as intended)
MOVLW B'01000001'
;MOVLW B'10000001' ;Fosc/32 conv clock select bits, enable ADC
MOVWF ADCON0

;call Del_ms
;acquisition delay
movlw d'10'
call DelWds
	
movlw h'80'
movwf ADC_PREV ;initialise previous detected adc value

;set initial variable pwm values
movlw d'12'	
movwf VPWM_Speed
movlw b'00000001'
movwf VPWM_Bright1
movwf CCPR1L		
movlw b'01100110'
movwf VPWM_Bright2			
		
movlw d'1'
movwf Bright_DI
;movlw b'01101100' ;01 P1D, 10 LSB, 11xx chooses pwm
;movwf CCP1CON			

;start of the program set the adc flag to select the required mode		
BSF PIR1, ADIF		
		
;} 
; end of your initialisation

MLoop	nop

; place your superloop code here ...  
;{

;set adc to read if it isnt
BANKSEL ADCON0
BTFSC ADCON0,GO
goto SKIPADC
BSF ADCON0,GO	
SKIPADC
		
		
;ADC update
btfss value,1 ;check if adc config variable is set from adc interrupt
goto noadcupdate		
call ServeAD		
		
noadcupdate
		
;enter config mode if needed		
btfss value,0		
goto noconfig
btfsc Program_State,1
call ConfigVPWM
btfsc Program_State,2
call ConfigSSStrobe
btfsc Program_State,3
call ConfigLFSR

;start adc after configuring
BCF ADCON0,GO			
BSF ADCON0,GO	
movlw d'5' ;enough time for the adc to read
call DelWds
BSF INTCON,ADIF ;check for new mode after configuring	
		
;Strobe_FB - denotes whether going forwards or backwards, 0=forwards 1=back		

noconfig		
	
;choose which mode to display
btfsc Program_State,1	
goto DisplayVPWM		
btfsc Program_State,2	
goto DisplaySSStrobe
btfsc Program_State,3	
goto DisplayLFSR

DisplayVPWM ;display variable pwm
bcf CCP1CON,7 ;configure required bits to function properly
bsf CCP1CON,6		
bsf CCP1CON,2
bsf CCP1CON,3
bcf CCP1CON,1
bcf CCP1CON,0		
		
;movlw b'01101100' ;01 P1D, 10 LSB, 11xx chooses pwm
;movwf CCP1CON ;msut be configured individually as seen above to function properly
		
;Using RD0 as the PWM LED, earlier bright2 was set to the larger value
;choose whether to increase or decrease the brightness, initially is increase
		
;movfw Bright_DI ;0 is decrease, 1 is increase
BTFSS Bright_DI,0
goto Decrease_Bright

Increase_Bright	;loop to increase the brightness

;change increase mode to decrease mode as the max has been reached	
movfw CCPR1L
subwf VPWM_Bright2,w 		
btfss STATUS,Z		;check if max has been reached
goto DI_nochange1
bcf Bright_DI,0		;set to decrease mode as max reached
goto MLoop		;reloop

DI_nochange1			
		
;assign current brightness value to pwm (10bit value)
;check whether to change the lsbs or the msb variable

;get the LSBs
movlw b'00110000'
andwf CCP1CON,w
sublw b'00110000' ;compare 11 LSB to 11 
btfss STATUS,Z
;this code executes because LSBs are 11	
goto skip1
movfw CCPR1L
addlw d'1'
movwf CCPR1L
;INCF CCPR1L,f
BCF CCP1CON,5
BCF CCP1CON,4 ;set LSBS to 0	
goto applyvpwmdelay		
skip1		
;LSBS arent 11, hence add 1
btfsc CCP1CON,5
goto inc_lsb10		;number is 10, hence set 4 to make it 11

btfsc CCP1CON,4			;number is 01, hence set 5 and clear 4 to make it 10
goto inc_lsb01

inc_lsb00		
bsf CCP1CON,4		;otherwise number is 00, set 4 to make it 01

goto applyvpwmdelay		
inc_lsb01
BSF CCP1CON,5;number is 01, hence set 5 and clear 4 to make it 10
BCF CCP1CON,4
goto applyvpwmdelay
		
inc_lsb10
bsf CCP1CON,4		;number is 10, hence set 4 to make it 11	
goto applyvpwmdelay
		
;if one is added to 2 lsbs and results in 3, set bottom lsbs to 0 and increment msbs
;for all final pwm brightness values, the lsbs are 0

;for subtracting
;if lsbs are 0, decrement msbs and set lsbs to 2(11), otherwise subtract from lsbs		
		
		
Decrease_Bright ;loop to decrease the brightness

;change decrease mode to increase mode as the max		
movfw CCPR1L
subwf VPWM_Bright1,w 		
btfss STATUS,Z
goto DI_nochange2
bsf Bright_DI,0	   ;set to increase mode
goto MLoop	    ;reloop

DI_nochange2			
		
;get the LSBs
;movfw CCP1CON
;andwf b'00110000'
movlw b'00110000'		
andwf CCP1CON,W
		
sublw b'00000000' ;compare 00 LSB to 00 
btfss STATUS,Z
goto skip2
;this code executes because LSBs are 00	
	
movlw d'1'
subwf CCPR1L,f
;movwf CCPR1L
nop
bsf CCP1CON,5
bsf CCP1CON,4	
;set LSBS to 11

goto applyvpwmdelay
	
skip2		
;this code executes when LSBs arent 00	
;subtract one from LSBs	
;maybe can actually just check if last part is 1 so 11 and 01 group together?
btfss CCP1CON,5			;number is 01, hence clear 4 to make it 00
goto inc_lsb01_2		;should be called sub lsb
		
btfss CCP1CON,4		;number is 10 or 11
goto inc_lsb10_2		;number is 10
;otherwise number is 11
inc_lsb11_2
BCF CCP1CON,4 ;number is 11 so subtract 1 for 10
goto applyvpwmdelay
	
inc_lsb01_2
BCF CCP1CON,4
goto applyvpwmdelay
		
inc_lsb10_2
;number is 10 so subtract 1 for 01		
BCF CCP1CON,5
bsf CCP1CON,4
goto applyvpwmdelay
				
;apply the required delay for the variable pwm
applyvpwmdelay
		
movfw VPWM_Speed
call DelWms
goto MLoop
		
DisplaySSStrobe ;displays the side to side strobe

clrf STATUS		;clear status register eg carry if needed		
movfw Strobe_Current
movwf PORTD		;move current state into the leds
		
btfsc Strobe_Direction,0 ;choose how to move the led next
goto Backandforth
btfsc Strobe_Direction,1		
goto Forwards
btfsc Strobe_Direction,2	
goto Backwards
btfsc Strobe_Direction,3
goto Backandforth		

Backandforth
btfsc Strobe_Current,0 ;if the edge is reached, change direction
goto Strobe_changeto_back
btfsc Strobe_Current,7
goto Strobe_changeto_forward	

;Strobe_FB - denotes whether going forwards or backwards, 0=forwards 1=back		
BCF STATUS,C ;clear the carry
btfsc Strobe_FB,0
RLF Strobe_Current ;shift in direction needed relating to whether to go forwards or back
btfss Strobe_FB,0		
RRF Strobe_Current
movfw Strobe_Speed
call DelWds
goto MLoop
		
Strobe_changeto_forward		
bcf Strobe_FB,0 ;change direction to forwards by clearing
BCF STATUS,C
RRF Strobe_Current ;move the current led state as needed
movfw Strobe_Speed ;apply required delay
call DelWds
goto MLoop ;reloop
		
Strobe_changeto_back
bsf Strobe_FB,0 ;change direction to backwards by setting
BCF STATUS,C
RLF Strobe_Current ;move current led state as needed
movfw Strobe_Speed
call DelWds
goto MLoop		
	
Forwards ;move led forwards
btfsc Strobe_Current,0 ;check if the end has been reached
goto Strobe_forwards_end
BCF STATUS,C
RRF Strobe_Current ;if end isnt reached, move forwards
movfw Strobe_Speed ;load required delay
call DelWds
goto MLoop

;LED needs to rotate around
Strobe_forwards_end		
BCF Strobe_Current,0 ;clear end and set front for a rotation
BSF Strobe_Current,7
movfw Strobe_Speed ;required delay
call DelWds		
goto MLoop	;reloop

Backwards ; move led backwards
btfsc Strobe_Current,7 ;check if end reached
goto Strobe_back_end
BCF STATUS,C
RLF Strobe_Current ;move backwards if end not reached
movfw Strobe_Speed ;delay as required
call DelWds
goto MLoop

;LED needs to rotate around
Strobe_back_end		
BCF Strobe_Current,7 ;clear end and set front to rotate around
BSF Strobe_Current,0
movfw Strobe_Speed ;load required delay
call DelWds		
goto MLoop 

DisplayLFSR ;display the LFSR generated number
movfw LFSR_STATE ;display lfsr current state
movwf PORTD
call PRNG ;call number generator function to modify the lfsr state
movfw LFSR_STATE ;move the generated number into the lfsr state
movfw LFSR_Speed ;load the required delay
call DelWds
goto MLoop	;reloop
		
;}	
; end of your superloop code
	FINAL
    goto	MLoop

end
