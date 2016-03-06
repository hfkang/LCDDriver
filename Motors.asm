    #include <p18f4620.inc>
    #include <MACROS.inc>
    #include <Constants.inc>

    extern LeftL, LeftH ,RightL,RightH
    extern DELAY_ROUTINE
    


   
    udata
offset	    res	    1 
direction   res	    1
    
    code
    global	CONFIG_PWM, STEPPER, REVERSE,FORWARD, PID, direction
    
CONFIG_PWM			;ccp1 : LEFT CCP2: RIGHT 
	movlw	    B'01111111'	;configure PWM Period 
	movwf	    PR2
	;configure CCP Module #1
	movlw	    DutyDefault	;configure duty cycle
	movwf	    LeftSpeed
	movlw	    B'01001100'	;configure duty cycle and RD5 forward, RD7 rev
	movwf	    CCP1CON		
       ;configure CCP Module #2
	movlw	    DutyDefault	;configure CCPR2L
	movwf	    RightSpeed
	movlw	    B'00001100'	;configure CCP2CON
	movwf	    CCP2CON
	movlw	    B'00000101'	;configure T2CON, set prescaler to 4
	movwf	    T2CON
	return 
	
PID
	movf	    LeftH,W		;copy Left to Wreg
	cpfseq	    RightH
	bra	    fine
coarse	nop
	
fine	movf	    LeftL,W
	subwf	    RightL,W
	movwf	    offset		;store in offset variable 
	rrncf	    offset		;divide by four
	rrncf	    offset
	bn	    left
	bra	    right
	
right	movf	    offset,W
	addwf	    DutyDefault,W
	movwf	    LeftSpeed
	movf	    offset,W
	subwf	    DutyDefault,W	; Wreg = default - offset 
	movwf	    RightSpeed
	return
	
left	movf	    offset,W
	addwf	    DutyDefault,W
	movwf	    RightSpeed
	movf	    offset,W
	subwf	    DutyDefault,W	; Wreg = default - offset 
	movwf	    LeftSpeed
	return 
    
REVERSE
	bcf	    T2CON,2		;stop timer2, and the PWM outputs
	bsf	    CCP1CON,7	;change to revserse on full bridge out (RD5)
	bsf	    T2CON,2		;start timer2, and renable PWM
	bsf	    direction,0		;set direction bit in direction register 
	return 

FORWARD
	bcf	    T2CON,2		;stop timer2, and the PWM outputs
	bcf	    CCP1CON,7	;change to revserse on full bridge out (RD5)
	bsf	    T2CON,2		;start timer2, and renable PWM
	bcf	    direction,0		;set direction bit in direction register 
	return 

       
STEPPER
	bsf		STEP
	delay		0x2		;gives 15 us delay 
	bcf		STEP
	return
	end