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
    
CONFIG_PWM			
	;movlw	    B'01111111'	;configure PWM Period 
	movlw	    DutyDefault
	movwf	    PR2
	;configure CCP Module #1
	movlw	    DutyDefault	;configure duty cycle
	movwf	    LeftSpeed
	movlw	    B'00111100'	;configure duty cycle and RD5 forward, RD7 rev
	movwf	    CCP1CON		
       ;configure CCP Module #2
	movlw	    DutyDefault	;configure CCPR2L
	movwf	    RightSpeed
	movlw	    B'00111100'	;configure CCP2CON
	movwf	    CCP2CON
	movlw	    B'00000101'	;configure T2CON, set prescaler to 4
	movwf	    T2CON
	return 
	
PID
	movlf	    DutyDefault,LeftSpeed	;reset speeds to be equal
	movlf	    DutyDefault,RightSpeed
	return 
	
	movf	    LeftH,W		;copy Left to Wreg
	cpfseq	    RightH
	bra	    coarse
	bra	    fine
	
coarse	
	movlf	    0x70,offset
	movf	    LeftH,W
	cpfslt	    RightH
	bra	    right		;left > right, turn left
	comf	    offset
	incf	    offset
	bra	    left
	
	
fine	movf	    LeftL,W
	subwf	    RightL,W
	movwf	    offset		;store in offset variable 
	
	movf	    LeftL,W
	cpfslt	    RightL
	bra	    right		;left > right, turn left
	bra	    left
	
right	divby8	    offset
	btfsc	    direction,0
	bra	    rback		;when direction 1 = backward, don't skip, go to backward mathod. 
	movf	    offset,W
	addwf	    LeftSpeed
	movf	    offset,W
	subwf	    RightSpeed
	return
rback	movf	    offset,W
	addwf	    RightSpeed
	movf	    offset,W
	subwf	    LeftSpeed
	return
	
left	comf	    offset
	incf	    offset
	divby8	    offset
	btfsc	    direction,0
	bra	    lback
	movf	    offset,W
	subwf	    LeftSpeed
	movf	    offset,W
	addwf	    RightSpeed
	return 
lback	movf	    offset,W
	subwf	    RightSpeed
	movf	    offset,W
	addwf	    LeftSpeed
	return 
    
REVERSE
	stopPWM	
	delay	    0xFF
	delay	    0xFF
	delay	    0xFF
	delay	    0xFF
	bcf	    T2CON,2		;stop timer2, and the PWM outputs
	;bsf	    CCP1CON,7	;change to revserse on full bridge out (RD5)
	bsf	    T2CON,2		;start timer2, and renable PWM
	bsf	    direction,0		;set direction bit in direction register 
	bsf	    LeftDirection
	bsf	    RightDirection
	startPWM
	return 

FORWARD
	stopPWM	
	delay	    0xFF
	delay	    0xFF
	delay	    0xFF
	delay	    0xFF
	bcf	    T2CON,2		;stop timer2, and the PWM outputs
	;bcf	    CCP1CON,7	;change to revserse on full bridge out (RD5)
	bsf	    T2CON,2		;start timer2, and renable PWM
	bcf	    direction,0		;set direction bit in direction register 
	bcf	    LeftDirection
	bcf	    RightDirection
	startPWM 
	return 

       
STEPPER
	bsf		STEP
	delay		0x2		;gives 15 us delay 
	bcf		STEP
	return
	end