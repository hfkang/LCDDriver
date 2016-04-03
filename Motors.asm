    #include <p18f4620.inc>
    #include <MACROS.inc>
    #include <Constants.inc>

    extern LeftL, LeftH ,RightL,RightH,threshH,threshL
    extern DELAY_ROUTINE,ErrorState
    extern  mypidStat1,mypidOut0,mypidOut1,mypidOut2, PidMain
    extern  pidStat1,pidOut0,pidOut1,pidOut2
    extern  dispOperationData,dispCorrection,stepsH,stepsL
    
    
    #define pid_sign	    7


   
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
	
PID:	movlf	    DutyDefault,LeftSpeed	;reset speeds to be equal
	movlf	    DutyDefault,RightSpeed
	call	    PidMain
	banksel	    mypidStat1

	movff	    pidStat1,mypidStat1
	movff	    pidOut0,mypidOut0
	movff	    pidOut1,mypidOut1
	movff	    pidOut2,mypidOut2
	
	movff	    mypidOut2,offset	
	
	btfsc	    mypidStat1,pid_sign		;execute positive direction: turn right
	call	    turnRight
	btfss	    mypidStat1,pid_sign
	call	    turnLeft	   
	
	clrf	    threshH
	movlf	    Duty50,threshL
	comp16	    mypidOut1,mypidOut2,pid_overflow,pid_normal,pid_normal
	
pid_normal:
	startPWM
	bra	    pid_end
	
pid_overflow:
	stopPWM
	bcf	    LeftMotor
	bcf	    RightMotor
	;call	    dispOperationData
	;call	    dispCorrection
failure	;bra	    failure
	
	btfsc	    mypidStat1,pid_sign		;execute positive direction: turn right
	bra	    forceRight
	btfss	    mypidStat1,pid_sign
	bra	    forceLeft	

forceRight
	clrf	    RightSpeed
	movlf	    DutyDefault,LeftSpeed
	bsf	    LeftMotor
	bcf	    RightMotor
	
	bra	    pid_end
	
forceLeft
	clrf	    LeftSpeed
	movlf	    DutyDefault,RightSpeed
	bcf	    LeftMotor
	bsf	    RightMotor
	bra	    pid_end
	
pid_end:
	banksel	    mypidStat1
	return 
	
	
turnRight:
	
	movf	    offset,W
	subwf	    RightSpeed
	;movf	    offset,W
	;addwf	    LeftSpeed

	;toggleMotor LeftConfig, disablePWM
	;bsf	    LeftMotor 

	return
turnLeft:
	;1FFE
	;movlw	    0x1F
	;addwf	    mypidOut1	
	;movlw	    0xFF
	;addwf	    mypidOut2
	
	;comf	    mypidOut1
	;comf	    mypidOut2
	;add16	    mypidOut1,mypidOut2,1
	
	movf	    offset,W
	subwf	    LeftSpeed

	;toggleMotor RightConfig, disablePWM
	;bsf	    RightMotor 
	
	return
	

	
old_PID
	movlf	    DutyDefault,LeftSpeed	;reset speeds to be equal
	movlf	    DutyDefault,RightSpeed
	;movf	    LeftH,W			;copy Left to Wreg
	;cpfseq	    RightH
	;goto	    coarse
	movlf	    0xFF,threshH
	movlf	    0x00,threshL
	comp16	    RightH,RightL,negpid,fine,negpid
	movlf	    0xFF,threshH
	movlf	    0x00,threshL
	comp16	    LeftH,LeftL,negpid,fine,negpid
	
	bra	    fine
	
negpid
	clrf	    offset
	movlw	    0xFF
	cpfslt	    RightH 
	call	    rightneg
	
	cpfslt	    LeftH
	call	    leftneg
	
	movff	    LeftH,threshH
	movff	    LeftL,threshL
	comp16	    RightH,RightL,left,right,endpid
	
	;branching to left and right goes here
	
coarse	
	movlf	    0x70,offset
	movf	    LeftH,W
	cpfslt	    RightH
	bra	    right			;left > right, turn left
	comf	    offset
	incf	    offset
	bra	    left
	
	
fine	movf	    LeftL,W
	subwf	    RightL,W
	movwf	    offset			;store in offset variable 
	
	
	movf	    LeftL,W
	cpfslt	    RightL
	bra	    right			;left > right, turn left
	bra	    left
	
right	divby8	    offset
	btfsc	    direction,0
	bra	    rback			;when direction 1 = backward, don't skip, go to backward mathod. 
	movf	    offset,W
	addwf	    LeftSpeed
	movf	    offset,W
	subwf	    RightSpeed
	goto	    endpid
rback	movf	    offset,W
	addwf	    RightSpeed
	movf	    offset,W
	subwf	    LeftSpeed
	goto	    endpid
	
left	comf	    offset
	incf	    offset
	divby8	    offset
	btfsc	    direction,0
	bra	    lback
	movf	    offset,W
	subwf	    LeftSpeed
	movf	    offset,W
	addwf	    RightSpeed
	goto	    endpid 
lback	movf	    offset,W
	subwf	    RightSpeed
	movf	    offset,W
	addwf	    LeftSpeed
	goto	    endpid 
	
endpid 
	movlw	    Duty50
	cpfsgt	    RightSpeed
	clrf	    RightSpeed
	movlw	    Duty50
	cpfsgt	    LeftSpeed
	clrf	    LeftSpeed
	
	movlw	    D'200'
	cpfslt	    RightSpeed
	clrf	    RightSpeed
	movlw	    D'200'
	cpfslt	    LeftSpeed
	clrf	    LeftSpeed
	
	return
	movlw	    D'90'
	cpfslt	    RightSpeed
	goto	    ErrorState
	movlw	    D'90'
	cpfslt	    LeftSpeed
	goto	    ErrorState
	return	


rightneg
	movff	    RightL,offset	;already neg
	return 

leftneg
	movlw	    0xFF
	subwf	    LeftL,W
	addwf	    offset
	return 
	
    
REVERSE
	stopPWM	
	delay	    0xFF
	delay	    0xFF
	delay	    0xFF
	delay	    0xFF
	bcf	    T2CON,2		;stop timer2, and the PWM outputs
	;bsf	    CCP1CON,7		;change to revserse on full bridge out (RD5)
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
	;bcf	    CCP1CON,7		;change to revserse on full bridge out (RD5)
	bsf	    T2CON,2		;start timer2, and renable PWM
	bcf	    direction,0		;set direction bit in direction register 
	bcf	    LeftDirection
	bcf	    RightDirection
	startPWM 
	return 

       
STEPPER
	banksel		direction
	bsf		STEP
	delay		0x2		;gives 15 us delay 
	bcf		STEP
	
	movlf		STEPPER_SPEED,TMR3H
	movlf		STEPPER_SPEEDL,TMR3L
	sub16		stepsH,stepsL,1
	movlw		0x00
	cpfseq		stepsH
	bra		fin_stepper
	cpfseq		stepsL
	bra		fin_stepper
	stopStepperMotor
fin_stepper
	bcf		T3FLAG
	retfie
	
	end