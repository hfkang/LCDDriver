    #include <p18f4620.inc>
    #include <LCDDriver.inc>
    #include <MACROS.inc>
    #include <Variables.inc>
    #include <Constants.inc>

    list P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC

;;;;;;Configuration Bits;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		CONFIG OSC=INTIO67, FCMEN=OFF, IESO=OFF
		CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
		CONFIG WDT = OFF, WDTPS = 32768
		CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF, CCP2MX = PORTC
		CONFIG STVREN = ON, LVP = OFF, XINST = OFF
		CONFIG DEBUG = OFF
		CONFIG CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
		CONFIG CPB = OFF, CPD = OFF
		CONFIG WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
		CONFIG WRTB = OFF, WRTC = OFF, WRTD = OFF
		CONFIG EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
		CONFIG EBTRB = OFF

		org 0x0
		goto	start
		org 0x8		
		goto	isr
		org 0x18	;low priority (ultrasonic ping)
		retfie
		
main_bank	udata
CurrBin		res 1
TotBin		res 1
threshL		res 1
threshH		res 1        
rtc_min		res 1
rtc_sec		res 1
rtc_hr		res 1
rtc_day		res 1
rtc_date	res 1
rtc_mon		res 1
rtc_yr		res 1
tens_digit	res 1
ones_digit	res 1
temp_var2	res 1
start_time_sec	res 1
start_time_min	res 1
end_time_sec	res 1
end_time_min	res 1
temp_var4	res 1
temp_var5	res 1
second_diff	res 1
min_diff	res 1
binstate	res 1
steps		res 1
stepsH	    	res 1
stepsL		res 1
armState	res 1
				    ;bank0 buffered version of PID variables. Because bank switching is annoying 
mypidOut0	res 1
mypidOut1	res 1
mypidOut2	res 1
mypidStat1	res 1
	
updates		res 1
encThreshL	res 1
encThreshH	res 1
		
MAIN CODE 
    global	    ErrorState,threshL,threshH,mypidStat1,mypidOut0,mypidOut1,mypidOut2
    global	    dispOperationData,dispCorrection,stepsH,stepsL
isr
	btfsc	INTCON,	    0	;branch if it was a port thing
	call	DIST,	    1
	btfsc	INTCON,	    1    ;encoder 1 int0, RB0
	call	ENCODER1,   1	 ;encoder 1 is on right side
	btfsc	INTCON3,    1	 ;encoder 2 on int2, RB2
	call	ENCODER2,   1	 ;encoder 2 is on left side 
	btfsc	PIR1,	    TMR1IF				
	call	PidInterrupt
	;btfsc	T3FLAG
	;call	STEPPER
	retfie
	
Start_Msg
    db "Push 0 to begin",0
Prog_Msg
    db "Operation in    progress..",0
Data_Prompt
    db "Press key  to   get loc.",0
Cont1
    db "Bin 1  (cm)",0
Cont2
    db "Bin 2  (cm)",0
Cont3
    db "Bin 3  (cm)",0
Cont4
    db "Bin 4  (cm)",0
Cont5
    db "Bin 5  (cm)",0
Cont6
    db "Bin 6  (cm)",0
Cont7
    db "Bin 7  (cm)",0
IRMsg
    db "Infrared:",0
ReverseMsg
    db "Entering Return",0
Elapsed
    db "Runtime: ",0
Stickers
    db "Front:",0
Stickers2
    db " Back:",0
NumberofBins
    db "Total Bins: ",0
Obstacle
    db "Obstacle Detected",0
PoleDetected
    db "Pole Detected",0
BinDetected
    db "Bin Detected",0
Biin
    db "Bin No: ",0
isBlack
    db "Black",0
isWhite
    db "White",0
Movingintoposition
    db "Moving into position",0
int0
    db "int0",0
int1
    db "int1",0
int2
    db "int2",0
Ultrasound
    db "Ultrasound:",0
ExtendingArm
    db "Extending Arm",0
RetractingArm
    db "Retract Arm",0

   
start
    movlw	B'01110010'	;Set internal oscillator frequency to 8MHz
    movwf	OSCCON		
    
    bcf		RCON,7		;disable interrupt priority
    movlw	B'11011000'	;disable RB change interrupts 
    movwf	INTCON		;config interrupts for delta B, en High,low priority
    
    bsf		INTCON,3	;enable the port change interrupts for now. 
    
    movlw	B'11110111'	;low priority for portb changing, interupt on rising edge
    movwf	INTCON2		
    movlw	B'11010000'	;Enable edge interrupt for RB2
    movwf	INTCON3
    
    disableEncoders
    
    movlf	b'00111111', TRISA
    movlf	b'00011111', TRISB
    movlf	b'11011000', TRISC
    movlf	b'00000000', TRISD
    bcf		TRISE,0
    bcf		TRISE,1

    clrf	LATA
    clrf	LATB
    clrf	LATC
    clrf	LATD
    clrf	LATE
    
    bsf		STEP_ENABLE		;disable stepper 
    
    clrf	direction
    clrf	LeftL
    clrf	LeftH
    clrf	RightL
    clrf	RightH
    clrf	IRState
    clrf	BinNum		;clear important registers. 
    clrf	PoleLocH
    clrf	PoleLocL
    clrf	PoleH
    clrf	PoleL
    clrf	TotBin
    clrf	CurrBin
    clrf	B1L
    clrf	B1H
    clrf	B1S
    clrf	B2L
    clrf	B2H
    clrf	B2S
    clrf	B3L
    clrf	B3H
    clrf	B3S
    clrf	B4L
    clrf	B4H
    clrf	B4S
    clrf	B5L
    clrf	B5H
    clrf	B5S
    clrf	B6L
    clrf	B6H
    clrf	B6S
    clrf	B7L
    clrf	B7H
    clrf	B7S
    clrf	armState
    clrf	mypidOut0	
    clrf	mypidOut1	
    clrf	mypidOut2	
    clrf	mypidStat1	
    clrf	rampstate
    movlf	0x2, rampinterval
    movlf	ActualDefault,baseline
    movlf	dutyStart,ramp
	
    clrf	updates		
    clrf	encThreshL	
    clrf	encThreshH	
    
    clrf	threshHtemp
    clrf	threshLtemp
    
    
    movlf	0xFF,PoleL	;stop premature beeping on the first run 
    movlf	0xFF,ultra4
    movlf	0xFF,ultra3
    movlf	0xFF,ultra2
    movlf	0xFF,ultra1
    movlf	.4,ultrastate
     
    movlw	B'00001101'	;configure ADCON1, Analog in for RA0, RA1  
    movwf	ADCON1		
    movlw	B'10110111'	;configure ADCON2
    movwf	ADCON2

    
    movlw	B'01000111'	;Configure Timer0 for distance measurement, 1 for CPP
    movwf	T0CON		;8bit prescaler from Fosc/4 in 16 bit mode 
    
    movlf	B'10110000',T3CON
    bcf		T3ENABLE
    bcf		T3INTENABLE
    bcf		T3FLAG
    bsf		STEPDIR		;forward direction by default
    bsf		STEP_ENABLE	;disable stepper until needed

    delay	0x10		;wait for LCD to initialie 
    call	LCD_INIT     
    call        ConfigureI2C            ; Configures I2C for RTC
    
    bcf		LeftDirection
    bcf		RightDirection
    call	CONFIG_PWM	
    stopPWM
        
    bcf		LeftMotor
    bcf		RightMotor
    call	PidInitalize		;initialize PID variables 
    
    ;goto	_rev
    ;call	ss 
    ;goto	fivesteps
    ;goto	testAN937
    ;goto	ultratest
    ;goto	testBuzzer
    ;goto	testPWM
    ;goto	_rev		;test reverse loop functionality 
    ;goto	Dumb
    ;goto	move4m
    ;call	REVERSE
    ;goto	irtesting
    goto	testPID
    ;goto	ploop
    
    
    
    banksel	RightH
    dispText	Start_Msg,first_line
    lcdNewLine
    call	RTCDisplayTimeDate	;display time and date on screen 
    call	READ_KEYPAD		;call in keypad. gets a value and returns
    call	PING
    dispText	Prog_Msg,first_line
    call        ReadFromRTC
    movff       rtc_sec, start_time_sec	;store begin time for elapsed trial time
    movff       rtc_min, start_time_min
    ;call	testPID			;intializes our run
    ;*************************************************************************
    ;
    ;		    BEGIN MAIN OPERATION LOOP (forward dir)
    ;
    ;*************************************************************************
Main_loop    
    banksel	RightH
    
    movlw	DISTTHRESH	    ;check the previous ultrasonic reading we got 
    cpfslt	PoleL		    ;can convert to 16 bit if necessary 
    goto	_pid		
    bsf		BUZZER		
    
    movff	    RightH,encThreshH
    movff	    RightL,encThreshL
    add16	    encThreshH,encThreshL,IRBinScanOffset	;offset should be a literal (constant)
    
checkifpole    
    movlw	    DISTTHRESH	     
    cpfslt	    PoleL
    bra		    _pol
    bra		    continuecheck

continuecheck
    
    call	    PID
    call	    PING
    
    enccomp16	    RightH,RightL,_myadc,checkifpole,_myadc
    ;encOffset 	IRBinScanOffset	    ;this makes the robot keep moving until it reaches the sticker
    ;call	PING
    			
   
_pol				;we know a pole is here
    dispText	PoleDetected,second_line
    movff	RightL,PoleLocL	
    movff	RightH,PoleLocH	
    sub16	PoleLocH,PoleLocL,IRBinScanOffset
    goto	_pid	    
    
_myadc
    lcdClear				
    call	ADC		    ;check ADC value 
    dispText	IRMsg,first_line    
    disp16	ADRESH,ADRESL
    
_bin				;we know a bin is here! now go to where sticker is 
    dispText	BinDetected,second_line
    incf	CurrBin
    incf	TotBin
    movlf	WHITEH,threshH
    movlf	WHITEL,threshL
    comp16	ADRESH,ADRESL,Bl,Wh,Wh
    
Bl  
    movf	CurrBin,W
    storeBin	WREG, 0, 0	;store frontside, black
    lcdClear
    dispText	isBlack,second_line
    encOffset	BinHalfway
    resetrolling
    goto	_pid
Wh  
    movf	CurrBin,W
    storeBin	WREG, 0, 1	;store frontside, white	
    lcdClear
    dispText	isWhite,second_line
    encOffset	BinHalfway
    resetrolling
    goto	_pid
    

_pid
    banksel	CurrBin
    call	PING
    bcf		BUZZER
    startPWM
    ;call	PID		;adjust motor output speeds
    
    movlw	7
    cpfslt	CurrBin		
    goto	_rev		;go to reverse process if we've reached 7 bins
    
    delay	LCD_DELAY_DURATION
    
    lcdHomeLine
    call	disp_encoders
    lcdNewLine
    call	dispPING
    
    ;movlf	0x06,threshH
    ;movlf	0x1D,threshL
    
checkdistance
    
    movlf	0x01,threshH
    movlf	0x98,threshL
    comp16	RightH,RightL,mayberev,Main_loop,Main_loop
    
mayberev 
    btfsc	RightH,7
    goto	Main_loop
    
    
    ;**************************************************************************
    ;
    ;				END MAIN OPERATION LOOP
    ;
    ;**************************************************************************

_rev
    banksel		CurrBin
    incf		CurrBin	    ;account for decrement offset in code 
    stopPWM
    movlf		armStepH,stepsH
    movlf		armStepL,stepsL
    stopPWM
    dispText		extendarmmsg,first_line
    extendStepper
    
extend
    delay		StepDelay
    call		STEPPER
    sub16		stepsH,stepsL,1
    clrf		threshH
    movlf		0x05,threshL
    comp16		stepsH,stepsL,extend,stahp_extend,stahp_extend
stahp_extend
    stopStepperMotor
    call	PING		    ;check the ultrasonic now. 
    call	REVERSE			;enter reverrse mode 
    dispText	ReverseMsg,first_line
    startPWM
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;			    TESTING VARIABLES ASSIGNED HERE
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
       
     
    ;**************************************************************************
    ;
    ;				   BEIGIN REVERSE LOOP
    ;
    ;**************************************************************************
Back_loop   
    
_pid2
    stopStepperMotor
    startPWM
    call	PING
    bcf		BUZZER
    call	PID		;adjust motor output speeds  
    
    movff	PoleLocH,threshH
    movff	PoleLocL,threshL
    add16	threshH,threshL,poleOffset	;trigger to dodge pole
    comp16	RightH,RightL,nopole,checkloc,checkloc
    
checkloc
    movff	PoleLocH,threshH
    movff	PoleLocL,threshL
    sub16	threshH,threshL,poleRangeOffset
    comp16	RightH,RightL,nearpole,nopole,nopole	;checks if we already passed the pole
    
nearpole
    movff	PoleLocH,threshH
    movff	PoleLocL,threshL
    sub16	threshH,threshL,poleExtendOffset
    comp16	RightH,RightL,retractarm,extendarm,retractarm	;extend arm if passed, otherwise retract arm

retractarm
    stopPWM
    dispText	RetractingArm,first_line
    btfsc	retracted
    bra		_pid2
    call	loadsteps
    bsf		retracted
    retractStepper
_ra    
    delay		StepDelay
    call		STEPPER
    sub16		stepsH,stepsL,1
    clrf		threshH
    movlf		0x05,threshL
    comp16		stepsH,stepsL,extend,_pid2,_pid2
    
extendarm   
    stopPWM
    dispText	ExtendingArm,first_line
    btfsc	extended
    bra		_pid2
    call	loadsteps
    bsf		extended
    extendStepper
_ea       
    delay		StepDelay
    call		STEPPER
    sub16		stepsH,stepsL,1
    clrf	threshH
    movlf	0x05,threshL
    comp16	stepsH,stepsL,_ea,_pid2,_pid2

    
nopole    
    stopStepperMotor
    lcdHomeLine
    call	disp_encoders
    lcdNewLine
    call	dispPING
        
    movlw	DISTTHRESH	    ;check the previous ultrasonic reading we got
    cpfsgt	PoleL		    ;can convert to 16 bit if necessary 
    goto	_bin2
    
    movlf	0x00,threshH
    movlf	0x05,threshL
    comp16	RightH,RightL,Back_loop,Finish,Finish
    

    
_bin2
    bsf		BUZZER    
    lcdClear				;we know a bin is here! now we wait
    dispText	BinDetected,second_line
    decf	CurrBin
    encOffset 	IRBinScanOffset	
    call	ADC		;check adc again
    movlf	WHITEH,threshH
    movlf	WHITEL,threshL
    sub16	threshH,threshL,SensorDelta  
    comp16	ADRESH,ADRESL,Bl2,Wh2,Wh2

Bl2 movf	CurrBin,W
    storeBin	WREG, 1, 0	;store frontside, black
    dispText	isBlack,second_line
    encOffset	BinHalfway
    resetrolling
    bra		_pid2
    
Wh2  movf	CurrBin,W
    storeBin	WREG, 1, 1	;store frontside, white	
    dispText	isWhite,second_line
    encOffset	BinHalfway
    resetrolling
    bra		_pid2
    


    
        

    ;**************************************************************************
    ;
    ;				    END REVERSE LOOP
    ;
    ;**************************************************************************
Finish
    stopPWM
    call        ReadFromRTC
    movff       rtc_sec, end_time_sec
    movff       rtc_min, end_time_min
    call	GetTimeTaken
    
    dispText	Elapsed,first_line
    rtc_disp	min_diff
    movlw	0x3A	
    lcdData
    rtc_disp	second_diff
    
    dispText	NumberofBins,second_line
    movf	TotBin,W
    addlw	0x30
    lcdData
    call	READ_KEYPAD
    
    
Disp_Data				;show results 
    stopPWM
    dispText	Data_Prompt,first_line
    call	READ_KEYPAD		;gets requested container from user
disp_again    
    movwf	keypress		;store value in register
    
    movlw	0x03
    cpfseq	keypress
    bra		$+4
    bra		Disp_Data
    movlw	0x07
    cpfseq	keypress
    bra		$+4
    bra		Disp_Data
    movlw	0x08
    cpfsgt	keypress
    bra		valid_key
    bra		Disp_Data

valid_key
    
    movlw	0x07
    cpfslt	keypress
    decf	keypress
    
    movlw	0x03
    cpfsgt	keypress
    incf	keypress
    
    movf	TotBin,W
    cpfsgt	keypress
    bra		show_bin
    bra		Disp_Data
    
show_bin    
    lcdClear
    decf	keypress		
    bz		Container1
    decf	keypress
    bz		Container2
    decf	keypress
    bz		Container3
    decf	keypress
    bz		Container4
    decf	keypress
    bz		Container5
    decf	keypress
    bz		Container6
    decf	keypress
    bz		Container7
    
Container1         readTable    Cont1
                   call                 DISP_TEXT
                   movff                B1L,NumL
                   movff                B1H,NumH
                   call                 convertEncoder
                   movff                B1S,binstate
                   call                 stick_disp
                   goto                 disp_again
Container2         readTable    Cont2
                   call                 DISP_TEXT
                   movff                B2L,NumL
                   movff                B2H,NumH
                   call                 convertEncoder
                   movff                B2S,binstate
                   call                 stick_disp
                   goto                 disp_again
Container3         readTable    Cont3
                   call                 DISP_TEXT
                   movff                B3L,NumL
                   movff                B3H,NumH
                   call                 convertEncoder
                   movff                B3S,binstate
                   call                 stick_disp
                   goto                 disp_again
Container4         readTable    Cont4
                   call                 DISP_TEXT
                   movff                B4L,NumL
                   movff                B4H,NumH
                   call                 convertEncoder
                   movff                B4S,binstate
                   call                 stick_disp
                   goto                 disp_again
Container5         readTable    Cont5
                   call                 DISP_TEXT
                   movff                B5L,NumL
                   movff                B5H,NumH
                   call                 convertEncoder
                   movff                B5S,binstate
                   call                 stick_disp
                   goto                 disp_again
Container6         readTable    Cont6
                   call                 DISP_TEXT
                   movff                B6L,NumL
                   movff                B6H,NumH
                   call                 convertEncoder
                   movff                B6S,binstate
                   call                 stick_disp
                   goto                 disp_again
Container7         readTable    Cont7
                   call                 DISP_TEXT
                   movff                B7L,NumL
                   movff                B7H,NumH
                   call                 convertEncoder
                   movff                B7S,binstate
                   call                 stick_disp
                   goto                 disp_again
    
    
Stop	bra 	Stop	
		   
stick_disp  
	dispText    Stickers,second_line
	btfsc	    binstate,0		;check the frontside
	bra	    _frontWhite
	bra	    _frontBlack
_frontWhite
	movlw	    0x57
	call	    NIBBLE_LCD
	bra	    _reardisp
_frontBlack
	movlw	    0x42
	call	    NIBBLE_LCD
_reardisp
	readTable   Stickers2
	call	    DISP_TEXT
	btfsc	    binstate,1
	bra	    _rearWhite
	bra	    _rearBlack
_rearWhite
	movlw	    0x57
	call	    NIBBLE_LCD
	bra	    _enddisp
_rearBlack
	movlw	    0x42
	call	    NIBBLE_LCD	
_enddisp
	call	    READ_KEYPAD
	return
	
convertEncoder		    ;perform a division by 4 on a 16 bit number 
	rlncf	    NumH
	rlncf	    NumH
	rlncf	    NumH
	rlncf	    NumH
	rlncf	    NumH
	rlncf	    NumH
	rrncf	    NumL
	rrncf	    NumL
	
	
	movf	    NumH,W
	mask_bits   NumL,B'11000000'
	clrf	    WREG
	mask_bits   NumH,B'11111110'
	call	    bin16_BCD
	call	    Disp_Number
	return
	;*********************************************************************
	;
	;		    loadsteps - sets stepper variables for arm movement
	;
	;********************************************************************
loadsteps
	movlf		armStepH,stepsH
	movlf		armStepL,stepsL
	return 
	
ErrorMsg
	db "Alignment error",0
ErrorState
	stopPWM
	bcf	    LATC,1
	bcf	    LATC,2
	dispText    ErrorMsg,first_line
	lcdNewLine
	call	    disp_encoders
	bsf	    BUZZER
	delay	    0xFF
	bcf	    BUZZER
	delay	    0x10
	bsf	    BUZZER
	delay	    0xFF
	bcf	    BUZZER
	delay	    0x10
	bsf	    BUZZER
	delay	    0xFF
	bcf	    BUZZER
	
	goto	    Stop
move4m
	;*********************************************************************
	;
	;		    move4m - move 4 m, showing encoders at all times
	;
	;*********************************************************************
	lcdClear
	call	FORWARD
	startPWM
loop4m	lcdHomeLine
	movff	    LeftL,NumL
        movff	    LeftH,NumH
        call	    convertEncoder
	movlw	    0x20
	lcdData
	movff	    RightL,NumL
        movff	    RightH,NumH
        call	    convertEncoder
	
	lcdNewLine
	
	call	PING		    ;check the ultrasonic now. 
	call	dispPING    
	bcf	BUZZER
	movlw	0xA
	cpfslt	PoleL
	bra	loop4m
u2	call	PING
	lcdNewLine
	call	dispPING
	bsf	BUZZER
	movlw	0xA
	cpfsgt	PoleL
	bra	u2
	
	movlf	0x01,threshH
	movlf	0x86,threshL
	comp16 RightH,RightL,gorev,loop4m,gorev
		
gorev	
	dispText    ReverseMsg,second_line
	call	    REVERSE 
	lcdClear
loopback4m
	lcdHomeLine
	movff      LeftL,NumL
        movff      LeftH,NumH
        call       convertEncoder
	movlw	   0x20
	lcdData
	movff	    RightL,NumL
        movff	    RightH,NumH
        call	    convertEncoder

	lcdNewLine
	call	PING		    ;check the ultrasonic now. 
	call	dispPING    
	bcf	BUZZER
	movlw	0xA
	cpfslt	PoleL
	bra	loop4m
u3	call	PING
	lcdNewLine
	call	dispPING
	bsf	BUZZER
	movlw	0xA
	cpfsgt	PoleL
	bra	u3
	
	movlf	0x00,threshH
	movlf	0x05,threshL
	comp16	RightH,RightL,loopback4m,stahp_pls,stahp_pls

stahp_pls
	stopPWM
	goto	Disp_Data
	
	
	

    ;*************************************************************************
    ;
    ;			    Dumb Testing Routine for the robot
    ;	
    ;*************************************************************************
Dumb
    lcdHomeLine
    call	disp_encoders
    btfsc	KEYPAD_DA
    bra		enc
    bra		Dumb
    
    
enc    
    startPWM		;start motors 
    ;encOffset		0x05
    btfsc	KEYPAD_DA
    bra		quit
    clrf		threshH
    movlf		0x05,threshL
    comp16		RightH,RightL,quit,enc,enc
quit
    movlf		0x20,steps
    stopPWM
    dispText		extendarmmsg,first_line
    
fivesteps
    bcf			STEP_ENABLE 
    call		STEPPER
    delay		STEPPER_SPEED 
    btfss		KEYPAD_DA
    bra			fivesteps
    
irtesting    
    dispText	IRMsg,first_line
irtestloop
    call	FORWARD
    call	ADC		;check ADC value 
    lcdNewLine
    disp16	ADRESH,ADRESL
    movlw	0x20
    lcdData 
    call	REVERSE
    call	ADC
    disp16	ADRESH,ADRESL
    btfsc	KEYPAD_DA
    bra		ultratest
    bra		irtestloop
    
ultratest
    lcdClear
    ;dispText	Ultrasound,second_line
    
ultraloop
    lcdHomeLine
    call	PING		    ;check the ultrasonic now. 
    call	dispPING    
    lcdNewLine
    call	disp_encoders
    bra		ultraloop 
    
    bcf		BUZZER
    movlw	0xA
    cpfslt	PoleL
    bra		ultraloop
u1  call	PING
    call	dispPING
    bsf		BUZZER
    movlw	0xA
    cpfsgt	PoleL
    bra		u1    
    bra		ultraloop
    
    btfsc	KEYPAD_DA
    bra		Dumb
    bra		ultratest
    

    
    
    ;************************************************************************
    ;Stepper Motor Testing
    ;************************************************************************
ss  
    bsf	    STEPDIR
ss_loop
    bsf	    T3ENABLE
    bsf	    T3INTENABLE
    
    bcf	    STEP_ENABLE
    
    btfss   KEYPAD_DA
    bra	    $-2
    
    bcf	    STEPDIR
    bra	    ss_loop
    return
    
    
    ;************************************************************************
    ;PWM Forward Reverse Testing
    ;************************************************************************
FullPower
    db "Full Power",0
PWM1
    db "75% Duty Cycle",0
PWM2
    db "50% Duty Cycle",0
PWM3
    db "25% Duty Cycle",0
Off
    db "Grounded",0
  
    
testPWM
    call	FORWARD
    startPWM
    dispText	FullPower,second_line
    movlf	DutyDefault,RightSpeed
    movlf	DutyDefault,LeftSpeed
g1  lcdHomeLine
    call	disp_encoders
    btfss	KEYPAD_DA
    bra		g1

    
    startPWM
    dispText	PWM1,second_line
    movlf	Duty75,RightSpeed
    movlf	Duty75,LeftSpeed
g2  lcdHomeLine
    call	disp_encoders
    btfss	KEYPAD_DA
    bra		g2    
    
    
    dispText	PWM2,second_line
    movlf	Duty50,RightSpeed
    call	READ_KEYPAD
    movlf	Duty50,LeftSpeed
    
g3  lcdHomeLine
    call	disp_encoders
    btfss	KEYPAD_DA
    bra		g3    
    
    dispText	PWM3,second_line
    movlf	Duty25,RightSpeed
    call	READ_KEYPAD
    movlf	Duty25,LeftSpeed
    
g4  lcdHomeLine
    call	disp_encoders
    btfss	KEYPAD_DA
    bra		g4    
    
nopwm
    
    dispText	Off,second_line
    movlf	0x00,RightSpeed
    movlf	0x00,LeftSpeed
    
g5  lcdHomeLine
    call	disp_encoders
    btfss	KEYPAD_DA
    bra		g5
    
    
    btfsc	direction,0
    bra		toforward
    
    btfss	direction,0
    bra		tobackward
toforward
    call	FORWARD
    bra		testPWM
tobackward
    call	REVERSE
    bra		testPWM
    
    return
    
    ;**********************************************************************
    ;
    ;				AN937 Testing ground
    ;
    ;**********************************************************************
testAN937    
    banksel	AARGB2
    
    clrf	AARGB3
    clrf	BARGB3
    clrf	AARGB2
    clrf	BARGB2
    clrf	AARGB1
    clrf	BARGB1
    clrf	AARGB0
    clrf	BARGB0
    
    movlf	0x10,AARGB2
    movlf	0x02,BARGB1
    
    call	FXD2416U
    
endtest	    bra	    endtest
    
    ;**********************************************************************
    ;
    ;				P Control Algorithm
    ;
    ;**********************************************************************    
testPID
	stopPWM
	lcdClear
	bcf	    direction,0		;set direction bit in direction register 
	bsf	    LeftDirection
	bsf	    RightDirection	
	call	    SuperDelay
	reset_encoders
	call	    rampup
	startPWM
	enableEncoders
	movlf	D'255',updates
ploop	
	banksel		RightH
	call		PID
	decfsz		updates
	bra		ploop
	call		dispOperationData
	movlf		D'255',updates

	movlf		endDistH,threshH
	movlf		endDistL,threshL
	sub16		threshH,threshL,endRampOffset
	comp16		RightH,RightL,slowdown,head_back,slowdown
slowdown	
	btfss		rampstate,0
	call		rampdown
	
head_back
	movlf		endDistH,threshH
	movlf		endDistL,threshL
	comp16		RightH,RightL,ReverseTest,ploop,ReverseTest
	
ReverseTest
	disableEncoders
	stopPWM
	call		REVERSE	
	call		rampup
	startPWM
	call		deslack
	enableEncoders
ploop2	
	banksel		RightH
	movlf	0xFF,threshH
	movlf	0xE0,threshL
	comp16	RightH,RightL,StopTest,disp2,StopTest
disp2	
	call		PID
	decfsz		updates
	bra		ploop2
	call		dispOperationData
	movlf		D'255',updates
	bra		ploop2


StopTest
	bsf		BUZZER
	stopPWM
	disableEncoders
	call	dispOperationData
	
	
lolend	
	disableEncoders
	stopPWM
	bra	lolend
	
dispOperationData
	lcdHomeLine
	call		disp_encoders
	lcdNewLine
	movff		LeftSpeed,NumL
	call		bin8_BCD
	call		Disp_Number
	movlw		0x20
	lcdData
	movff		RightSpeed,NumL
	call		bin8_BCD
	call		Disp_Number
	return	    
	
dispCorrection:
	lcdNewLine
	movff		mypidOut1,NumH
	movff		mypidOut2,NumL
	call		bin16_BCD
	call		Disp_Number
	return		
	

rampup:
	bcf	    rampstate,0
	movlf	    dutyStart,ramp
	return
	
rampdown:
	bsf	    rampstate,0
	movlf	    ActualDefault,ramp
	return 
	
deslack:
	movlf	    Duty5,LeftSpeed
	movlf	    Duty5,RightSpeed
	delay	    0x40
	movff	    RightH,LeftH
	movff	    RightL,LeftL
	return
    ;**********************************************************************
    ;
    ;			Stepper Driver Test 
    ;
    ;**********************************************************************
extendarmmsg
	db "Move arm out",0
retractarmmsg
	db "Retract arm in",0
testStepper
    stopPWM
    bcf		STEPDIR
    dispText	extendarmmsg,first_line
lo  call	STEPPER
    delay	STEPPER_SPEED
    btfss	PORTB,1
    bra		lo
    dispText	retractarmmsg,first_line
    
    bsf		STEPDIR
li  call	STEPPER
    delay	STEPPER_SPEED
    btfss	PORTB,1
    bra		li
    bra		testStepper
    return 
    
T0Overflow
    comf	LATC
    bcf		INTCON,TMR0IF
    retfie	
    
SuperDelay
    delay 0x80
    return
    
testBuzzer
    bsf	    BUZZER
    call    SuperDelay
    bcf	    BUZZER
    call    SuperDelay
    bra	    testBuzzer
    

    
; ----------------------------------------------------------------------------
; I2C Subroutines
; ----------------------------------------------------------------------------
; CheckI2C: Loops until I2C has successfully been read/written from/to
; INPUT: None
; OUTPUT: None
; ----------------------------------------------------------------------------
CheckI2C
        btfss       PIR1, SSPIF   ; set whenever complete byte transferred
        goto        CheckI2C
        goto        EndCheckI2C
EndCheckI2C
        bcf         PIR1, SSPIF
        return
; ----------------------------------------------------------------------------
; ConfigureI2C: Configures I2C/RTC for use
; INPUT: None
; OUTPUT: None
; ----------------------------------------------------------------------------
ConfigureI2C
        ;enable MSSPmode set to I2c master mode - SSPCON1
        ; disable slew control rate control
        ; store 24 in sspadd
        movlf       b'00101000', SSPCON1     ; enable MSSP master mode, enable serial port
        movlf       b'10000000', SSPSTAT     ; disable slew control, write mode
        movlf       d'24', SSPADD           ; 100 kHz baud rate
        rtc_wr      rtc_con, 0x90
        return
; ----------------------------------------------------------------------------
; InitializeRTC: Sets date, time, year, etc on RTC- only to be used on initial 
; use
; INPUT: None
; OUTPUT: None
; ----------------------------------------------------------------------------
InitializeRTC
        rtc_wr      seconds, d'0'       ; seconds
        rtc_wr      minutes, 0x40       ; minutes
        rtc_wr      hours, 0x21         ; hours
        rtc_wr      day, d'1'           ; days
        rtc_wr      date, 0x19		; date
        rtc_wr      month, d'3'         ; month
        rtc_wr      year, 0x16		; year
        return

; ----------------------------------------------------------------------------
; ReadFromRTC: Reads time, date, year, etc of current values on RTC
; INPUT: None
; OUTPUT: rtc_min, rtc_sec, rtc_hr, rtc_day, rtc_date, rtc_mon, rtc_yr
; ----------------------------------------------------------------------------
ReadFromRTC
        ; write that we want to start at seconds register
        i2c_start
        movlw       0xD0                ; slave address: write
        i2c_write
        movlw       0x00                ; seconds register
        i2c_write
        i2c_stop
        ; configure RTC clock for reading data
        i2c_start
        movlw       0xD1                ; slave address: read
        i2c_write
        ; repeatedly read values from i2c buffer, write into appropriate rtc
        ; register
        rd_i2c_buf_ack  rtc_sec
        rd_i2c_buf_ack  rtc_min
        rd_i2c_buf_ack  rtc_hr
        rd_i2c_buf_ack  rtc_day
        rd_i2c_buf_ack  rtc_date
        rd_i2c_buf_ack  rtc_mon
        rd_i2c_buf_nack rtc_yr
        i2c_stop
        return
; ----------------------------------------------------------------------------
; RTCDisplayTimeDate: Displays date and time on two lines of LCD display
; INPUT: None
; OUTPUT: None
; ----------------------------------------------------------------------------
RTCDisplayTimeDate
        call        ReadFromRTC         ; get current value of date, time
        ; display date in "mm/dd/yy" format
        rtc_disp    rtc_mon
        movlw       0x2F                ; "/"
        lcdData
        rtc_disp    rtc_date
        movlw       0x2F                ; "/"
	lcdData
	rtc_disp    rtc_yr
        ; display time in "hh:mm: format
	movlw	    0x20
	lcdData
        rtc_disp    rtc_hr
        movlw       0x3A                ; ":"
	lcdData
        rtc_disp    rtc_min
        return

; ----------------------------------------------------------------------------
; ComputeTimeDifference: Computes how long an operation took by computing
; difference between start and end times, stores BCD results in second_diff and
; minute_diff
; INPUT: start_time_sec, start_time_min, end_time_sec, end_time_min
; OUTPUT: second_diff, min_diff
; ----------------------------------------------------------------------------
GetTimeTaken
FirstDigitSeconds
        ; first digit of second
        movf    end_time_sec, W
        andlw   0x0F                         ; mask upper MSBs
        movwf   temp_var4
        movf    start_time_sec, W
        andlw   0x0F                         ; mask upper MSBs of starting time
        movwf   temp_var5
        ; ending first digit (seconds) - starting first digit (seconds)
        movf    temp_var5, W
        subwf   temp_var4, w                 
        bnn     FirstDigitSecondsNotNegative ; no other processing needed
        addlw   d'10'
        andlw   0x0F                    ; mask to have lower bit only
        movwf   second_diff
        ; add 1 to starting second digit (seconds), same thing as subtracting 1 
        ; from ending second digit (seconds)
        swapf   start_time_sec, W       ; swap to lower nibble and store in W
        andlw   0x0F                    ; mask it
        incf    WREG                    ; add one
        swapf   WREG                    ; swap back to upper nibble
        movwf   start_time_sec          ; move it back to old data
        bra     SecondDigitSeconds

FirstDigitSecondsNotNegative
        andlw   0x0F                    ; mask MSBs
        movwf   second_diff             ; no other processing needed

SecondDigitSeconds
        ; get second digit of ending time (seconds)
        swapf   end_time_sec, w
        andlw   0x0F                    
        movwf   temp_var4
        ; get second digit of starting time (seconds)
        swapf   start_time_sec, w
        andlw   0x0F            
        movwf   temp_var5
        ; ending second digit(seconds) - starting second digit (seconds)
        movff   temp_var5, WREG
        subwf   temp_var4, w            
        bnn     SecondDigitSecondsNotNegative   ; no other processing needed
                                                ; if this is not negative
        ; if negative, then add 6 to - result of carrying from minutes
        addlw   d'6'
        swapf   WREG
        andlw   0xF0                    ; this is second digit so mask lower bits
        addwf   second_diff, 1          ; add to store into the second difference
        ;and add 1 to the low digit of minute of the OLD one
        ; before adding it, need to check if it's 9. If it is, then will need
        ; to add one to HighMinute 
        sublw   d'9'
        bnz     AddOneToFirstDigitMinutes
AddOneToSecondDigitMinutes
            movlw   0xF0                ; make the lower bit 0 (bcoz 9+1=10)
            andwf   start_time_min, 1
            movlw   0x10
            addwf   start_time_min, 1   ; add 1 to the high nibble
            goto     FirstDigitMinutes

AddOneToFirstDigitMinutes
            ; simply add 1 to the RTC_Minute_Old
            incf     start_time_min, 1
            goto     FirstDigitMinutes

SecondDigitSecondsNotNegative
                andlw   0x0F
                swapf   WREG
                addwf   second_diff

FirstDigitMinutes
        ; lower digit of minute
        movff   end_time_min, WREG
        andlw   0x0F                    ; read lower nibble only
        movwf   temp_var4
        movff   start_time_min, WREG
        andlw   0x0F
        movwf   temp_var5
       
        movff   temp_var5, WREG
        subwf   temp_var4, 0    
        bnn     FirstDigitMinutesNotNegative        
                                       
        
        addlw   d'10'
        andlw   0x0F                    
        movwf   min_diff
        
        swapf   start_time_min, 0       
        andlw   0x0F                    
        incf    WREG                    
        swapf   WREG                    
        movwf   start_time_min         
        bra     SecondDigitMinutes

FirstDigitMinutesNotNegative
                andlw   0x0F
                movwf   min_diff

SecondDigitMinutes
        ; high digit of minute
        swapf   end_time_min, w
        andlw   0x0F                    
        movwf   temp_var4
        swapf   start_time_min, w
        andlw   0x0F
        movwf   temp_var5
       
        movff   temp_var5, WREG
        subwf   temp_var4, 0   
        bnn     SecondDigitMinutesNotNegative     
                                       
   
        addlw   d'6'
        swapf   WREG
        andlw   0xF0                   
        addwf   min_diff, 1      
        
        bra     EndGetTimeTaken

SecondDigitMinutesNotNegative
                andlw   0x0F
                swapf   WREG
                addwf   min_diff

EndGetTimeTaken
        return
    
    end