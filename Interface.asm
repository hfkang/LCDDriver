    #include <p18f4620.inc>
    #include <LCDDriver.inc>
    #include <MACROS.inc>
    #include <Variables.inc>

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

#define	    fourMeters	    0x06    ;check on the high bit for the encoders
#define RS  LATD,2
#define E   LATD,3
#define LCD_DELAY_DURATION  0x20	
#define	STEPPER_SPEED	    0x20
		
MAIN CODE 
Start_Msg
    db "Press any key tobegin...",0
Prog_Msg
    db "Operation in    progress..",0
Data_Prompt
    db "Press key  to   get loc.",0
Cont1
    db "Bin 1     ",0
Cont2
    db "Bin 2     ",0
Cont3
    db "Bin 3     ",0
Cont4
    db "Bin 4     ",0
Cont5
    db "Bin 5     ",0
Cont6
    db "Bin 6     ",0
Cont7
    db "Bin 7     ",0
IRMsg
    db "Infrared:",0
ReverseMsg
    db "Entering Return",0
    
isr
	btfsc	INTCON,	    0	;branch if it was a port thing
	call	DIST,	    1
	btfsc	INTCON3,    1    ;encoder 1
	call	ENCODER1,   1
	btfsc	INTCON,	    1	 ;encoder 2 on int0, RB0
	call	ENCODER2,   1
	retfie
start
    movlw	B'01110010'	;Set internal oscillator frequency to 8MHz
    movwf	OSCCON		
    clrf	direction
    
    bcf		RCON,7		;disable interrupt priority
    movlw	B'11011000'	
    movwf	INTCON		;config interrupts for delta B, en High,low priority
    movlw	B'11110101'	;low priority for portb changing, interupt on rising edge
    movwf	INTCON2		
    movlw	B'10010000'	;Enable edge interrupt for RB2
    movwf	INTCON3
    
    movlw	b'00000011'	;Analog In on 0, 1
    movwf	TRISA		;for PORTA
    movlw	b'11110111'	;keypad and ultrasonic inputs on PORTB
    movwf	TRISB		;for PORTB
    movlw	b'00000001'
    movwf   	TRISC		;set PORTC as output    , but c0 is input 
    movlw	b'00000000'	
    movwf	TRISD		;set PORTD as output

    clrf	LATA
    clrf	LATB
    clrf	LATC
    clrf	LATD
    
    clrf	LeftL
    clrf	LeftH
    clrf	RightL
    clrf	RightH
    clrf	IRState
    clrf	BinNum		;clear important registers. 
    clrf	PoleLocH
    clrf	PoleLocL

    movlw	B'00001101'	;configure ADCON1, Analog in for RA0, RA1 
    movwf	ADCON1		
    movlw	B'10110111'	;configure ADCON2
    movwf	ADCON2
    call	CONFIG_PWM	
    stopPWM
 
    movlw	B'01000111'	;Configure Timer0 for PWM measurement, 1 for CPP
    movwf	T0CON		;8bit prescaler from Fosc/4 in 16 bit mode 
    delay	0x50		;wait for LCD to initialie 
    call	LCD_INIT     
    
    readTable	Start_Msg
    call	DISP_TEXT
    call	READ_KEYPAD		;call in keypad. gets a value and returns
    readTable	Prog_Msg
    call	DISP_TEXT
    call	PING
    
    
  
    ;BEGIN MAIN OPERATION LOOP (forward dir)
Main_loop    
    call	PING
    call	LCD_INIT
    call	dispPING		;send out ultrasonic pulse
    call	SuperDelay
    bra		Main_loop
ir    
    stopPWM
    readTable	IRMsg
    call	DISP_TEXT
    call	ADC
    disp16bit	ADRESH,ADRESL		;show ADC result
    movff	BinNum,NumL		
    call	bin8_BCD
    lcdNewLine
    call	Disp_Number		;show current bin number 
    movff	IRState,NumL		;show ir state vector
    call	bin8_BCD
    call	Disp_Number
    delay	0xfF
    startPWM
    ;call	disp_encoders 
    
    movlw	0x04
    cpfsgt	BinNum			;IRState has the number of bins we've read
    bra		Main_loop

    ;movlw	0x20			;reverse on two encoder ticks to right
    ;cpfslt	RightL			
    ;bra	Back_loop			;call reversal method 
        
    ;END MAIN OPERATION LOOP
    ;BEGIN REVERSE LOOP
    call	REVERSE			;enter reverrse mode 
    stopPWM
    readTable	ReverseMsg
    call	DISP_TEXT
    call	READ_KEYPAD
    startPWM
Back_loop
    ;readTable	IRMsg
    ;call	DISP_TEXT
    stopPWM
    readTable	IRMsg
    call	DISP_TEXT
    call	ADC
    movff	ADRESL,NumL
    movff	ADRESH,NumH
    call	bin16_BCD
    call	Disp_Number
    movff	BinNum,NumL
    call	bin8_BCD
    lcdNewLine
    call	Disp_Number
    startPWM
    
    
    movlw	8
    cpfseq	IRState
    bra		Back_loop		;when bin is zero agian go to end
    stopPWM
    
    ;END REVERSE LOOP
    
Disp_Data				;show results 
    readTable	Data_Prompt
    call	DISP_TEXT
    call	READ_KEYPAD		;gets requested container from user
    movwf	keypress		;store value in register
    incf	keypress
    decf	keypress		
    bz		Container1
    decf	keypress
    bz		Container2
    decf	keypress
    bz		Container3
    decf	keypress
    decf	keypress
    bz		Container4
    decf	keypress
    bz		Container5
    decf	keypress
    bz		Container6
    decf	keypress
    decf	keypress
    bz		Container7
    
Container1
    readTable	Cont1
    call	DISP_TEXT
    movff	B1L,NumL
    movff	B1H,NumH
    call	bin16_BCD
    call	Disp_Number
    call	READ_KEYPAD

    goto	Disp_Data    

Container2
    readTable	Cont2
    call	DISP_TEXT
    call	READ_KEYPAD	
    goto	Disp_Data
Container3
    readTable	Cont3
    call	DISP_TEXT
    call	READ_KEYPAD
    goto	Disp_Data
Container4
    readTable	Cont4
    call	DISP_TEXT
    call	READ_KEYPAD	
    goto	Disp_Data
Container5
    readTable	Cont5
    call	DISP_TEXT
    call	READ_KEYPAD	
    goto	Disp_Data
Container6
    readTable	Cont6
    call	DISP_TEXT
    call	READ_KEYPAD	
    goto	Disp_Data
Container7
    readTable	Cont7
    call	DISP_TEXT
    call	READ_KEYPAD	
    goto	Disp_Data
    
    
Stop	bra 	Stop	
    
    ;************************************************************************
    ;Stepper Motor Testing
    ;************************************************************************
ss  call	STEPPER
    delay	STEPPER_SPEED
    bra	ss
    return
    
    ;************************************************************************
    ;PWM Forward Reverse Testing
    ;************************************************************************
testPWM
    clrf	LATD
    call	FORWARD
    startPWM
    call	READ_KEYPAD
    stopPWM
    clrf	LATD
    call	REVERSE
    startPWM
    call	READ_KEYPAD
    return
    
T0Overflow
	comf	LATC
    	bcf	INTCON,TMR0IF
    retfie	

SuperDelay
    delay 0xFF
    delay 0xFF
    delay 0xFF
    delay 0xFF
    delay 0xFF
    delay 0xFF    
    delay 0xFF    
    delay 0xFF
    delay 0xFF    

    return
    
    end