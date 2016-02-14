    #include <p18f4620.inc>
    #include <LCDDriver.inc>
    #include <MACROS.inc>

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
		goto start
		org 0x8		
		call DIST
		org 0x18	;low priority (ultrasonic ping)
		retfie
MAIN CODE 
Start_Msg
    db "Press any key tobegin...",0
Prog_Msg
    db "Operation in    progress..",0
Data_Prompt
    db "Press key  to   get loc.",0
Cont1
    db "Container 1     40cm",0
Cont2
    db "Container 2     60cm",0
Cont3
    db "Container 3     70cm",0
Cont4
    db "Container 4     80cm",0
Cont5
    db "Container 5     100cm",0
Cont6
    db "Container 6     150cm",0
Cont7
    db "Container 7     200cm",0
    
start
    
    movlw	B'01110000'	;Set internal oscillator frequency to 8MHz
    movwf	OSCCON
    bsf		OSCTUNE, 6	;Enable PLL - oscillator speed = 32MHz
    
    bcf		RCON,7		;disable interrupt priority
    movlw	B'11001000'	
    movwf	INTCON		;config interrupts for delta B, en High,low priority
    movlw	B'10000001'	;low priority for portb changing, interupt on rising edge
    movwf	INTCON2		
    
    movlw	b'00000011'	;Analog In on 0, 1
    movwf	TRISA		;for PORTA
    movlw	b'11110011'	;keypad and ultrasonic inputs on PORTB
    movwf	TRISB		;for PORTB
    clrf	TRISC		;set PORTC as output    
    clrf	TRISD		;set PORTD as output

    clrf	LATA
    clrf	LATB
    clrf	LATC
    clrf	LATD

    movlw	B'00001101'	;configure ADCON1
    movwf	ADCON1		
    movlw	B'00110111'	;configure ADCON2
    movwf	ADCON2
    ;call	CONFIG_PWM	
    
    
    movlw	B'10011000'	;Configure Timer3 for PWM measurement, 1 for CPP
    movwf	T3CON		;8bit prescaler from Fosc/4 in 16 bit mode
    
    delay	0x50		;wait for LCD to initialie 
    call	LCD_INIT    
    
    
    readTable	Start_Msg
    call	DISP_TEXT
    call	READ_KEYPAD		;call in keypad. gets a value and returns
    readTable	Prog_Msg
    call	DISP_TEXT
read   
    ;call	ADC
    ;movwf	LATC		;write current state onto the PORTC LEDS
    ;call	READ_KEYPAD		
    ;bra		read		;repeat on keypress! Infinite loop! 
    
Disp_Data    
    call	PING
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
    delay 0xFF
    delay 0xFF
    delay 0xFF
    return
    
    end