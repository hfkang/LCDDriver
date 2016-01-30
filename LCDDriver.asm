    #include <p18f4620.inc>

    list P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC

;;;;;;Configuration Bits;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
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

    
    
    cblock 0xD0
    delay1
    delay2
    delay3
    lcdCursor
    lcd_buffer
    endc
    
#define RS  LATD,2
#define E   LATD,3
    

    
delay macro duration
    movlw duration	;will implement proper calcuations later
    call DELAY_ROUTINE	;calls subroutine to handle delay
    endm

lcdInst macro instruction
    bcf RS  ;instruction register
    bsf E   ;prepare to be set down
    movlw instruction
    call NIBBLE_LCD
    endm
lcdNewLine macro	    ;go to 2nd line
    lcdInst B'11000000'
    endm
lcdHomeLine macro	    ;go to first line
    lcdInst B'10000000'
    endm
lcdData macro
    bsf RS  ;data register
    bsf E   ;prepare to be clocked down
    call NIBBLE_LCD
    endm
    
    
    org 0
    goto start
    org 0x8
    retfie
    org 0x18
    retfie 
  

MAIN CODE 
start
    movlw 0
    movwf lcdCursor
    clrf LATD
    movlw 0x0F
    movwf ADCON1
    clrf TRISD
    clrf INTCON
    clrf TRISA
    movlw b'1110010'
    movwf TRISB
    clrf    TRISC
    clrf    LATA
    clrf    LATB
    clrf    LATC
    
    
    
    lcdInst B'00110011'
    lcdInst B'00110010'
    lcdInst B'00101000'
    lcdInst B'00001111'
    lcdInst B'00000110'
    lcdInst B'00000001' 
    
test     btfss		PORTB,1   ;Wait until data is available from the keypad
         goto		test

         swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
         andlw		0x0F
	 rlncf		WREG, W		;Program Memory in PIC18 counts up by 2
         call     	KPHexToChar ;Convert keypad value to LCD character (value is still held in W)
	 lcdData
	 
         btfsc		PORTB,1     ;Wait until key is released
         goto		$-2
         goto    	test

KPHexToChar
          addwf     PCL,f
          dt        "123A456B789C*0#D"

          

    
ReadTable
    movlw upper Table
    movwf TBLPTRU
    movlw high Table 
    movwf TBLPTRH
    movlw low Table
    movwf TBLPTRL
    
    tblrd*
    movf    TABLAT,W 
    
Again
    lcdData
    incf lcdCursor, F ;move forward one character
    movlw 0x10
    subwf lcdCursor, W
    bnz Read
    lcdNewLine
Read   tblrd+*
    movf TABLAT,W
    bnz Again
    
Stop bra Stop
	
Table
    db "Prepare thy     anuses.",0      
    
    
DELAY_ROUTINE 
    movwf delay1
    movwf delay2
    movwf delay3
Loop3
    movwf delay2
Loop2
    movwf delay1
Loop1
    decfsz delay1,F
	goto Loop1
    decfsz delay2,F
	goto Loop2
    decfsz delay3,F
    return
    
;Overrites value with working register, using mask
mask_bits macro value, mask
    xorwf value,W
    andlw mask
    xorwf value,F
    endm 
    
NIBBLE_LCD
    ;Recieve data in working reg.
    movwf lcd_buffer	;set aside inst.
    mask_bits LATD,0xF0
    bcf E
    delay 0x2F
    bsf E
    swapf lcd_buffer,1	    
    movf lcd_buffer, 0	    ;recover original data
    mask_bits LATD,0xF0
    bcf E
    delay 0x2F
    bsf E
    return 
    end