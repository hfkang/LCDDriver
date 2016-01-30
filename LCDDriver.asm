    list p=18F4620
    #include <p18f4620.inc>
    CONFIG OSC=HS, IESO = ON
    
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
    org 8
    goto Stop
    org 0x18
    goto Stop

  

MAIN CODE 
start
    movlw 0
    movwf lcdCursor
    clrf LATD
    movlw 0x0F
    movwf ADCON1
    clrf TRISD
    delay 0xFF
    
    lcdInst B'00110011'
    lcdInst B'00110010'
    lcdInst B'00101000'
    lcdInst B'00001111'
    lcdInst B'00000110'
    lcdInst B'00000001' 
    
    

    
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
    db "Prepare your butts.",0      
    
    
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