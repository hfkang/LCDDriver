    #include <p18f4620.inc>
    #include <MACROS.inc>
    #include <Constants.inc>
        
   udata 
delay1	    res 1
delay2	    res 1
delay3	    res 1
lcd_buffer  res 1
lcdCursor   res 1
keypress    res 1 

    code 
    global LCD_INIT, DELAY_ROUTINE, NIBBLE_LCD, DISP_TEXT, READ_KEYPAD, keypress    
    
    
READ_KEYPAD     
	 btfss		PORTB,1   ;Wait until data is available from the keypad
         goto		READ_KEYPAD
         swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
         andlw		0x0F
	 btfsc		PORTB,1     ;Wait until key is released
         goto		$-2
	 ;word_wrap	lcdCursor
	 ;goto		test
	 return 

   

DISP_TEXT   	    
	lcdHomeLine
	lcdClear
	movlw 0
	movwf lcdCursor
	tblrd*
	movf    TABLAT,W
Again
    lcdData
    word_wrap lcdCursor
    tblrd+*
    movf TABLAT,W
    bnz Again
    return 

LCD_INIT
    lcdInst B'00110011'
    lcdInst B'00110010'
    lcdInst B'00101000'
    lcdInst B'00001111'
    lcdInst B'00000110'
    lcdInst B'00000001' 
    
    movlw 0
    movwf lcdCursor
    
    return 
    

    
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
    
  
NIBBLE_LCD
    ;Recieve data in working reg.
    movwf lcd_buffer	;set aside inst.
    mask_bits LCD_PORT,LCD_MASK
    bcf E
    delay LCD_DELAY_DURATION
    bsf E    
    delay LCD_DELAY_DURATION
    swapf lcd_buffer,1	    
    movf lcd_buffer, 0	    ;recover original data
    mask_bits LCD_PORT,LCD_MASK
    bcf E
    delay LCD_DELAY_DURATION
    bsf E
    delay LCD_DELAY_DURATION
    return 
    
    end