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
lcdClear macro
    lcdInst 0x1	
    endm
    
    ;Overrites value with working register, using mask
mask_bits macro value, mask
    xorwf value,W
    andlw mask
    xorwf value,F
    endm 
  
word_wrap macro lcdCursor
	local		LineWrap
	local		EndCheck
	incf		lcdCursor
	movlw		0x11
	subfwb		lcdCursor,W
	bnz		LineWrap
	lcdNewLine 
LineWrap movlw		0x20
	 subfwb		lcdCursor,W
	 bnz		EndCheck
	 lcdHomeLine
	 movlw		0
	 movwf		lcdCursor
EndCheck nop
	 endm
	 
readTable macro TableLabel
    movlw upper TableLabel
    movwf TBLPTRU
    movlw high TableLabel
    movwf TBLPTRH
    movlw low TableLabel
    movwf TBLPTRL
    endm
