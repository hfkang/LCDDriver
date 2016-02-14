;---------------- Binary (16-bit) to BCD -----------------------
;
;bin8_BCD:	; --- Takes Binary.number in      NumL
;bin16_BCD:	; --- Takes Binary.number in NumH:NumL 
		; --> Returns decimal.form  in TenK:Thou:Hund:Tens:Ones
;
; Uses variables (for which YOU MUST have laid out space in a CBlock:
; NumH, NumL
; TenK, Thou, Hund, Tens, Ones
    #include <p18f4620.inc>

    
    udata
NumH
NumL
TenK
Thou
Hund
Tens
Ones  
		
    code
    global bin8_BCD,bin16_BCD

bin8_BCD
	clrF	NumH

bin16_BCD
        swapf   NumH, W
        andlw   0x0F
        addlw   0xF0
        movwf   Thou 
        addwf   Thou, F 
        addlw   0xE2 
        movwf   Hund 
        addlw   0x32 
        movwf   Ones 

        movf    NumH, W 
        andlw   0x0F 
        addwf   Hund, F 
        addwf   Hund, F 
        addwf   Ones, F 
        addlw   0xE9 
        movwf   Tens 
        addwf   Tens, F 
        addwf   Tens, F 

        swapf   NumL, W 
        andlw   0x0F 
        addwf   Tens, F 
        addwf   Ones, F 

        rlcf    Tens, F 
        rlcf    Ones, F 
        comf	Ones, F 
        rlcf    Ones, F 

        movf    NumL, W 
        andlw   0x0F 
        addwf   Ones, F 
        rlcf    Thou,F 

        movlw   0x07 
        movwf   TenK 

        movlw   0x0A
Lb1 
        decf    Tens, F 
        addwf   Ones, F 
        btfss   STATUS, C 
        bra		Lb1 
Lb2
        decf    Hund, F 
        addwf   Tens, F 
        btfss   STATUS,C 
        bra		Lb2 
Lb3
        decf    Thou, F 
        addwf   Hund, F 
        btfss   STATUS,C
        bra		Lb3 
Lb4
        decf    TenK, F 
        addwf   Thou, F 
        btfss   STATUS,C 
        bra		Lb4 

        retlw   0
	end 



