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
    
dispText macro	Table,line
    local	_rdtbl
    movlw	line
    btfsc	WREG,6
    bra		_rdtbl
    lcdClear 
_rdtbl
    readTable	Table
    lcdInst	line
    call	DISP_TEXT
    endm
    
    
incBinNumber macro
	bcf	IRState,5
	bcf	IRState,4	;reset FSM state vector 
	incf	IRState	
	movff	IRState,tempBin	;tempBin for use in the destructive method lateron
	movff	IRState,BinNum
	endm
	
decBinNumber macro
	bcf	IRState,5
	bcf	IRState,4	;reset FSM state vector 
	decf	IRState	
	movff	IRState,tempBin	;tempBin for use in the destructive method lateron
	movff	IRState,BinNum
	endm
	
dispBinInfo macro   state
	local	    _iswhite,_isblack,_finih
	dispText    Biin,first_line
	movff	    CurrBin,NumL
	call	    bin8_BCD
	call	    Disp_Number
	btfsc	    state,0		;skip if black
	bra	    _iswhite
	bra	    _isblack
_iswhite
	dispText    isWhite,second_line
	bra	    _finih
_isblack
	dispText    isBlack,second_line
_finih
	endm 
	
	
 
storeBinData macro LocH, LocL, BinStatusReg, Side, Color 
	local	    _rear
	local	    _end
	movlw	    Side
	btfsc	    WREG,0
	bra	    _rear	;if the Side bit is set then store color only
	movff	    RightL, LocL	;this is for the front side bins
	movff	    RightH, LocH
	movlw	    Color	    	;store color in wreg to be stored in reg
	movwf	    BinStatusReg
	bra	    _end
_rear	movlw	    Color
	rlncf	    WREG,1
	mask_bits   BinStatusReg, B'00000010'	;write in the next bit 
_end	;dispBinInfo BinStatusReg
	endm 
	
storeBin macro BinNumber, Side, Color
	local	    bin1,bin2,bin3,bin4,bin5,bin6,bin7,_end
	
	decf	    BinNumber
	bz	    bin1
	decf	    BinNumber
	bz	    bin2
	decf	    BinNumber
	bz	    bin3
	decf	    BinNumber
	bz	    bin4
	decf	    BinNumber
	bz	    bin5
	decf	    BinNumber
	bz	    bin6
	bra	    bin7
bin1	storeBinData B1H, B1L, B1S, Side, Color
	bra	    _end
bin2	storeBinData B2H, B2L, B2S, Side, Color
	bra	    _end	
bin3	storeBinData B3H, B3L, B3S, Side, Color
	bra	    _end	
bin4	storeBinData B4H, B4L, B4S, Side, Color
	bra	    _end
bin5	storeBinData B5H, B5L, B5S, Side, Color
	bra	    _end
bin6	storeBinData B6H, B6L, B6S, Side, Color
	bra	    _end
bin7	storeBinData B7H, B7L, B7S, Side, Color
	bra	    _end
_end	
	endm
	
stopPWM macro
	movlw	    0x00
	mask_bits   CCP1CON, 0x0F
	movlw	    0x00
	mask_bits   CCP2CON, 0x0F
	endm
	
startPWM macro
	movlw	    0x0C
	mask_bits   CCP1CON, 0x0F
	movlw	    0x0C
	mask_bits   CCP2CON, 0x0F	
	movlw	    DutyDefault
	movwf	    PR2
	movlw	    B'00000101'	;configure T2CON, set prescaler to 4
	movwf	    T2CON


	endm
	
toggleMotor macro    Side, Setting
	movlw	     Setting
	mask_bits    Side, 0x0F
	endm
	
disp16bit macro high, low
	movff	low,NumL
	movff	high,NumH
	call	bin16_BCD
	call	Disp_Number
	endm
	

;This macro does a 16 bit comaprison operation, and branches depending on how the values turn out
comp16 macro varH,varL,gotoHigh,gotoLow,gotoEq
	local	    _geeq,_eq,_geeqlow
	banksel	    threshH
	movf	    threshH,W
	cpfslt	    varH   
	bra	    _geeq	    ;since varH >= threshH, go to high method
	goto	    gotoLow	    ;definitely lower

_geeq
	cpfseq	    varH
	goto	    gotoHigh	    ;definitely higher
_eq
	movf	    threshL,W
	cpfslt	    varL	    ;high bits are equal, idk about low bits
	bra	    _geeqlow	    ;high bits are equal, but lower bit is higher or equal
	goto	    gotoLow	    ;high was equal, but lower is low
_geeqlow
	cpfseq	    varL
	goto	    gotoHigh	    ;lower register is higher than threshold
	goto	    gotoEq	    ;both registers are totally equal 
	endm
	
enccomp16 macro varH,varL,gotoHigh,gotoLow,gotoEq
	local	    _geeq,_eq,_geeqlow
	banksel	    varH
	movf	    encThreshH,W
	cpfslt	    varH   
	bra	    _geeq	    ;since varH >= threshH, go to high method
	goto	    gotoLow	    ;definitely lower

_geeq
	cpfseq	    varH
	goto	    gotoHigh	    ;definitely higher
_eq
	movf	    encThreshL,W
	cpfslt	    varL	    ;high bits are equal, idk about low bits
	bra	    _geeqlow	    ;high bits are equal, but lower bit is higher or equal
	goto	    gotoLow	    ;high was equal, but lower is low
_geeqlow
	cpfseq	    varL
	goto	    gotoHigh	    ;lower register is higher than threshold
	goto	    gotoEq	    ;both registers are totally equal 
	endm
	
disp16 macro high,low	
    movff	low,NumL
    movff	high,NumH
    call	bin16_BCD
    call	Disp_Number
    endm
    
    
add16 macro fileH,fileL,operand
	banksel	    fileH
	movlw	    operand
	addwf	    fileL,F
	btfsc	    STATUS,C
	incf	    fileH	
	endm

sub16	macro fileH,fileL,operand
	banksel	    fileH
	movlw	    operand 
	subwf	    fileL,F
	btfss	    STATUS,C
	decf	    fileH
	endm	
	
movlf   macro   literal, register
	movlw   literal
	movwf   register
	endm
	
stopStepperMotor macro
    bcf	    T3ENABLE
    bcf	    T3INTENABLE
    bsf	    STEP_ENABLE
    endm
    
extendStepper macro
    bsf	    STEPDIR
    ;bsf	    T3ENABLE
    ;bsf	    T3INTENABLE
    bcf	    STEP_ENABLE
    endm

retractStepper macro
    bcf	    STEPDIR
    ;bsf	    T3ENABLE
    ;bsf	    T3INTENABLE
    bsf	    STEP_ENABLE
    endm

	
encOffset macro	    offset
	local	    _add,_sub,_fin,_add1,_sub1
	banksel	    encThreshH
	movff	    RightH,encThreshH
	movff	    RightL,encThreshL
	btfsc	    direction,0		;add if forward, subtract if backward
	bra	    _sub		;subtract because its set, & not skipped
_add	add16	    encThreshH,encThreshL,offset	;offset should be a literal (constant)
_add1	call	    PID
	banksel	    threshH
	lcdHomeLine
	call	    disp_encoders
	enccomp16	    RightH,RightL,_fin,_add1,_add1
_sub	sub16	    encThreshH,encThreshL,offset
_sub1	call	    PID
	banksel	    threshH
	lcdHomeLine
	call	    disp_encoders
	enccomp16	    RightH,RightL,_sub1,_fin,_sub1
_fin	
	endm
	
rtc_disp    macro   rtc_data
	rtc_convert rtc_data
	movff       tens_digit, WREG
	lcdData
	movff       ones_digit, WREG
	lcdData
	endm
	 
divby8	macro reg
	rlncf	reg
	rlncf	reg
	;rrncf	reg
	;bcf	reg,7
	;bcf	reg,6
	nop
	endm

divby4	macro	reg
	rrncf	reg
	;rrncf	reg
	bcf	reg,7
	;bcf	reg,6
	endm
	
resetrolling macro
	movlf	0xFF,ultra1
	movlf	0xFF,ultra2
	movlf	0xFF,ultra3
	movlf	0xFF,ultra4
	endm
	
rollingavg macro reg,dest
 	movff	reg,comparetemp
	divby4  comparetemp
	movf	comparetemp,w
	addwf	dest
	endm

cpfsne	  macro   f1,f2
          movf    f2,w
          subwf   f1,w
          btfsc   STATUS,Z
          endm	
	
i2c_start       macro
        bsf     SSPCON2, SEN
        call    CheckI2C
        endm
i2c_stop        macro
        bsf     SSPCON2, PEN
        call    CheckI2C
        endm
i2c_write       macro
        movwf   SSPBUF
        call    CheckI2C
        endm
rd_i2c_buf_ack  macro   rtc_reg
        bsf     SSPCON2, RCEN
        call    CheckI2C
        i2c_common_ack
        movff   SSPBUF, rtc_reg
        endm
rd_i2c_buf_nack macro   rtc_reg
        bsf     SSPCON2, RCEN
        call    CheckI2C
        i2c_common_nack
        movff   SSPBUF, rtc_reg
        endm
i2c_common_ack  macro
        bcf     SSPCON2,ACKDT
        bsf     SSPCON2,ACKEN
        call    CheckI2C
        endm
i2c_common_nack	macro
        bsf      SSPCON2,ACKDT
        bsf      SSPCON2,ACKEN
        call     CheckI2C
        endm
rtc_wr   macro   address, rtc_data
        i2c_start
        movlw       0xD0            ; RTC address
        i2c_write
        movlw       address         ; set register address on I2C
        i2c_write
        movlw       rtc_data        ; write data to RTC register
        i2c_write
        i2c_stop
        endm
rtc_convert macro   rtc_data
        swapf       rtc_data, W
        andlw       B'00001111'     ; masks all but MSBs of rtc_data
        addlw       0x30            ; required to convert to ASCII
        movwf       tens_digit      ; store in tens_digit
        movf        rtc_data, W
        andlw       B'00001111'     ; masks all but LSBs of rtc_data
        addlw       0x30            ; required to convert to ASCII
        movwf       ones_digit      ; store in ones_digit
        endm
	
reset_encoders macro
	clrf	    RightH
	clrf	    RightL
	clrf	    LeftL
	clrf	    LeftH
	endm