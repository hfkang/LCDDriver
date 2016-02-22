    #include <p18f4620.inc>
    #include <MACROS.inc>
    extern bin16_BCD
    extern LCD_INIT, DELAY_ROUTINE, NIBBLE_LCD, DISP_TEXT, READ_KEYPAD, keypress

    
#define FIR	PORTA,0
#define BIR	PORTA,1
#define TRIG	LATD,0
#define ECHO	PORTB,4
#define RS	LATD,2
#define E	LATD,3
#define	LComp	PORTD,0
#define	RComp	PORTD,1		;temporary. currently complifts with ultrasound
    
    udata
pingtime    res	   1
pingH	    res	   1
NumH	    res	    1
NumL	    res	    1
TenK	    res	    1
Thou	    res	    1
Hund	    res	    1
Tens	    res	    1
Ones	    res	    1
LeftL	    res	    1
LeftH	    res	    1
RightL	    res	    1
RightH	    res	    1
     
   

    code
    global	ADC,PING,DIST,pingtime, pingH, NumH, NumL, TenK, Thou ,Hund, Tens, Ones,ENCODER1,ENCODER2, LeftL, LeftH, disp_encoders,RightL,RightH
    
ADC	movlw	B'00000001'	;configure ADCON0 for AN0 input (RA0)
     	movwf	ADCON0		
     	bsf	ADCON0,1	;start the conversion

WAIT	btfsc	ADCON0,1	;wait until the conversion is completed
     	bra	WAIT		;poll the GO bit in ADCON0
     	movf	ADRESH,W	;move the high 8-bit to W
     	return

Disp_Number
	movlw	0x30
	addwf	TenK,W
	lcdData
	movlw	0x30
	addwf	Thou,W
	lcdData
	movlw	0x30
	addwf	Hund,W
	lcdData
	movlw	0x30
	addwf	Tens,W
	lcdData
	movlw	0x30
	addwf	Ones,W
	lcdData
	return
	
disp_encoders 
	lcdClear
	movff	LeftL,NumL
	movff	LeftH,NumH
	call	bin16_BCD
	call	Disp_Number
	movff	RightL,NumL
	movff	RightH,NumH
	call	bin16_BCD
	call	Disp_Number
	return
	
PING	call	bin16_BCD	;convert the previous values to BCD notation
	lcdClear
	call	Disp_Number
	
	bsf	TRIG		;delay 0x2 gives 15 us approximately at 8Mhz
	delay	0x2
	bcf	TRIG
	return
	
DIST	btfss	PORTB,4		;if RB4 is high, reset and start timer 3
	goto	echo_b
echo_a	clrf	TMR0H
	clrf	TMR0L		;clear register and prescaler
	bsf	T0CON,7		;enable timer
	bcf	INTCON,0	;reset interrupt vector 
	retfie	1
echo_b	bcf	T0CON,7		;disable timer
	;movff	TMR3L,NumL
	;movff	TMR3H,NumH
	movf	TMR0L,W
	movwf	NumL
	movf	TMR0H,W
	movwf	NumH
	movff	TMR0H,LATC
	bcf	INTCON,0	;reset interrupt vector
	retfie	1			;return from interrupt
	
ENCODER1
	btfsc	LComp		;check direction of encoder
	goto	for
	goto	back
	
for	incf	LeftL
	btfsc	STATUS,C
	incf	LeftH
	goto	transf
back	decf	LeftL
	btfss	STATUS,C
	decf	LeftH
transf	bcf	INTCON3,1	;reset flag bit
	retfie	1
	
ENCODER2
	btfss	RComp		;check direction of encoder. directions reversed
	goto	for2
	goto	back2
	
for2	incf	RightL
	btfsc	STATUS,C
	incf	RightH
	goto	transf2
back2	decf	RightL
	btfss	STATUS,C
	decf	RightH
transf2	bcf	INTCON3,0	;reset flag bit
	retfie	1
	
	end

    

