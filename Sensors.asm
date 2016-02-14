    #include <p18f4620.inc>
    #include <MACROS.inc>
    extern DELAY_ROUTINE
    
#define FIR	PORTA,0
#define BIR	PORTA,1
#define TRIG	LATD,0
#define ECHO	PORTB,4
    
    cblock
    pingtime
    pingH
    NumH
    NumL
    TenK
    Thou
    Hund
    Tens
    Ones
    endc 
   

    code
    global	ADC,PING,DIST
    
ADC	movlw	B'00000001'	;configure ADCON0 for AN0 input (RA0)
     	movwf	ADCON0		
     	bsf	ADCON0,1	;start the conversion

WAIT	btfsc	ADCON0,1	;wait until the conversion is completed
     	bra	WAIT		;poll the GO bit in ADCON0
     	movf	ADRESH,W	;move the high 8-bit to W
     	return
	
PING	movff	TMR3L,LATC	;debug : show data from last trial
	bsf	TRIG		;delay 0x5 gives 13.25 us approximately 
	delay	0x5
	bcf	TRIG
	return
	
oldPING	bsf	TRIG		;delay 0x5 gives 13.25 us approximately 
	delay	0x5
	bcf	TRIG		
	btfss	ECHO
	bra	$-2		;wait until we get a ping back from sensor
	incf	pingtime	;incremnet counter
	bnov	$+4
	incf	pingH
	btfsc	ECHO
	bra	$-8		;kount until zero
	movff	pingtime,LATC
	clrf	pingH
	clrf	pingtime
	
	
	delay	0xFF
	delay	0xFF
	delay	0xFF
	
	bra	PING
	return
	
DIST	
	btfsc	PORTB,4		;if RB4 is high, reset and start timer 3
	goto	echo_b
echo_a	clrf	TMR3H
	clrf	TMR3L		;clear register and prescaler
	bsf	T3CON,0		;enable timer
	bcf	INTCON,0
	retfie
echo_b	
	bcf	T3CON,0		;disable timer
	bcf	INTCON,0	;reset interrupt vecotr
	movff	TMR3H,pingtime	;export timer data
	retfie			;return from interrupt
	
	end

    

