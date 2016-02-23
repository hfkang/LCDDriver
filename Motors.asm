    #include <p18f4620.inc>
    #include <MACROS.inc>
    
#define STEP	LATD,1
    
    code
    global	CONFIG_PWM, STEPPER
    
CONFIG_PWM
    movlw	B'01100011'	;configure PR2
    movwf	PR2
    ;configure CCP Module #1
    movlw	B'00111010'	;configure duty cycle
    movwf	CCPR1L
    movlw	B'00011111'	;configure duty cycle and config
    movwf	CCP1CON		;disable PWM until i can remake the PCB 
   ;configure CCP Module #2
    movlw	B'00101010'	;configure CCPR2L
    movwf	CCPR2L
    movlw	B'00001100'	;configure CCP2CON
    movwf	CCP2CON
    movlw	B'00000101'	;configure T2CON, set prescaler to 4
    movwf	T2CON
    return 
    
STEPPER
    bsf		STEP
    bcf		STEP
    return
    end