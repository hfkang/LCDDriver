#define RS  LATD,2
#define E   LATD,3
#define LCD_DELAY_DURATION  0x9
#define	LCD_PORT    LATD

#define BUZZER	    LATC,0
#define	LCD_MASK    B'11110000'
#define	first_line  b'10000000'
#define second_line b'11000000'
#define	KEYPAD_DA   PORTB,1
#define	KEYPAD_PORT PORTA


#define	fourMeters	    0x06    ;check on the high bit for the encoders


#define	SENSOR_DELAY	    0XFF	;delay for reading in the sensor after a ping event 
#define	STICKER_DELAY	    0xFF
#define	IRBinScanOffset	    D'70'   
#define	poleAvoid	    D'60'
#define BinHalfway	    D'50'	;get away from bin!

#define FIR	PORTA,0
#define BIR	PORTA,1
#define TRIG	LATD,0
#define ECHO	PORTB,4		
#define	RComp	PORTC,6		;CHANGE LATER to proper port. 
#define	LComp	PORTC,7		;temporary. currently complifts with ultrasound
#define	WHITEH	0x02
#define WHITEL	0xD5
#define	SensorDelta D'50'
#define	T3Flag	PIR2,1

;#define	BINH	0x02
;#define BINL	0xB3

#define	DISTTHRESH	D'10'	;trigger for ultrasonic sensor 

#define STEP	LATC,5
#define	STEPDIR	LATD,1
#define STEP_ENABLE	LATB,7
#define	STEP_EXTEND	0x01
#define	STEP_RETRACT	0x00

#define DutyDefault	0x80
#define	ActualDefault	Duty50
#define	rampConstant	.180

#define	DutyMax		0x40
#define	Duty90		0x3A
#define	Duty85		.55
#define Duty825		D'53'
#define	Duty80		0x35
#define	Duty75		0x30
#define	Duty70		0x2C
#define	Duty50		0x40
#define	Duty60		.77
#define	Duty25		0x10
#define	Duty30		D'38'

#define	LeftSpeed	CCPR1L
#define	RightSpeed	CCPR2L   
#define LeftMotor	LATC,2
#define	RightMotor	LATC,1
#define LeftConfig	CCP1CON
#define	enablePWM	0x0C
#define	disablePWM	0x00
#define	RightConfig	CCP2CON
#define LeftDirection	LATE,1
#define RightDirection	LATE,0
#define	StepDelay	0x09

#define	STEPPER_SPEED	0xFE
#define STEPPER_SPEEDL	0xD0

#define	POLE_SPEED	0x25
#define poleOffset	D'75'		;about one full wheel rev (22cm)
#define poleRangeOffset	D'90'
#define	poleExtendOffset    D'70'
#define	armStepH	0x09
#define armStepL	0x5F
#define	retracted	armState,0
#define	extended	armState,1
#define	T3ENABLE	T3CON,0
#define	T3FLAG		PIR2,1
#define	T3INTENABLE	PIE2,1



#define     seconds     0x0
#define     minutes     0x1
#define     hours       0x2
#define     day         0x3
#define     date        0x4
#define     month       0x5
#define     year        0x6
#define     rtc_con     0x7