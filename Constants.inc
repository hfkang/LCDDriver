#define RS  LATD,2
#define E   LATD,3
#define LCD_DELAY_DURATION  0x20
#define	LCD_PORT    LATD
#define	LCD_MASK    B'11110000'
#define	first_line  b'10000000'
#define second_line b'11000000'
#define	KEYPAD_DA   PORTB,1
#define	KEYPAD_PORT PORTB

#define	fourMeters	    0x06    ;check on the high bit for the encoders

#define	SENSOR_DELAY	    0XFF	;delay for reading in the sensor after a ping event 
#define	STICKER_DELAY	    0xFF
#define	IRBinScanOffset	    D'20'    	;4cm in encoder ticks 
#define	IRStickerScanOffset D'98'	;25cm to get to sticker 

#define FIR	PORTA,0
#define BIR	PORTA,1
#define TRIG	LATC,5
#define ECHO	PORTB,4
#define	LComp	PORTC,7		;CHANGE LATER to proper port. 
#define	RComp	PORTC,6		;temporary. currently complifts with ultrasound
#define	WHITETHRESH	D'100'
#define	BLACKTHRESH	0x50
#define	BLACKVERIFY	D'100'
#define	DISTTHRESH	D'10'	;trigger for ultrasonic sensor 

#define STEP	LATC,2
#define	STEPDIR	LATC,5		
#define	STEP_EXTEND	0x01
#define	STEP_RETRACT	0x00
#define DutyDefault	0x40
#define	Duty75		0x30
#define	Duty50		0x20
#define	Duty25		0x10

#define	LeftSpeed	CCPR1L
#define	RightSpeed	CCPR2L   
#define	StepDelay	0x01
#define	STEPPER_SPEED	0x20
#define poleOffset	0x5A		;about one full wheel rev (22cm)


#define     seconds     0x0
#define     minutes     0x1
#define     hours       0x2
#define     day         0x3
#define     date        0x4
#define     month       0x5
#define     year        0x6
#define     rtc_con     0x7