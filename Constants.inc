#define RS  LATD,2
#define E   LATD,3
#define LCD_DELAY_DURATION  0x20
#define	LCD_PORT    LATD
#define	LCD_MASK    0xF0

#define	fourMeters	    0x06    ;check on the high bit for the encoders

#define	SENSOR_DELAY	    0XFF	;delay for reading in the sensor after a ping event 
#define	STICKER_DELAY	    0xFF

#define FIR	PORTA,0
#define BIR	PORTA,1
#define TRIG	LATD,0
#define ECHO	PORTB,4
#define	LComp	PORTC,0		;CHANGE LATER to proper port. 
#define	RComp	PORTD,1		;temporary. currently complifts with ultrasound
#define	WHITETHRESH	D'100'
#define	BLACKTHRESH	0x50
#define	BLACKVERIFY	D'100'
#define	DISTTHRESH	D'10'	;trigger for ultrasonic sensor 

#define STEP	LATD,1
#define DutyDefault	B'00111111'
#define	LeftSpeed	CCPR1L
#define	RightSpeed	CCPR2L   
#define	StepDelay	0x01
#define	STEPPER_SPEED	    0x20

#define     seconds     0x0
#define     minutes     0x1
#define     hours       0x2
#define     day         0x3
#define     date        0x4
#define     month       0x5
#define     year        0x6
#define     rtc_con     0x7