
#include "main.h"

/*
https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328

// CS BITS
CS02	CS01    CS00 	 DESCRIPTION
0	0 	0 	 Timer/Counter0 Disabled 
0	0 	1 	 No Prescaling
0	1 	0 	 Clock / 8
0	1 	1 	 Clock / 64
1	0 	0 	 Clock / 256
1	0 	1 	 Clock / 1024

CS12	 CS11 	 CS10 	 DESCRIPTION
0	0 	0 	 Timer/Counter1 Disabled 
0	0 	1 	 No Prescaling
0	1 	0 	 Clock / 8
0	1 	1 	 Clock / 64
1	0 	0 	 Clock / 256
1	0 	1 	 Clock / 1024

CS22	 CS21 	 CS20 	 DESCRIPTION
0	0 	0 	 Timer/Counter2 Disabled 
0	0 	1 	 No Prescaling
0	1 	0 	 Clock / 8
0	1 	1 	 Clock / 32
1	0 	0 	 Clock / 64
1	0 	1 	 Clock / 128
1	1 	0 	 Clock / 256
1	1 	1 	 Clock / 1024 


// WAVEFORM GENERATOR BITS
	WGM02	WGM01	WGM00	 DESCRIPTION	 	TOP
0	0 	0	0	 Normal 	 	0xFF
1	0	0	1	 PWM, Phase Corrected	0xFF
2	0	1	0	 CTC			OCR0A
3	0	1	1	 Fast PWM		0xFF
4	1	0	0	 Reserved	 	-
5	1	0	1	 Fast PWM, Phase Corr	OCR0A
6	1	1	0	 Reserved		-
7	1	1	1	 Fast PWM		OCR0A

MODE	WGM13	WGM12	WGM11	WGM10	 DESCRIPTION            	 TOP
0	 0	 0 	0	0	 Normal 	                 0xFFFF
1	0	0	0	1	 PWM, Phase Corrected, 8bit	 0x00FF
2	0	0	1	0	 PWM, Phase Corrected, 9bit	 0x01FF
3	0	0	1	1	 PWM, Phase Corrected, 10bit 	 0x03FF 
4	0	1	0	0        CTC	                         OCR1A 
5	0	1	0	1	 Fast PWM, 8bit 	          0x00FF 
6	0	1	1	0	 Fast PWM, 9bit 	          0x01FF 
7	0	1	1	1	 Fast PWM, 10bit 	          0x03FF 
8	1	0	0	0	 PWM, Phase and Frequency Corr    ICR1 
9	1	0	0	1	 PWM, Phase and Frequency Corr    OCR1A 
10	1	0	1	0	 PWM, Phase Correct 	          ICR1 
11	1	0	1	1	 PWM, Phase Correct 	         OCR1A
12	1	1	0	0	 CTC	                         ICR1
13	1	1	0	1	 RESERVED	 
14	1	1	1	0	 Fast PWM 	                  ICR1 
15	1	1	1	1	 Fast PWM	                  OCR1A 

MODE	WGM21	WGM20	 DESCRIPTION	          TOP
0	0	0	 Normal 	         0xFF
1	0	1	 PWM Phase Corrected	 
2	1	0	 CTC	                  OCR2
3	1	1	 Fast PWM 	 



x = Timer Number
 	7 bit	 6 bit 	 5 bit 	 4 bit 	 3 bit 	 2 bit 	 1 bit 	 0 bit     Description
TCCRxA	COMxA1	 COMxA0  COMxB1  COMxB0  -	 -	 WGMx1	 WGMx0     Timer/Counter Control Register x A (x=0,2)

TCCR1B	ICNC1	 ICES1	 -	 WGM13	 WGM12	 CS12	 CS11	CS10 
TCCRxB	FOCxA    FOCxB   -       -       WGMx2   CSx2    CSx1    CSx0      Timer/Counter Control Register x B

TIMSKx	-        -       -       -       -       OCIExB  OCIExA  TOIEx     Timer/Counter Interrupt Mask Register
TIFRx	-	 -	 -	 -       -       OCFxB	 OCFxA   TOVx      Timer/Counter Interrupt Flag Register
TCNTx                                                                      Timer/Counter Register (stores the counter value)
OCRxA                                                                      Output Compare Register x A
OCRxB                                                                      Output Compare Register x B


*/

void motorInterrupt();
void motorInitInterrupt();

#define BL_PHASE_CONFIG 1

#if (BL_PHASE_CONFIG == 1)
#define BL_PHASE_A  0
#define BL_PHASE_B  85
#define BL_PHASE_C  170

#elif (BL_PHASE_CONFIG == 2)
#define BL_PHASE_A  0
#define BL_PHASE_B  170
#define BL_PHASE_C  85

#elif (BL_PHASE_CONFIG == 3)
#define BL_PHASE_A  85
#define BL_PHASE_B  0
#define BL_PHASE_C  170

#elif (BL_PHASE_CONFIG == 4)
#define BL_PHASE_A  85
#define BL_PHASE_B  170
#define BL_PHASE_C  0

#elif (BL_PHASE_CONFIG == 5)
#define BL_PHASE_A  170
#define BL_PHASE_B  0
#define BL_PHASE_C  85

#elif (BL_PHASE_CONFIG == 6)
#define BL_PHASE_A  170
#define BL_PHASE_B  85
#define BL_PHASE_C  0

#endif

//compatibilità
uint16 motors_pwm_period = 0;	//periodo (ovvero il COUNT) del PWM, quando duty è pari a questo valore, l'uscita è sempre alta

void pinSetPwm(uint8 pin, uint16 freq, uint16 dutyPerMille)
{
	uint16 period = pwmSetFrequency(pin, freq);
	uint16 duty2 = period * dutyPerMille / 1000;
	pwmWrite(pin, duty2);
}

void pinSetDuty(uint8 pin, uint16 dutyPerMille)
{
	uint16 duty2 = motors_pwm_period * dutyPerMille / 1000;
	pwmWrite(pin, duty2);
}


void init_motors(void)
{
	Serial.print("Init MOTORs\r\n");
	pinMode(BOARD_MOTOR_EN, OUTPUT);
	Serial.print("Disable MOTOR output\r\n");
	digitalWrite(BOARD_MOTOR_EN, LOW);

	Serial.print("Init MOTOR SENSE\r\n");
	pinMode(BOARD_MOT1_ISENSE, INPUT_ANALOG);
	pinMode(BOARD_MOT2_ISENSE, INPUT_ANALOG);
	pinMode(BOARD_MOT2_ISENSE, INPUT_ANALOG);

	Serial.print("Init PWM for MOTORs\r\n");
	uint16 freq = GIMBAL_PWM_FREQ;

#ifdef PWM_8KHZ_FAST
	freq = 8000;
#endif

#ifdef PWM_32KHZ_PHASE
	freq = 32000;
#endif

#ifdef PWM_4KHZ_PHASE
	freq = 4000;
#endif


	uint16 duty = 0;

	pinMode(BOARD_MOT1_CMD_A, PWM);
	pinMode(BOARD_MOT1_CMD_B, PWM);
	pinMode(BOARD_MOT1_CMD_C, PWM);
	pinMode(BOARD_MOT2_CMD_A, PWM);
	pinMode(BOARD_MOT2_CMD_B, PWM);
	pinMode(BOARD_MOT2_CMD_C, PWM);
	pinMode(BOARD_MOT3_CMD_A, PWM);
	pinMode(BOARD_MOT3_CMD_B, PWM);
	pinMode(BOARD_MOT3_CMD_C, PWM);
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	delay(100);




	uint16 period = pwmSetFrequency(BOARD_MOT1_CMD_A, freq);
	pwmSetFrequency(BOARD_MOT1_CMD_B, freq);
	pwmSetFrequency(BOARD_MOT1_CMD_C, freq);
	pwmSetFrequency(BOARD_MOT2_CMD_A, freq);
	pwmSetFrequency(BOARD_MOT2_CMD_B, freq);
	pwmSetFrequency(BOARD_MOT2_CMD_C, freq);
	pwmSetFrequency(BOARD_MOT3_CMD_A, freq);
	pwmSetFrequency(BOARD_MOT3_CMD_B, freq);
	pwmSetFrequency(BOARD_MOT3_CMD_C, freq);



	pwmWrite(BOARD_MOT1_CMD_A, duty);
	pwmWrite(BOARD_MOT1_CMD_B, duty);
	pwmWrite(BOARD_MOT1_CMD_C, duty);
	pwmWrite(BOARD_MOT2_CMD_A, duty);
	pwmWrite(BOARD_MOT2_CMD_B, duty);
	pwmWrite(BOARD_MOT2_CMD_C, duty);
	pwmWrite(BOARD_MOT3_CMD_A, duty);
	pwmWrite(BOARD_MOT3_CMD_B, duty);
	pwmWrite(BOARD_MOT3_CMD_C, duty);

	motors_pwm_period = period;
	recalcMotorStuff();


	Serial.print("Enable MOTORs\r\n");

	//Serial.printf("PWM period %d\r\nPWM max    %d\r\nPWM sin_x  %d\r\n", motors_pwm_period, motors_pwm_max , motors_sin_count);

	digitalWrite(BOARD_MOTOR_EN, HIGH);
	/*
	delay(10);
	uint8 v = digitalRead(BOARD_MOT1_ISENSE);
	if (v != 0)
	{
		Serial.print("Detected motor 1\r\n");

	}
	//motorMove(0, 0);
	v = digitalRead(BOARD_MOT2_ISENSE);
	 if (v != 0)
	 {
		Serial.print("Detected motor 2\r\n");

	 }
	 //motorMove(1, 0);
	 v = digitalRead(BOARD_MOT3_ISENSE);
	 if (v != 0)
	 {
		Serial.print("Detected motor 3\r\n");

	 }
	 motorMove(2, 0);
*/
}

void initBlController()
{
	init_motors();
	sei();

	motorInitInterrupt();
}


inline void setPwmSin(int pin, int pos, pwmsin_t* pwmSin)
{
	pos &= (N_SIN - 1);
	pwmWrite(pin, pwmSin[pos]);
}

// 3 lsb of MotorPos still reserved for precision improvement (TBD) 
inline void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, pwmsin_t* pwmSin)
{
  int posStep;

  if (motorNumber == 0)
  {
    posStep = MotorPos >> 3;
    //posStep &= 0xff;
    //pwmWrite(BOARD_MOT1_CMD_A, pwmSin[(uint8_t)posStep + BL_PHASE_A]);
    //pwmWrite(BOARD_MOT1_CMD_B, pwmSin[(uint8_t)(posStep + BL_PHASE_B)]);
    //pwmWrite(BOARD_MOT1_CMD_C, pwmSin[(uint8_t)(posStep + BL_PHASE_C)]);
    setPwmSin(BOARD_MOT1_CMD_A, posStep + BL_PHASE_A, pwmSin);
    setPwmSin(BOARD_MOT1_CMD_B, posStep + BL_PHASE_B, pwmSin);
    setPwmSin(BOARD_MOT1_CMD_C, posStep + BL_PHASE_C, pwmSin);
  }
 
  if (motorNumber == 1)
  {
    posStep = MotorPos >> 3;
//    posStep &= 0xff;
//    pwmWrite(BOARD_MOT2_CMD_A, pwmSin[(uint8_t)posStep + BL_PHASE_A]);
//    pwmWrite(BOARD_MOT2_CMD_B, pwmSin[(uint8_t)(posStep + BL_PHASE_B)]);
//    pwmWrite(BOARD_MOT2_CMD_C, pwmSin[(uint8_t)(posStep + BL_PHASE_C)]);
    setPwmSin(BOARD_MOT2_CMD_A, posStep + BL_PHASE_A, pwmSin);
	setPwmSin(BOARD_MOT2_CMD_B, posStep + BL_PHASE_B, pwmSin);
	setPwmSin(BOARD_MOT2_CMD_C, posStep + BL_PHASE_C, pwmSin);
  }

  if (motorNumber == 2)
  {
    posStep = MotorPos >> 3;
//    posStep &= 0xff;
//    pwmWrite(BOARD_MOT3_CMD_A, pwmSin[(uint8_t)posStep + BL_PHASE_A]);
//    pwmWrite(BOARD_MOT3_CMD_B, pwmSin[(uint8_t)(posStep + BL_PHASE_B)]);
//    pwmWrite(BOARD_MOT3_CMD_C, pwmSin[(uint8_t)(posStep + BL_PHASE_C)]);
    setPwmSin(BOARD_MOT3_CMD_A, posStep + BL_PHASE_A, pwmSin);
	setPwmSin(BOARD_MOT3_CMD_B, posStep + BL_PHASE_B, pwmSin);
	setPwmSin(BOARD_MOT3_CMD_C, posStep + BL_PHASE_C, pwmSin);
  }
}



void fastMoveMotor(uint8_t motorNumber, int dirStep,pwmsin_t* pwmSin)
{
  if (motorNumber == 0)
  {
    currentStepMotor0 += dirStep;
//    currentStepMotor0 &= 0xff;
//    pwmWrite(BOARD_MOT1_CMD_A, pwmSin[currentStepMotor0 + BL_PHASE_A]);
//    pwmWrite(BOARD_MOT1_CMD_B, pwmSin[(uint8_t)(currentStepMotor0 + BL_PHASE_B)]);
//    pwmWrite(BOARD_MOT1_CMD_C, pwmSin[(uint8_t)(currentStepMotor0 + BL_PHASE_C)]);
    setPwmSin(BOARD_MOT1_CMD_A, currentStepMotor0 + BL_PHASE_A, pwmSin);
	setPwmSin(BOARD_MOT1_CMD_B, currentStepMotor0 + BL_PHASE_B, pwmSin);
	setPwmSin(BOARD_MOT1_CMD_C, currentStepMotor0 + BL_PHASE_C, pwmSin);
  }
 
  if (motorNumber == 1)
  {
    currentStepMotor1 += dirStep;
//    currentStepMotor1 &= 0xff;
//    pwmWrite(BOARD_MOT2_CMD_A, pwmSin[currentStepMotor1 + BL_PHASE_A]) ;
//    pwmWrite(BOARD_MOT2_CMD_B, pwmSin[(uint8_t)(currentStepMotor1 + BL_PHASE_B)]) ;
//    pwmWrite(BOARD_MOT2_CMD_C, pwmSin[(uint8_t)(currentStepMotor1 + BL_PHASE_C)]) ;
    setPwmSin(BOARD_MOT2_CMD_A, currentStepMotor1 + BL_PHASE_A, pwmSin);
	setPwmSin(BOARD_MOT2_CMD_B, currentStepMotor1 + BL_PHASE_B, pwmSin);
	setPwmSin(BOARD_MOT2_CMD_C, currentStepMotor1 + BL_PHASE_C, pwmSin);
  }

  if (motorNumber == 2)
  {
    currentStepMotor2 += dirStep;
//    currentStepMotor2 &= 0xff;
//    pwmWrite(BOARD_MOT3_CMD_A, pwmSin[currentStepMotor2 + BL_PHASE_A]) ;
//    pwmWrite(BOARD_MOT3_CMD_B, pwmSin[(uint8_t)(currentStepMotor2 + BL_PHASE_B)]) ;
//    pwmWrite(BOARD_MOT3_CMD_C, pwmSin[(uint8_t)(currentStepMotor2 + BL_PHASE_C)]) ;
    setPwmSin(BOARD_MOT3_CMD_A, currentStepMotor2 + BL_PHASE_A, pwmSin);
	setPwmSin(BOARD_MOT3_CMD_B, currentStepMotor2 + BL_PHASE_B, pwmSin);
	setPwmSin(BOARD_MOT3_CMD_C, currentStepMotor2 + BL_PHASE_C, pwmSin);
  }
}

// switch off motor power
// TODO: for some reason motor control gets noisy, if call from ISR
inline void MotorOff(uint8_t motorNumber, pwmsin_t* pwmSin)
{
	if (motorNumber == 0)
	{
		pwmWrite(BOARD_MOT1_CMD_A, pwmSin[0]);
		pwmWrite(BOARD_MOT1_CMD_B, pwmSin[0]);
		pwmWrite(BOARD_MOT1_CMD_C, pwmSin[0]);
	}

	if (motorNumber == 1)
	{
		pwmWrite(BOARD_MOT2_CMD_A, pwmSin[0]);
		pwmWrite(BOARD_MOT2_CMD_B, pwmSin[0]);
		pwmWrite(BOARD_MOT2_CMD_C, pwmSin[0]);
	}

	if (motorNumber == 2)
	{
		pwmWrite(BOARD_MOT3_CMD_A, pwmSin[0]);
		pwmWrite(BOARD_MOT3_CMD_B, pwmSin[0]);
		pwmWrite(BOARD_MOT3_CMD_C, pwmSin[0]);
	}
}


void calcSinusArray(uint8_t maxPWM, pwmsin_t *array, pwmsin_t pwmResolution)
{
  for(int i=0; i<N_SIN; i++)
  {
//    array[i] = maxPWM / 2.0 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
    //array[i] = 128 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
	  array[i] = (pwmResolution / 2) + sin(2.0 * i / N_SIN * 3.14159265) * (pwmResolution * maxPWM / 255.0) / 2.0;
  }  
}

void recalcMotorStuff()
{
  cli();
  calcSinusArray(config.maxPWMmotorPitch,pwmSinMotorPitch, motors_pwm_period);
  calcSinusArray(config.maxPWMmotorRoll,pwmSinMotorRoll, motors_pwm_period);
  calcSinusArray(config.maxPWMmotorYaw, pwmSinMotorYaw, motors_pwm_period);
  sei();
}

/********************************/
/* Motor Control IRQ Routine    */
/********************************/

void motorInitInterrupt()
{
//	//inizializzo interrupt timer
//	Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
//	Timer2.setPeriod(MOTORUPDATE_FREQ); // in microseconds
//	Timer2.setCompare1(1);      // overflow might be small
//	Timer2.attachInterrupt(1, motorInterrupt);
}

// motor position control
uint32_t _last_motor_interrupt = 0;
//ISR( TIMER1_OVF_vect )
void motorInterrupt()
{
  // 0.88us / 8.1us
//  freqCounter++;
//  if(freqCounter==(CC_FACTOR*1000/MOTORUPDATE_FREQ))
//  {
 //   freqCounter=0;

	uint32_t now = millis();

	if ((now - _last_motor_interrupt) >= 1000/MOTORUPDATE_FREQ)
	{
		_last_motor_interrupt = now;
		if (enableMotorUpdates)
		{
			// move pitch motor
			MoveMotorPosSpeed(config.motorNumberPitch, pitchMotorDrive, pwmSinMotorPitch);
			// move roll motor
			MoveMotorPosSpeed(config.motorNumberRoll, rollMotorDrive, pwmSinMotorRoll);
			// move roll yaw
			MoveMotorPosSpeed(config.motorNumberYaw, yawMotorDrive, pwmSinMotorYaw);
		}
		// update event
		motorUpdate = true;
	}
}


void motorMove(uint8_t motorNum, int steps)
{
	if (motorNum == 0)
		fastMoveMotor(0, steps, pwmSinMotorRoll);
	if (motorNum == 1)
		fastMoveMotor(1, steps, pwmSinMotorPitch); //config.motorNumberPitch
	if (motorNum == 2)
		fastMoveMotor(2, steps, pwmSinMotorYaw);
}

void motorTest()
{
  #define MOT_DEL 10
  cli();
  delay(10 * CC_FACTOR);
  // Move Motors to ensure function
  for(int i=0; i<N_SIN; i++) { fastMoveMotor(config.motorNumberPitch, 1,pwmSinMotorPitch); delay(MOT_DEL * CC_FACTOR);  }
  for(int i=0; i<N_SIN; i++) { fastMoveMotor(config.motorNumberPitch, -1,pwmSinMotorPitch); delay(MOT_DEL * CC_FACTOR);  }
  delay(200 * CC_FACTOR);
  for(int i=0; i<N_SIN; i++) { fastMoveMotor(config.motorNumberRoll, 1,pwmSinMotorRoll); delay(MOT_DEL * CC_FACTOR);  }
  for(int i=0; i<N_SIN; i++) { fastMoveMotor(config.motorNumberRoll, -1,pwmSinMotorRoll); delay(MOT_DEL * CC_FACTOR);  }
  delay(200 * CC_FACTOR);
  for(int i=0; i<N_SIN; i++) { fastMoveMotor(config.motorNumberYaw, 1,pwmSinMotorYaw); delay(MOT_DEL * CC_FACTOR);  }
  for(int i=0; i<N_SIN; i++) { fastMoveMotor(config.motorNumberYaw, -1,pwmSinMotorYaw); delay(MOT_DEL * CC_FACTOR);  }
  sei();  
}


