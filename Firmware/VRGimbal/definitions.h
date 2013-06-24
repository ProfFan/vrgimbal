/*************************/
/* Definitions           */
/*************************/
#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_


//inserisco qui le definzioni per mantenere la compatibilità
#define PGM_P const void *
#define F(x) x
#define cli dummy //noInterrupts
#define sei dummy //interrupts





#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION 51

#define ANGLE_PRECISION 1000  //millesimi di grado



// MPU Address Settings
#define MPU6050_ADDRESS_AD0_LOW     0x68 // default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // Drotek MPU breakout board
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_HIGH


// Define Brushless PWM Mode, uncomment ONE setting
#define PWM_32KHZ_PHASE  // Resolution 8 bit for PWM
//#define PWM_8KHZ_FAST    // Resolution 8 bit for PWM
//#define PWM_4KHZ_PHASE   // Resolution 8 bit for PWM
//#define NO_PWM_LOOP

typedef uint16_t pwmsin_t; //uint8_t


#define MOTORUPDATE_FREQ 1000 //500                // in Hz, 1000 is default // 1,2,4,8 for 32kHz, 1,2,4 for 4kHz
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ    // loop control sample rate equals motor update rate
#define DT_FLOAT (1.0/LOOPUPDATE_FREQ)      // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ) 

#define POUT_FREQ 25     // rate of ACC print output in Hz, 25 Hz is default


#define IDLE_TIME_SEC 2  // gimbal fast lock time at startup
#define LOCK_TIME_SEC 5  // gimbal fast lock time at startup 

// LP filter coefficient
#define LOWPASS_K_FLOAT(TAU) (DT_FLOAT/(TAU+DT_FLOAT))

// Do not change for now
#define MPU6050_GYRO_FS MPU6050_GYRO_FS_250  // +-250,500,1000,2000 deg/s
#define MPU6050_DLPF_BW MPU6050_DLPF_BW_256  // 5,10,20,42,98,188,256 Hz

// Number of sinus values for full 360 deg.
// NOW FIXED TO 256 !!!
// Reason: Fast Motor Routine using uint8_t overflow for stepping
#define N_SIN 256

#define SCALE_ACC 10000.0
#define SCALE_PID_PARAMS 1000.0f

// RC Pins
//#define RC_PIN_ROLL  BOARD_PWM_IN0
//#define RC_PIN_PITCH BOARD_PWM_IN1
//#define RC_PIN_YAW   BOARD_PWM_IN2
//#define RC_PIN_MODE  BOARD_PWM_IN3


#define RC_PWM_CHANNELS 4
#define MIN_RC 1000
#define MAX_RC 2000
#define RC_DEADBAND 50
#define RC_TIMEOUT 100000

// PPM Decoder
#define RC_PPM_GUARD_TIME 4000
#define RC_PPM_RX_MAX_CHANNELS 8

// I2C Frequency
//#define I2C_SPEED 100000L     //100kHz normal mode
//#define I2C_SPEED 400000L   //400kHz fast mode
#define I2C_SPEED 800000L   //800kHz ultra fast mode

// Hardware Abstraction for Motor connectors,
// DO NOT CHANGE UNLES YOU KNOW WHAT YOU ARE DOING !!!
#define PWM_A_MOTOR1 OCR2A
#define PWM_B_MOTOR1 OCR1B
#define PWM_C_MOTOR1 OCR1A

#define PWM_A_MOTOR0 OCR0A
#define PWM_B_MOTOR0 OCR0B
#define PWM_C_MOTOR0 OCR2B


#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif


#ifdef PWM_32KHZ_PHASE
  #define CC_FACTOR 1 //32
#endif
#ifdef PWM_4KHZ_PHASE
  #define CC_FACTOR 1 //4
#endif
#ifdef PWM_8KHZ_FAST
  #define CC_FACTOR 1 //8
#endif
#ifdef NO_PWM_LOOP
  #define CC_FACTOR 1
#endif


#define LEDPIN_PINMODE             pinMode (BOARD_LED_RED_PIN, OUTPUT);
//#define LEDPIN_SWITCH              digitalWrite(BOARD_LED_RED_PIN,!bitRead(PORTB,0));
#define LEDPIN_OFF                 digitalWrite(BOARD_LED_RED_PIN, LOW);
#define LEDPIN_ON                  digitalWrite(BOARD_LED_RED_PIN, HIGH);

#define LEDGREPIN_PINMODE             pinMode (BOARD_LED_GRE_PIN, OUTPUT);
//#define LEDPIN_SWITCH              digitalWrite(BOARD_LED_GRE_PIN,!bitRead(PORTB,0));
#define LEDGREPIN_OFF                 digitalWrite(BOARD_LED_GRE_PIN, LOW);
#define LEDGREPIN_ON                  digitalWrite(BOARD_LED_GRE_PIN, HIGH);
/*
// note: execution time for CH2_ON/CH2_OFF = 4 us
#define CH2_PINMODE                pinMode (BOARD_PWM_IN2, OUTPUT);
#define CH2_OFF                    digitalWrite(BOARD_PWM_IN2, LOW);
#define CH2_ON                     digitalWrite(BOARD_PWM_IN2, HIGH);

#define CH3_PINMODE                pinMode (BOARD_PWM_IN3, OUTPUT);
#define CH3_OFF                    digitalWrite(BOARD_PWM_IN3, LOW);
#define CH3_ON                     digitalWrite(BOARD_PWM_IN3, HIGH);
*/

// enable stack and heapsize check (use just for debugging)
//#define STACKHEAPCHECK_ENABLE

#endif //_DEFINITIONS_H_
