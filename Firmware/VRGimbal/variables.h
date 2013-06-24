/*************************/
/* Config Structure      */
/*************************/
#ifndef _VARIABLES_H_
#define _VARIABLES_H_

struct config_t
{
uint8_t vers;
int8_t modePitch; //0 = PID, 1 = ABSOLUTE
int8_t modeRoll; //0 = PID, 1 = ABSOLUTE
int8_t modeYaw; //0 = PID, 1 = ABSOLUTE
int32_t gyroPitchKp; 
int32_t gyroPitchKi;   
int32_t gyroPitchKd;
int32_t gyroRollKp;
int32_t gyroRollKi;
int32_t gyroRollKd;
int32_t gyroYawKp;
int32_t gyroYawKi;
int32_t gyroYawKd;
int16_t accTimeConstant;
int8_t  mpuLPF;             // mpu LPF 0..6, 0=fastest(256Hz) 6=slowest(5Hz)
int16_t angleOffsetPitch;   // angle offset, deg*100
int16_t angleOffsetRoll;
int16_t angleOffsetYaw;
uint8_t nPolesMotorPitch;
uint8_t nPolesMotorRoll;
uint8_t nPolesMotorYaw;
int8_t dirMotorPitch;
int8_t dirMotorRoll;
int8_t dirMotorYaw;
int32_t stepsMotorPitch;  //steps for 360°
int32_t stepsMotorRoll;//steps for 360°
int32_t stepsMotorYaw;//steps for 360°
uint8_t motorNumberPitch;
uint8_t motorNumberRoll;
uint8_t motorNumberYaw;
uint8_t maxPWMmotorPitch;
uint8_t maxPWMmotorRoll;
uint8_t maxPWMmotorYaw;
int8_t minRCPitch;
int8_t maxRCPitch;
int8_t minRCRoll;
int8_t maxRCRoll;
int8_t minRCYaw;
int8_t maxRCYaw;
int16_t rcGain;
int16_t rcLPF;             // low pass filter for RC absolute mode, units=1/10 sec
bool rcModePPM;            // RC mode, true=common RC PPM channel, false=separate RC channels 
int8_t rcChannelPitch;     // input channel for pitch
int8_t rcChannelRoll;      // input channel for roll
int8_t rcChannelYaw;
int16_t rcMid;             // rc channel center ms
bool rcAbsolute;
bool accOutput;
bool enableGyro;           // enable gyro attitude update
bool enableACC;            // enable acc attitude update
bool axisReverseZ;
bool axisSwapXY;
};

extern config_t config;

void recalcMotorStuff();
void initPIDs();

void setDefaultParameters();


typedef struct PIDdata {
  int32_t   Kp, Ki, Kd;
} PIDdata_t;

extern PIDdata_t pitchPIDpar;
extern PIDdata_t rollPIDpar;
extern PIDdata_t yawPIDpar;

void initPIDs(void);


/*************************/
/* Variables             */
/*************************/



// motor drive

extern pwmsin_t pwmSinMotorPitch[N_SIN];
extern pwmsin_t pwmSinMotorRoll[N_SIN];
extern pwmsin_t pwmSinMotorYaw[N_SIN];

extern int currentStepMotor0;
extern int currentStepMotor1;
extern int currentStepMotor2;
extern bool motorUpdate;

extern int8_t pitchDirection;
extern int8_t rollDirection;
extern int8_t yawDirection;

extern int freqCounter; // TODO: back to char later ...

extern int pitchMotorDrive;
extern int rollMotorDrive;
extern int yawMotorDrive;

// control motor update in ISR
extern bool enableMotorUpdates;


// Variables for MPU6050
extern float gyroPitch;
extern float gyroRoll; //in deg/s
extern float gyroYaw; //in deg/s

extern float resolutionDevider;
extern int16_t x_val;
extern int16_t y_val;
extern int16_t z_val;

extern float PitchPhiSet;
extern float RollPhiSet;
extern float YawPhiSet;
extern float pitchAngleSet;
extern float rollAngleSet;
extern float yawAngleSet;

extern int count;

// RC single channel PWM decoder
extern int32_t microsRisingEdge[RC_PWM_CHANNELS];
extern int32_t microsLastPWMUpdate[RC_PWM_CHANNELS];

// RC PPM decoder
extern uint16_t rcRxChannel[RC_PPM_RX_MAX_CHANNELS];
extern bool updateRC[RC_PPM_RX_MAX_CHANNELS];      // RC channel value got updated
extern bool validRC[RC_PPM_RX_MAX_CHANNELS];    // RC inputs valid

extern int32_t microsLastPPMupdate;
extern bool rxPPMvalid;


// RC control
extern float pitchRCSpeed;
extern float rollRCSpeed;
extern float yawRCSpeed;
extern float pitchRCSetpoint;
extern float rollRCSetpoint;
extern float yawRCSetpoint;
extern float rcLPF_tc;

// Gimbal State
enum gimStateType {
 GIM_IDLE=0,      // no PID
 GIM_UNLOCKED,    // PID on, fast ACC
 GIM_LOCKED       // PID on, slow ACC
};

extern gimStateType gimState;
//extern int stateCount;
extern uint32_t stateStart;


//*************************************
//
//  IMU
//
//*************************************
struct flags_struct {
  uint8_t SMALL_ANGLES_25 : 1;
};

extern flags_struct flags;

enum axisDef {
	axisROLL,
	axisPITCH,
	axisYAW
};

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;



//********************
// sensor orientation
//********************
typedef struct sensorAxisDef {
  char idx;
  int  dir;
} t_sensorAxisDef;

typedef struct sensorOrientationDef {
  t_sensorAxisDef Gyro[3];
  t_sensorAxisDef Acc[3];
} t_sensorOrientationDef;

extern t_sensorOrientationDef sensorDef;

// gyro calibration value
extern int16_t gyroOffset[3];



extern float gyroScale;

extern int32_t accSmooth[3];
extern int16_t gyroADC[3];
extern int16_t accADC[3];

extern t_fp_vector EstG;

extern float accLPF[3];
extern int32_t accMag;

extern float AccComplFilterConst;  // filter constant for complementary filter

extern int16_t acc_25deg;      //** TODO: check

extern int32_t angle[3];  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

// DEBUG only
extern uint32_t stackTop;
extern uint32_t stackBottom;

extern uint32_t heapTop;
extern uint32_t heapBottom;


extern bool g_bSendDebugOutput;
extern bool g_bSendRCOutput;

#endif //_VARIABLES_H_
