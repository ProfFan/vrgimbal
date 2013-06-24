/*************************/
/* Config Structure      */
/*************************/

#include "main.h"
#include "variables.h"


config_t config;
PIDdata_t pitchPIDpar;
PIDdata_t rollPIDpar;
PIDdata_t yawPIDpar;

void setDefaultParameters()
{
  config.vers = VERSION;
  config.modePitch = 0; //0 = PID, 1 = ABSOLUTE
  config.modeRoll = 0; //0 = PID, 1 = ABSOLUTE
  config.modeYaw = 1; //0 = PID, 1 = ABSOLUTE
  config.gyroPitchKp = 20000;
  config.gyroPitchKi = 25000;
  config.gyroPitchKd = 40000;
  config.gyroRollKp = 20000;
  config.gyroRollKi = 25000;
  config.gyroRollKd = 30000;
  config.gyroYawKp = 0;
  config.gyroYawKi = 0;
  config.gyroYawKd = 0;
  config.accTimeConstant = 7;
  config.mpuLPF = 0;
  config.angleOffsetPitch = 0;
  config.angleOffsetRoll = 0;
  config.angleOffsetYaw = 0;
  config.nPolesMotorPitch = 14;
  config.nPolesMotorRoll = 14;
  config.nPolesMotorYaw = 14;
  config.dirMotorPitch = 1;
  config.dirMotorRoll = -1;
  config.dirMotorYaw = 1;
  config.stepsMotorPitch = 8000;  //steps for 360°
  config.stepsMotorRoll = 8000;//steps for 360°
  config.stepsMotorYaw = 8000;//steps for 360°
  config.motorNumberPitch = 1; //0;
  config.motorNumberRoll = 0; //1;
  config.motorNumberYaw = 2; //1;
  config.maxPWMmotorPitch = 80;
  config.maxPWMmotorRoll = 80;
  config.maxPWMmotorYaw = 127;
  config.minRCPitch = -30;
  config.maxRCPitch = 30;
  config.minRCRoll = -30;
  config.maxRCRoll = 30;
  config.minRCYaw = -30;
  config.maxRCYaw = 30;
  config.rcGain = 5;
  config.rcLPF = 20;              // 2 sec
  config.rcModePPM = false;
  config.rcChannelRoll = 0;
  config.rcChannelPitch = 1;
  config.rcChannelYaw = 3;
  config.rcMid = 1500;
  config.rcAbsolute = true;
  config.accOutput=false;
  config.enableGyro=true;
  config.enableACC=true;
  config.axisReverseZ=true;
  config.axisSwapXY=false;
}


void initPIDs(void)
{
  rollPIDpar.Kp = config.gyroRollKp;
  rollPIDpar.Ki = config.gyroRollKi/1000;
  rollPIDpar.Kd = config.gyroRollKd;

  pitchPIDpar.Kp = config.gyroPitchKp;
  pitchPIDpar.Ki = config.gyroPitchKi/1000;
  pitchPIDpar.Kd = config.gyroPitchKd;

  yawPIDpar.Kp = config.gyroYawKp;
  yawPIDpar.Ki = config.gyroYawKi/1000;
  yawPIDpar.Kd = config.gyroYawKd;
}



/*************************/
/* Variables             */
/*************************/



// motor drive

pwmsin_t pwmSinMotorPitch[N_SIN];
pwmsin_t pwmSinMotorRoll[N_SIN];
pwmsin_t pwmSinMotorYaw[N_SIN];

int currentStepMotor0 = 0;
int currentStepMotor1 = 0;
int currentStepMotor2 = 0;
bool motorUpdate = false;

int8_t pitchDirection = 1;
int8_t rollDirection = 1;
int8_t yawDirection = 1;

int freqCounter=0; // TODO: back to char later ...

int pitchMotorDrive = 0;
int rollMotorDrive = 0;
int yawMotorDrive = 0;

// control motor update in ISR
bool enableMotorUpdates = false;


// Variables for MPU6050
float gyroPitch;
float gyroRoll; //in deg/s
float gyroYaw; //in deg/s

float resolutionDevider;
int16_t x_val;
int16_t y_val;
int16_t z_val;

float PitchPhiSet = 0;
float RollPhiSet = 0;
float YawPhiSet = 0;
float pitchAngleSet=0;
float rollAngleSet=0;
float yawAngleSet=0;

int count=0;

// RC single channel PWM decoder
int32_t microsRisingEdge[RC_PWM_CHANNELS] = {0,};
int32_t microsLastPWMUpdate[RC_PWM_CHANNELS] = {0,};

// RC PPM decoder
uint16_t rcRxChannel[RC_PPM_RX_MAX_CHANNELS] = {0,};
bool updateRC[RC_PPM_RX_MAX_CHANNELS] = {false,};      // RC channel value got updated
bool validRC[RC_PPM_RX_MAX_CHANNELS] = {false, };    // RC inputs valid

int32_t microsLastPPMupdate = 0;
bool rxPPMvalid = false;


// RC control
float pitchRCSpeed=0.0;
float rollRCSpeed=0.0;
float yawRCSpeed=0.0;
float pitchRCSetpoint = 0.0;
float rollRCSetpoint = 0.0;
float yawRCSetpoint = 0.0;
float rcLPF_tc = 1.0;

// Gimbal State
gimStateType gimState = GIM_IDLE;
//int stateCount = 0;
uint32_t stateStart = 0;


//*************************************
//
//  IMU
//
//*************************************
flags_struct flags;


//********************
// sensor orientation
//********************

t_sensorOrientationDef sensorDef = {
    {{0, 1}, {1, 1}, {2, 1}},    // Gyro
    {{0, 1}, {1, 1}, {2, 1}}     // Acc
  };

// gyro calibration value
int16_t gyroOffset[3] = {0, 0, 0};



float gyroScale=0;

int32_t accSmooth[3];
int16_t gyroADC[3];
int16_t accADC[3];

t_fp_vector EstG;

float accLPF[3];
int32_t accMag = 0;

float AccComplFilterConst = 0;  // filter constant for complementary filter

int16_t acc_25deg = 25;      //** TODO: check

int32_t angle[3]    = {0,0, 0};  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

// DEBUG only
uint32_t stackTop = 0xffffffff;
uint32_t stackBottom = 0;

uint32_t heapTop = 0;
uint32_t heapBottom = 0xffffffff;


bool g_bSendDebugOutput = false;
bool g_bSendRCOutput = false;

