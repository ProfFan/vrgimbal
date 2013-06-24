
/*
Brushless Gimbal Controller Software by Christian Winkler and Alois Hahn (C) 2013

Brushless Gimbal Controller Hardware and Software support 
by Ludwig Fäerber, Alexander Rehfeld and Martin Eckart

Project homepage: http://brushlessgimbal.de/
Discussions:
http://fpv-community.de/showthread.php?20795-Brushless-Gimbal-Controller-SOFTWARE
http://fpv-community.de/showthread.php?22617-Gimbal-Brushless-Controller-V3-0-50x50mm-by-Martinez
http://fpv-community.de/showthread.php?19252-Brushless-Gimbal-Controller

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version. see <http://www.gnu.org/licenses/>

Anyhow, if you start to commercialize our work, please read on http://code.google.com/p/brushless-gimbal/ on how to contribute

// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
*/


// FOR CHANGES PLEASE READ: ReleaseHistory.txt

// Serial Programming for Settings!!!
/* HOWTO:
- edit setDefaultParameters() in variables.h if you want to.
- Upload Firmware.
- Open Arduino Terminal and enable NL in the lower right corner of the window.
- Type in HE 
-... enjoy
*/




/*************************/
/* Include Header Files  */
/*************************/

#include <main.h>
#include "EEPROMAnything.h"

FastSerial  Serial;
HardwareSPI spi2(2);
HardwareI2C i2c2(2);
SerialCommand sCmd(&Serial);     // Create SerialCommand object

#if defined ( IMU_BRUGI )
MPU6050 mpu(&i2c2);            // Create MPU object



void initMPUlpf() {
  // Set Gyro Low Pass Filter(0..6, 0=fastest, 6=slowest)
  switch (config.mpuLPF) {
    case 0:  mpu.setDLPFMode(MPU6050_DLPF_BW_256);  break;
    case 1:  mpu.setDLPFMode(MPU6050_DLPF_BW_188);  break;
    case 2:  mpu.setDLPFMode(MPU6050_DLPF_BW_98);   break;
    case 3:  mpu.setDLPFMode(MPU6050_DLPF_BW_42);   break;
    case 4:  mpu.setDLPFMode(MPU6050_DLPF_BW_20);   break;
    case 5:  mpu.setDLPFMode(MPU6050_DLPF_BW_10);   break;
    case 6:  mpu.setDLPFMode(MPU6050_DLPF_BW_5);    break;
    default: mpu.setDLPFMode(MPU6050_DLPF_BW_256);  break;
  }
}
#endif

#if defined ( IMU_AP )


#ifdef GIMBAL_ENABLE_COMPASS
AP_Compass_HMC5843      	compass(&i2c2, &Serial);
#endif

Arduino_Mega_ISR_Registry 	isr_registry;
AP_TimerProcess  scheduler;
static GPS							*g_gps_null = NULL;
#ifdef GIMBAL_IMU_SPI
AP_InertialSensor_MPU6000 	ins(  &spi1,BOARD_CS_IMU_SPI, false, BOARD_INT_IMU_SPI, &Serial);
#else
AP_InertialSensor_MPU6000_I2C	ins(&i2c2, BOARD_INT_IMU, &Serial);
#endif
AP_AHRS_DCM					ahrs(&ins, g_gps_null);





void initMPUlpf() {

}
#endif

#ifdef RC_USE_LIBS
APM_RC_MP32V3 				APM_RC;
#endif

/**********************************************/
/* Initialization                             */
/**********************************************/
void read_config()
{
	EEPROM_readAnything(0, config);
}
void write_config()
{
	EEPROM_writeAnything(0, config);
}

void dummy(void)
{}

void setup() 
{

  // just for debugging
#ifdef STACKHEAPCHECK_ENABLE
  stackCheck();
  heapCheck();
#endif

  LEDPIN_PINMODE
  LEDGREPIN_PINMODE

  LEDGREPIN_ON
  

  //CH2_PINMODE
  //CH3_PINMODE
  
  // Start Serial Port
	Serial.configure(SERIAL_CLI_PORT);
	Serial.begin(SERIAL_CLI_BAUD, 0x01, 0x07);

	Serial.print("\r\n\r\nLASER NAVIGATION SRL\r\n\r\n");
	Serial.printf("\r\nInit %s|%s %s\r\n", THISFIRMWARE, __DATE__, __TIME__);
	Serial.printf("System Clock: %lu MHz\r\n", SystemCoreClock/1000000);

  // Set Serial Protocol Commands
  setSerialProtocol();
  
  // Start I2C and Configure Frequency
  Serial.print("Init I2C\r\n");
  i2c2.begin();


	Serial.print("Init SPI2\r\n");
	spi2.begin(SPI_1_125MHZ, MSBFIRST, 0);
	EEPROM.init(&spi2, BOARD_CS_EEPROM); //, true, 9, 512, 512);

  // Read Config or fill with default settings
  if(EEPROM.read(0)==VERSION)
  {
	  read_config();
  }
  else
  {
    setDefaultParameters();
    write_config();
  }
  
  // Init Sinus Arrays and Motor Stuff 
  recalcMotorStuff();
  
  // Init PIDs to reduce floating point operations.
  initPIDs();

  // init RC variables
  initRC();
  

 
   // Init BL Controller
  //initBlController();
  // Initialize MPU 
  initResolutionDevider();
  
  // Init IMU variables
  initIMU();
  
#if defined ( IMU_BRUGI )
  // Auto detect MPU address
  mpu.setAddr(MPU6050_ADDRESS_AD0_HIGH);
  mpu.initialize();
  if(mpu.testConnection()) {
    Serial.println(F("MPU6050 ok (HIGH)"));  
  } else {
    mpu.setAddr(MPU6050_ADDRESS_AD0_LOW);
    mpu.initialize();
    if(mpu.testConnection()) {
      Serial.println(F("MPU6050 ok (LOW)"));  
    } else {
      Serial.println(F("MPU6050 failed"));
    }
  }
#endif

  //CH2_ON
  
  // set sensor orientation (from config)
  initSensorOrientation();
  
#if defined ( IMU_BRUGI )
  // Init MPU Stuff
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);          // Set Clock to ZGyro
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);           // Set Gyro Sensitivity to config.h
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);       //+- 2G
  initMPUlpf();                                         // Set Gyro Low Pass Filter
  mpu.setRate(0);                                       // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
  mpu.setSleepEnabled(false); 
  
  // Gyro Offset calibration
  Serial.println(F("Gyro calibration: do not move"));
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
  gyroOffsetCalibration();
  initMPUlpf();
  Serial.println(F("Gyro calibration: done"));
#endif
  
  LEDPIN_ON
  
   // Init BL Controller
  initBlController();
  // motorTest();

  // Init RC-Input
  initRCPins();

  LEDGREPIN_OFF
  
  Serial.println(F("GO! Type HE for help, activate NL in Arduino Terminal!"));

  //CH2_OFF
  //CH3_OFF

  //TEO 20130607
  gimState = GIM_IDLE;
  stateStart = millis();
 
}

/************************/
/* PID Controller       */
/************************/
int32_t ComputePID(int32_t DTms, int32_t in, int32_t setPoint, int32_t *errorSum, int32_t *errorOld, int32_t Kp, int16_t Ki, int32_t Kd)
{
  int32_t error = setPoint - in;
  int32_t Ierr;
   
  Ierr = error * Ki * DTms;
  Ierr = constrain_int32(Ierr, -(int32_t)1000*100, (int32_t)1000*100);
  *errorSum += Ierr;
 
  /*Compute PID Output*/
  int32_t out = (Kp * error) + *errorSum + Kd * (error - *errorOld) * DTms;
  *errorOld = error;

  return out / 4096;
}



/**********************************************/
/* Main Loop                                  */
/**********************************************/

//test RC
void loop_testrc()
{
	static uint32_t output_time = 0;

  if (motorUpdate) // loop runs with motor ISR update rate (1000Hz)
  {
	motorUpdate = false;
	// Evaluate RC-Signals
	if(config.rcAbsolute==1) {
	  evaluateRCSignalAbsolute();  // gives rollRCSetPoint, pitchRCSetpoint
	  utilLP_float(&pitchAngleSet, PitchPhiSet, rcLPF_tc);
	  utilLP_float(&rollAngleSet, RollPhiSet, rcLPF_tc);
	  utilLP_float(&yawAngleSet, YawPhiSet, rcLPF_tc);
	} else {
	  evaluateRCSignalProportional(); // gives rollRCSpeed, pitchRCSpeed
	  utilLP_float(&pitchAngleSet, PitchPhiSet, 0.01);
	  utilLP_float(&rollAngleSet, RollPhiSet, 0.01);
	  utilLP_float(&yawAngleSet, YawPhiSet, 0.01);
	}


    uint32_t now = millis();

    if (now - output_time > (1000 / POUT_FREQ))
	{
		output_time = now;

		//if (g_bSendRCOutput)
		{
			Serial.print(now); Serial.print("\t");
			for (int i = 0; i < RC_PPM_RX_MAX_CHANNELS; i++)
			{
				Serial.print(rcRxChannel[i]); Serial.print("\t");
			}

			//Serial.print(rollAngleSet); Serial.print("\t");
			//Serial.print(pitchAngleSet); Serial.print("\t");
			//Serial.print(yawAngleSet); Serial.print("\t");

			Serial.println();
		}
	}

    //****************************
    // Evaluate Serial inputs
    //****************************
    sCmd.readSerial();
  }
}

uint32 superfast_loopTimer = 0;
uint32 fast_loopTimer = 0;
uint32 superfastloop_speed = 1; // SUPER_FAST_MICROSECONDS;
uint32 fastloop_speed = 5; //FAST_MICROSECONDS;
void loop()
{    
  int32_t pitchPIDVal;
  int32_t rollPIDVal;
  int32_t yawPIDVal;
  static int32_t pitchErrorSum;
  static int32_t rollErrorSum;
  static int32_t yawErrorSum;
  static int32_t pitchErrorOld;
  static int32_t rollErrorOld;
  static int32_t yawErrorOld;
  
  static char pOutCnt = 0;
  static uint32_t output_time = 0;

  //static int stateCount = 0;
  
  motorInterrupt();

#ifdef IMU_AP

	//uint32_t micr = micros();
  	  uint32_t micr = millis();

	if (micr - superfast_loopTimer >= superfastloop_speed) // 250 MPU6000 500 VRIMU
	{
		superfast_loopTimer = micr;
		//insert here all routines called by timer_scheduler
		//superfast_loop();
		ins.read();
	}

//	uint32_t timer	= micros();
	uint32_t timer	= millis();
	// We want this to execute fast
	// ----------------------------
	if (((timer - fast_loopTimer) >= fastloop_speed) && (ins.num_samples_available() >= 1))
	{
		fast_loopTimer = timer;
		//fast_loop();
		#ifdef GIMBAL_ENABLE_COMPASS
			compass.read();
		#endif

			ahrs.update();
	}
#endif



  if (motorUpdate) // loop runs with motor ISR update rate (1000Hz)
  {
    motorUpdate = false;
    
//    CH2_ON
   
    // update IMU data            
    readGyros();
    
    if (config.enableGyro) updateGyroAttitude();
    if (config.enableACC) updateACCAttitude(); 

    getAttiduteAngles();
    
    // Evaluate RC-Signals
    if(config.rcAbsolute==1) {
      evaluateRCSignalAbsolute();  // gives rollRCSetPoint, pitchRCSetpoint
      utilLP_float(&pitchAngleSet, PitchPhiSet, rcLPF_tc);
      utilLP_float(&rollAngleSet, RollPhiSet, rcLPF_tc);
      utilLP_float(&yawAngleSet, YawPhiSet, rcLPF_tc);
    } else {
      evaluateRCSignalProportional(); // gives rollRCSpeed, pitchRCSpeed
      utilLP_float(&pitchAngleSet, PitchPhiSet, 0.01);
      utilLP_float(&rollAngleSet, RollPhiSet, 0.01);
      utilLP_float(&yawAngleSet, YawPhiSet, 0.01);
    }
       
    //****************************
    // pitch PID
    //****************************
    if (config.modePitch == 0)
    {
		pitchPIDVal = ComputePID(DT_INT_MS, angle[axisPITCH], pitchAngleSet, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd);
		// motor control
		pitchMotorDrive = pitchPIDVal * config.dirMotorPitch;
    } else {
    	pitchMotorDrive = (int) ((float) config.stepsMotorPitch * ((float)(pitchAngleSet - config.angleOffsetPitch) / ANGLE_PRECISION) / 360.0);
    }
    //****************************
    // roll PID
    //****************************
    if (config.modePitch == 0)
    {
		rollPIDVal = ComputePID(DT_INT_MS, angle[axisROLL], rollAngleSet, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd);

		// motor control
		rollMotorDrive = rollPIDVal * config.dirMotorRoll;
	} else {
		rollMotorDrive = (int) ((float) config.stepsMotorRoll * ((float)(rollAngleSet - config.angleOffsetRoll) / ANGLE_PRECISION) / 360.0);
	}

    //****************************
    // yaw PID
    //****************************
    if (config.modeYaw == 0)
    {
		yawPIDVal = ComputePID(DT_INT_MS, angle[axisYAW], yawAngleSet, &yawErrorSum, &yawErrorOld, yawPIDpar.Kp, yawPIDpar.Ki, yawPIDpar.Kd);

		// motor control
		yawMotorDrive = yawPIDVal * config.dirMotorYaw;
	} else {
		yawMotorDrive = (int) ((float) config.stepsMotorYaw * ((float)(yawAngleSet - config.angleOffsetYaw) / ANGLE_PRECISION) / 360.0);
	}

    //****************************
    // slow rate actions
    //****************************
    uint32_t now = millis();

    if (now - output_time > (1000 / POUT_FREQ))
	{
		output_time = now;
		// 600 us
		if(config.accOutput==1){ Serial.print(angle[axisPITCH]);
									Serial.print(F(" ACC "));Serial.print(angle[axisROLL]);
									Serial.print(F(" "));Serial.println(angle[axisYAW]);}

		if (g_bSendDebugOutput){
			Serial.print(F(" R "));Serial.print(angle[axisROLL]); Serial.print(F(" m "));Serial.print(rollMotorDrive);
			Serial.print(F(" P "));Serial.print(angle[axisPITCH]); Serial.print(F(" m "));Serial.print(pitchMotorDrive);
			Serial.print(F(" Y "));Serial.print(angle[axisYAW]); Serial.print(F(" m "));Serial.print(yawMotorDrive);
			Serial.println();
		}

		if (g_bSendRCOutput)
		{
			Serial.print(now); Serial.print("\t");
			for (int i = 0; i < RC_PPM_RX_MAX_CHANNELS; i++)
			{
				Serial.print(rcRxChannel[i]); Serial.print("\t");
			}

			Serial.print(rollAngleSet); Serial.print("\t");
			Serial.print(pitchAngleSet); Serial.print("\t");
			Serial.print(yawAngleSet); Serial.print("\t");

			Serial.println();
		}

	}



    switch (count) {
    case 1:
      readACC(axisROLL); break;
    case 2:
      readACC(axisPITCH); break;
    case 3:
      readACC(axisYAW); break;
    case 4:
      updateACC(); break;
    case 5:
      break;
    case 6:
      // gimbal state transitions 
      switch (gimState)
      {
        case GIM_IDLE :
          // wait 2 sec to settle ACC, before PID controlerbecomes active 
          if (now - stateStart >= IDLE_TIME_SEC)
          {
            gimState = GIM_UNLOCKED;
            stateStart = now;
          }
          break;
        case GIM_UNLOCKED :
          // allow PID controller to settle on ACC position
          if (now - stateStart >= LOCK_TIME_SEC)
          {
            gimState = GIM_LOCKED;
            stateStart = now;
          }
          break;
        case GIM_LOCKED :
          // normal operation
          break;
      }
      // gimbal state actions 
      switch (gimState) {
        case GIM_IDLE :
          enableMotorUpdates = false;
          LEDGREPIN_OFF
          setACCFastMode(true);
          break;
        case GIM_UNLOCKED :
          enableMotorUpdates = true;
          LEDGREPIN_ON
          setACCFastMode(true);
          break;
        case GIM_LOCKED :
          enableMotorUpdates = true;
          LEDGREPIN_ON
          setACCFastMode(false);
          break;
      }
      break;
    case 7:
      // RC Pitch function
      if (validRC[config.rcChannelPitch]) {
        if(config.rcAbsolute==1) {
            PitchPhiSet = pitchRCSetpoint*ANGLE_PRECISION;
        }
        else {
          if(abs(pitchRCSpeed)>0.01) {
            PitchPhiSet += pitchRCSpeed;
          }
        }
      } else {
        PitchPhiSet = 0;
      }
      PitchPhiSet = constrain(PitchPhiSet, config.minRCPitch*ANGLE_PRECISION, config.maxRCPitch*ANGLE_PRECISION);
      break;
    case 8:
      // RC roll function
      if (validRC[config.rcChannelRoll]){
        if(config.rcAbsolute==1){
          RollPhiSet = rollRCSetpoint*ANGLE_PRECISION;
        } else {
          if(abs(rollRCSpeed)>0.01) {
            RollPhiSet += rollRCSpeed;
          }
        }
      } else {
        RollPhiSet = 0;
      }
      RollPhiSet = constrain(RollPhiSet, config.minRCRoll*ANGLE_PRECISION, config.maxRCRoll*ANGLE_PRECISION);
      break;
    case 9:
    	// RC yaw function
	  if (validRC[config.rcChannelYaw]){
		if(config.rcAbsolute==1){
		  YawPhiSet = yawRCSetpoint*ANGLE_PRECISION;
		} else {
		  if(abs(yawRCSpeed)>0.01) {
			YawPhiSet += yawRCSpeed;
		  }
		}
	  } else {
		YawPhiSet = 0;
	  }
	  YawPhiSet = constrain(YawPhiSet, config.minRCYaw*ANGLE_PRECISION, config.maxRCYaw*ANGLE_PRECISION);
	  break;

//      // regular ACC output
//      pOutCnt++;
//      if (pOutCnt == (LOOPUPDATE_FREQ/10/POUT_FREQ))
//      {
//    	// 600 us
//        if(config.accOutput==1){ Serial.print(angle[axisPITCH]); Serial.print(F(" ACC "));Serial.println(angle[axisROLL]);}
//
//        if (g_bSendDebugOutput){
//        	Serial.print(F(" R "));Serial.print(angle[axisROLL]); Serial.print(F(" m "));Serial.print(rollMotorDrive);
//        	Serial.print(F(" P "));Serial.print(angle[axisPITCH]); Serial.print(F(" m "));Serial.print(pitchMotorDrive);
//        	Serial.println();
//        }
//        pOutCnt = 0;
//      }
      break;
    case 10:    
#ifdef STACKHEAPCHECK_ENABLE
      stackHeapEval(false);
#endif
      count=0;
      break;
    default:
      break;
    }
    count++;
       
    //****************************
    // check RC channel timeouts
    //****************************
    if (config.rcModePPM)
    {
      checkPPMTimeout();
    }
    else
    {
    	for (int i = 0; i < RC_PWM_CHANNELS; i++)
    		checkPWMTimeout(i);
    }

    //****************************
    // Evaluate Serial inputs 
    //****************************
    sCmd.readSerial();

//    CH2_OFF
  }

}


