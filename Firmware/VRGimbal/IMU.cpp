// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#include "main.h"

#if defined ( IMU_BRUGI )

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 40
#endif

#define ACC_1G 16384.0f

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f



// set default sensor orientation (sensor upside)
void initSensorOrientationDefault() {
  
  // channel assignment
  sensorDef.Gyro[axisROLL].idx = 0;
  sensorDef.Gyro[axisPITCH].idx = 1;
  sensorDef.Gyro[axisYAW].idx = 2;

  sensorDef.Acc[axisROLL].idx = 1;     // y
  sensorDef.Acc[axisPITCH].idx = 0;    // x
  sensorDef.Acc[axisYAW].idx = 2;      // z

  // direction
  sensorDef.Gyro[axisROLL].dir = 1;
  sensorDef.Gyro[axisPITCH].dir = 1;
  sensorDef.Gyro[axisYAW].dir = 1;

  sensorDef.Acc[axisROLL].dir = 1;
  sensorDef.Acc[axisPITCH].dir = 1;
  sensorDef.Acc[axisYAW].dir = 1;
  
}

// swap two char items
void swap_char(char * a, char * b) {
  char tmp = *a;
  *a = *b;
  *b = tmp;
}
// swap two int items
void swap_int(int * a, int * b) {
  int tmp = *a;
  *a = *b;
  *b = tmp;
}

// set sensor orientation according config
//
//   config.axisReverseZ
//        false ... sensor mounted on top
//        true  ... sensor mounted upside down
//   config.axisSwapXY
//        false ... default XY axes
//        true  ... swap XY (means exchange Roll/Pitch)

void initSensorOrientation() {
  
  initSensorOrientationDefault();
  
  if (config.axisReverseZ) {
    // flip over roll
    sensorDef.Acc[axisYAW].dir *= -1;
    sensorDef.Acc[axisROLL].dir *= -1;
    sensorDef.Gyro[axisPITCH].dir *= -1;
    sensorDef.Gyro[axisYAW].dir *= -1;
  }
  if (config.axisSwapXY) {
    // swap gyro axis
    swap_char(&sensorDef.Gyro[axisROLL].idx, &sensorDef.Gyro[axisPITCH].idx);
    swap_int(&sensorDef.Gyro[axisROLL].dir, &sensorDef.Gyro[axisPITCH].dir); sensorDef.Gyro[axisPITCH].dir *= -1;   // try and error ;-)
    // swap acc axis
    swap_char(&sensorDef.Acc[axisROLL].idx, &sensorDef.Acc[axisPITCH].idx);
    swap_int(&sensorDef.Acc[axisROLL].dir, &sensorDef.Acc[axisPITCH].dir); sensorDef.Acc[axisROLL].dir *= -1;
  }
}

void setACCFastMode (bool fastMode) {
  if (fastMode) {
    AccComplFilterConst = (float)DT_FLOAT/(2.0 + DT_FLOAT); // 2 sec
  } else {
    AccComplFilterConst = (float)DT_FLOAT/(config.accTimeConstant + DT_FLOAT);
  }
}

void initIMU() {
 
  // resolutionDevider=131, scale = 0.000133
  // 102us
  gyroScale =  1.0 / resolutionDevider / 180.0 * 3.14159265359;  // convert to radians
  
  setACCFastMode(false);
 
  // initialize coordinate system in EstG
  EstG.V.X = 0;
  EstG.V.Y = 0;
  EstG.V.Z = ACC_1G;

}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
// needs angle in radian units !
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[axisROLL]  * v_tmp.X + delta[axisPITCH] * v_tmp.Y;
  v->X += delta[axisROLL]  * v_tmp.Z - delta[axisYAW]   * v_tmp.Y;
  v->Y += delta[axisPITCH] * v_tmp.Z + delta[axisYAW]   * v_tmp.X;
}

void readGyros() {
  int16_t axisRot[3];
  char idx;
  // 414 us

  // read gyros
  mpu.getRotation(&axisRot[0], &axisRot[1], &axisRot[2]);
  idx = sensorDef.Gyro[0].idx;
  gyroADC[axisROLL] = axisRot[idx]-gyroOffset[idx];
  gyroADC[axisROLL] *= sensorDef.Gyro[0].dir;

  idx = sensorDef.Gyro[1].idx;
  gyroADC[axisPITCH] = -(axisRot[idx]-gyroOffset[idx]);
  gyroADC[axisPITCH] *= sensorDef.Gyro[1].dir;

  idx = sensorDef.Gyro[2].idx;
  gyroADC[axisYAW] = axisRot[idx]-gyroOffset[idx];
  gyroADC[axisYAW] *= sensorDef.Gyro[2].dir;
  
}

void readACC(axisDef axis) {
  // get acceleration
  // 382 us
  char idx;
  int16_t val;
  idx = sensorDef.Acc[axis].idx;
  val = mpu.getAccelerationN(idx);  // TODO: 370us 
  val *= sensorDef.Acc[axis].dir;
  accADC[axis] = val;
}

void updateGyroAttitude(){
  uint8_t axis;
  
  float deltaGyroAngle[3];

  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * gyroScale * DT_FLOAT;
  }
  // 168 us
  rotateV(&EstG.V,deltaGyroAngle);
}

void updateACC(){
  uint8_t axis;

  // 179 us
  for (axis = 0; axis < 3; axis++) {
    accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
    accSmooth[axis] = accLPF[axis];
    accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;
  }

  //  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G); 
  // 130 us
  // split operation to avoid 32-bit overflow, TODO: no division may happen !!!
  accMag = accMag/(int32_t)ACC_1G;
  accMag = accMag*100;
  accMag = accMag/(int32_t)ACC_1G;

  // 11 us
  if ( abs(accSmooth[axisROLL])<acc_25deg && abs(accSmooth[axisPITCH])<acc_25deg && accSmooth[axisYAW]>0) {
    flags.SMALL_ANGLES_25 = 1;
  } else {
    flags.SMALL_ANGLES_25 = 0;
  }

}


void updateACCAttitude(){
  uint8_t axis;
  
  // 255 us
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ( ( 36 < accMag && accMag < 196 ) || flags.SMALL_ANGLES_25 ) {
    for (axis = 0; axis < 3; axis++) {
      int32_t acc = accSmooth[axis];
      EstG.A[axis] = EstG.A[axis] * (1.0 - AccComplFilterConst) + acc * AccComplFilterConst; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
    } 
  }
}

/*
void getAttiduteAngles() {

  // attitude of the estimated vector  

  // 200 us
  angle[axisROLL]  = config.angleOffsetRoll +  _atan2(EstG.V.X , sqrt(EstG.V.Z*EstG.V.Z+EstG.V.Y*EstG.V.Y));
 
  // 192 us
   angle[axisPITCH] = config.angleOffsetPitch + Rajan_FastArcTan2_deg100(EstG.V.Y , EstG.V.Z);
}*/

void getAttiduteAngles() {

  // attitude of the estimated vector

  // 272 us
  angle[axisROLL]  = config.angleOffsetRoll +  Rajan_FastArcTan2_deg1000(EstG.V.X , sqrt(EstG.V.Z*EstG.V.Z+EstG.V.Y*EstG.V.Y));

  // 192 us
  angle[axisPITCH] = config.angleOffsetPitch + Rajan_FastArcTan2_deg1000(EstG.V.Y , EstG.V.Z);
}


#elif defined( IMU_AP )

// set default sensor orientation (sensor upside)
void initSensorOrientationDefault() {

  // channel assignment
  sensorDef.Gyro[axisROLL].idx = 0;
  sensorDef.Gyro[axisPITCH].idx = 1;
  sensorDef.Gyro[axisYAW].idx = 2;

  sensorDef.Acc[axisROLL].idx = 1;     // y
  sensorDef.Acc[axisPITCH].idx = 0;    // x
  sensorDef.Acc[axisYAW].idx = 2;      // z

  // direction
  sensorDef.Gyro[axisROLL].dir = 1;
  sensorDef.Gyro[axisPITCH].dir = 1;
  sensorDef.Gyro[axisYAW].dir = 1;

  sensorDef.Acc[axisROLL].dir = 1;
  sensorDef.Acc[axisPITCH].dir = 1;
  sensorDef.Acc[axisYAW].dir = 1;

}



// set sensor orientation according config
//
//   config.axisReverseZ
//        false ... sensor mounted on top
//        true  ... sensor mounted upside down
//   config.axisSwapXY
//        false ... default XY axes
//        true  ... swap XY (means exchange Roll/Pitch)

void initSensorOrientation() {

}

void setACCFastMode (bool fastMode) {

}


void flash_leds()
{}

void initIMU() {

	//isr_registry.init();
	//scheduler.init(&isr_registry);

#ifdef GIMBAL_IMU_SPI

	//Serial.print("Init SPI1\r\n");
	//spi1.begin(SPI_1_125MHZ, MSBFIRST, 0);
	//delay(500);

	Serial.print("INS Init\r\n");
	ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ, delay, flash_leds, NULL, &Serial);
	//ins.calibrate_accel(delay, flash_leds, setup_printf_P, setup_wait_key);
	//ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ, delay, flash_leds, NULL, &Serial);


#else
	//Serial.print("I2C Begin\r\n");
	//i2c2.begin();
	//delay(500);


	Serial.print("INS Init\r\n");
	ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_1000HZ, delay, flash_leds, NULL, &Serial);
	ins.init_accel(delay, flash_leds);
	//ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_1000HZ, delay, flash_leds, NULL, &Serial);

#endif



	ahrs.init();
	//ahrs.set_fast_gains(true);

#ifdef GIMBAL_ENABLE_COMPASS
	Serial.print("COMPASS Init\r\n");
	compass.set_orientation(MAG_ORIENTATION);                                                   // set compass's orientation on aircraft
	if (!compass.init() || !compass.read()) {
		Serial.println("COMPASS INIT ERROR");
		ahrs.set_compass(NULL);
	} else {
		ahrs.set_compass(&compass);
	}
#else
	ahrs.set_compass(NULL);
#endif

	Serial.print("Sensors Init END\r\n");
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
// needs angle in radian units !
void rotateV(struct fp_vector *v,float* delta) {

}

void readGyros() {
	//ins.read();
}

void readACC(axisDef axis) {

}

void updateGyroAttitude(){
#ifdef GIMBAL_ENABLE_COMPASS
//	static uint32_t compass_time = 0;
	compass.read();

//	if ((millis() - compass_time > 100) &&
//			compass.read()) {
//			compass.calculate(ahrs.get_dcm_matrix());
//			// read compass at 10Hz
//			compass_time = millis();
//		}
#endif

	ahrs.update();
}

void updateACC(){


}


void updateACCAttitude(){

}


void getAttiduteAngles() {

  // attitude of the estimated vector

  //angle[axisROLL]  = (int32_t) ( 180.0 * ahrs.roll * 1000.0 / PI );
  //angle[axisPITCH] = (int32_t) ( 180.0 * ahrs.pitch * 1000.0  / PI );
  //angle[axisYAW] = (int32_t) ( 180.0 * ahrs.yaw * 1000.0  / PI );

	angle[axisROLL]  = (int32_t)  ahrs.roll_sensor * ANGLE_PRECISION / 100;
	angle[axisPITCH] = (int32_t)  ahrs.pitch_sensor * ANGLE_PRECISION / 100;
	angle[axisYAW] = (int32_t)  ahrs.yaw_sensor * ANGLE_PRECISION / 100;

}


#endif
