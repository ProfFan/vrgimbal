/* AP_InertialSensor_VRIMU
 This class is developed by Roberto Navoni a member of Arducopter DevTeam
 This class is compatible with VR NAVI FULL Sensor Board.
 It's use for retrival the data from Gyro and Acc Sensor.
*/
//#define DEBSER
#include "AP_InertialSensor_VRIMU.h"

static FastSerial *serPort;

static const unsigned char adc_cmd[9] = { 0x87, 0xC7, 0x97, 0xD7, 0xA7, 0xE7, 0xB7, 0xF7, 0x00 };


HardwareSPI *AP_InertialSensor_VRIMU::_SPIx;

uint8_t AP_InertialSensor_VRIMU::_cs_pin;


// ADC channel mappings on for the APM Oilpan
// Sensors: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
//const uint8_t AP_InertialSensor_Oilpan::_sensors[6] = { 1, 0, 2, 3, 4, 5 };

// ADC result sign adjustment for sensors.
//const  int8_t AP_InertialSensor_Oilpan::_sensor_signs[6] = { -1, -1, -1,- 1, 1 , -1 };

// ADC channel reading the gyro temperature
//const uint8_t AP_InertialSensor_Oilpan::_gyro_temp_ch = 3;
//const uint8_t AP_InertialSensor_Oilpan::_gyro_temp_ch = 6;

// Maximum possible value returned by an offset-corrected sensor channel
//const float AP_InertialSensor_Oilpan::_adc_constraint = 900;

// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g,
// 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 418
// 1G in the raw data coming from the accelerometer
// Value based on actual sample data from 20 boards
//const float AP_InertialSensor_Oilpan::_gravity = 423.8;

///< would like to use _gravity here, but cannot
//const float AP_InertialSensor_VRIMU::_accel_scale = 9.80665 / 423.8;

//const float AP_InertialSensor_VRIMU::_accel_scale = 9.80665 / 423.8;

// Oilpan accelerometer scaling & offsets
#define OILPAN_ACCEL_SCALE_1G   (GRAVITY * 2.0 / (2465.0 - 1617.0))
#define OILPAN_RAW_ACCEL_OFFSET ((2465.0 + 1617.0) * 0.5)
#define OILPAN_RAW_GYRO_OFFSET  1658.0


// IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s,
// 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192

const float  AP_InertialSensor_VRIMU::_gyro_gain_x = ToRad(0.4);
const float  AP_InertialSensor_VRIMU::_gyro_gain_y = ToRad(0.41);
const float  AP_InertialSensor_VRIMU::_gyro_gain_z = ToRad(0.41);

//const float _gyro_scale = (0.01745329252 / 16.4);
//const float _accel_scale = 9.81 / 423.0 ;

/* pch: I believe the accel and gyro indicies are correct
 *      but somone else should please confirm.
 */
const uint8_t AP_InertialSensor_VRIMU::_gyro_data_index[3]  = { 1, 0, 2 };
//const int8_t  AP_InertialSensor_VRIMU::_gyro_data_sign[3]   = { 1, 1, -1 };
const int8_t  AP_InertialSensor_VRIMU::_gyro_data_sign[3]   = { -1, -1, -1 };

const uint8_t AP_InertialSensor_VRIMU::_accel_data_index[3] = { 3, 4, 5 };
//const int8_t  AP_InertialSensor_VRIMU::_accel_data_sign[3]  = { 1, -1, -1 };
const int8_t  AP_InertialSensor_VRIMU::_accel_data_sign[3]  = { -1,  1, -1 };

const uint8_t AP_InertialSensor_VRIMU::_temp_data_index = 3;

static volatile uint8_t _new_data;

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t      _sum[7];

//static  int32_t _sum[7];

// how many values we've accumulated since last read
static  volatile uint16_t _count;


AP_InertialSensor_VRIMU::AP_InertialSensor_VRIMU( uint8_t cs_pin, HardwareSPI *spi_dev, FastSerial *ser_port)
{
  serPort = ser_port;
  _cs_pin = cs_pin; /* can't use initializer list,  is static */
  _gyro.x = 0;
  _gyro.y = 0;
  _gyro.z = 0;
  _accel.x = 0;
  _accel.y = 0;
  _accel.z = 0;
  _temp = 0;
  _initialised = 0;
  _SPIx = spi_dev;
  _scheduler = NULL;
  //Serial.printf("Create_VRIMU\n");

}


uint16_t  AP_InertialSensor_VRIMU::_init_sensor( AP_PeriodicProcess * scheduler, Sample_rate sample_rate )
{
    if (_initialised) return 1;
    _initialised = 1;
    hardware_init();
    _scheduler = scheduler;
    if (_scheduler)
    	_scheduler->register_process( &AP_InertialSensor_VRIMU::read );
#ifdef DEBSER
    serPort->printf_P("Init_VRIMU\n");
#endif
return 1;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_VRIMU::update( void )
{
    Vector3f gyro_offset = _gyro_offset.get();
    Vector3f accel_scale = _accel_scale.get();
    Vector3f accel_offset = _accel_offset.get();
	double sum[7];
	uint16_t count;
	double count_scale;
	//float adc_value[7];

	if (_count != 0)
	{
		// disable interrupts for mininum time
		//cli();
		noInterrupts();
		for (int i=0; i<7; i++) {
			sum[i] =(double) _sum[i];
			_sum[i] = 0;
		}
		count = _count;
		_count = 0;
		//sei();
		interrupts();

		count_scale = 1.0 / (double)count;

	_gyro.x = _gyro_gain_x * _gyro_data_sign[0] * (sum[_gyro_data_index[0]]* count_scale - OILPAN_RAW_GYRO_OFFSET );
    _gyro.y = _gyro_gain_y * _gyro_data_sign[1] * (sum[_gyro_data_index[1]]* count_scale - OILPAN_RAW_GYRO_OFFSET );
    _gyro.z = _gyro_gain_z * _gyro_data_sign[2] * (sum[_gyro_data_index[2]]* count_scale - OILPAN_RAW_GYRO_OFFSET );
    _gyro -= gyro_offset;

    _accel.x = accel_scale.x * _accel_data_sign[0] * (sum[_accel_data_index[0]]* count_scale - OILPAN_RAW_ACCEL_OFFSET) * OILPAN_ACCEL_SCALE_1G;
    _accel.y = accel_scale.y * _accel_data_sign[1] * (sum[_accel_data_index[1]]* count_scale - OILPAN_RAW_ACCEL_OFFSET) * OILPAN_ACCEL_SCALE_1G;
    _accel.z = accel_scale.z * _accel_data_sign[2] * (sum[_accel_data_index[2]]* count_scale - OILPAN_RAW_ACCEL_OFFSET) * OILPAN_ACCEL_SCALE_1G;
    _accel -= accel_offset;
		
#ifdef DEBSER	
    serPort->printf_P("Update AX: %f  AY: %f  AZ: %f  GX: %f  GY: %f  GZ: %f T=%f\n",
				  _accel.x, _accel.y, _accel.z, _gyro.x, _gyro.y, _gyro.x, _temp);
    serPort->printf_P("Update sum*count_scale 0:%f  1:%f  2:%f 3:%f  4:%f  5:%f 6:%f\n",
				  sum[0]*count_scale, sum[1]*count_scale, sum[2]*count_scale, sum[3]*count_scale, sum[4]*count_scale, sum[5]*count_scale, sum[6]*count_scale);
    serPort->printf_P("Update sum tot 0:%f  1:%f  2:%f 3:%f  4:%f  5:%f 6:%f\n",
				  sum[0], sum[1], sum[2], sum[3], sum[4], sum[5], sum[6]);
    serPort->printf_P("Update sum med 0:%f  1:%f  2:%f 3:%f  4:%f  5:%f 6:%f\n",
				  sum[0]*count_scale, sum[1]*count_scale, sum[2]*count_scale, sum[3]*count_scale, sum[4]*count_scale, sum[5]*count_scale, sum[6]*count_scale);
    serPort->printf_P("Update count_scale %f count : %d \n",count_scale,count);

#endif
	return true;
	}
  return false;
}
bool AP_InertialSensor_VRIMU::new_data_available( void )
{
    return _count != 0;
}
/*
void AP_InertialSensor_VRIMU::get_sensors( float * sensors )
{
  sensors[0] = _gyro.x;
  sensors[1] = _gyro.y;
  sensors[2] = _gyro.z;
  sensors[3] = _accel.x;
  sensors[4] = _accel.y;
  sensors[5] = _accel.z;
}
*/

float AP_InertialSensor_VRIMU::temperature() { return _temp; }

uint32_t AP_InertialSensor_VRIMU::sample_time()
{
  uint32_t us = micros();
  uint32_t delta = us - _last_sample_micros;
  reset_sample_time();
  return delta;
}

void AP_InertialSensor_VRIMU::reset_sample_time()
{
    _last_sample_micros = micros();
}

/*================ HARDWARE FUNCTIONS ==================== */

void AP_InertialSensor_VRIMU::read(uint32_t time)
{
uint8_t ch;
uint16_t v;
	
	digitalWrite(77,LOW);
	
	_SPIx->transfer(adc_cmd[0]);                       // Command to read the first channel
	
	for (ch = 0; ch < 8; ch++) {
	
		v = _SPIx->transfer(0) << 8;	         // Read first byte
		v |= _SPIx->transfer(adc_cmd[ch + 1]);  // Read second byte and send next command
		_sum[ch] += (v >> 3);	

	}
	_count++;

	digitalWrite(77,HIGH);
}

uint8_t AP_InertialSensor_VRIMU::register_read( uint8_t reg )
{
  uint8_t return_value;
  uint8_t addr = reg | 0x80; // Set most significant bit

  digitalWrite(SPI2_CS, LOW);

  _SPIx->transfer(addr);
  return_value = _SPIx->transfer(0);

  digitalWrite(SPI2_CS, HIGH);

  return return_value;
}

void AP_InertialSensor_VRIMU::register_write(uint8_t reg, uint8_t val)
{
  digitalWrite(SPI2_CS, LOW);
  _SPIx->transfer(reg);
  _SPIx->transfer(val);
  digitalWrite(SPI2_CS, HIGH);
}

// MPU6000 new data interrupt on INT6
void AP_InertialSensor_VRIMU::data_interrupt(void)
{
    // tell the timer routine that there is data to be read
    _new_data = 1;
}

void AP_InertialSensor_VRIMU::hardware_init()
{

	pinMode(SPI2_CS,OUTPUT);
	
	_SPIx->begin(SPI_1_125MHZ   ,MSBFIRST ,0);

	// VRIMU chip select setup
   
    digitalWrite(SPI2_CS, HIGH);
    delay(1);

	digitalWrite(SPI2_CS,LOW);
	
	// get an initial value for each channel. This ensures
	// _count[] is never zero
	for (uint8_t i=0; i<8; i++) {
		uint16_t adc_tmp;
		adc_tmp  = _SPIx->transfer(0) << 8;
		adc_tmp |= _SPIx->transfer(adc_cmd[i + 1]);
		_count = 1;
		_sum[i]   = adc_tmp;
	
	}
    digitalWrite(SPI2_CS, HIGH);
#ifdef DEBSER

    serPort->printf_P("Hardware Init sum 0:%d  1:%d  2:%d  3:%d  4:%d  5:%d 6:%d\n",
				  _sum[0], _sum[1], _sum[2], _sum[3], _sum[4], _sum[5], _sum[6]);
#endif

//	last_ch6_micros = micros();
	_SPIx->begin(SPI_2_25MHZ   ,MSBFIRST ,0);

}

float AP_InertialSensor_VRIMU::_temp_to_celsius ( uint16_t regval )
{
    /* TODO */
    return 20.0;
}

void AP_InertialSensor_VRIMU::read()
{
	AP_InertialSensor_VRIMU::read((uint32_t)micros());
}
uint32_t AP_InertialSensor_VRIMU::get_delta_time_micros() {
      uint32_t us = micros();
 _delta_time_micros = us - _last_sample_micros;
  reset_sample_time();
  return _delta_time_micros;
}
bool AP_InertialSensor_VRIMU::automaticRead()
{
	if (_scheduler)
		return true;
	return false;
}
float AP_InertialSensor_VRIMU::get_gyro_drift_rate(void)
{
    // 3.0 degrees/second/minute
    return ToRad(3.0/60);
}
uint16_t AP_InertialSensor_VRIMU::num_samples_available()
{
    return _count;
}
