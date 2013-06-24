/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_MS5611_H__
#define __AP_BARO_MS5611_H__

#include <stdlib.h>
#include <wirish.h>
#include "AP_Baro.h"

//#define AP_BARO_MS5611_DEBUG_ENABLE

class AP_Baro_MS5611 : public AP_Baro
{
public:
	// Constructor
	AP_Baro_MS5611(uint8_t cs_pin, HardwareSPI *spi_dev, FastSerial *ser_port);

  /* AP_Baro public interface: */
  bool init( AP_PeriodicProcess *scheduler );
  uint8_t read();
  float get_pressure();     // in mbar*100 units
  float get_temperature();  // in celsius degrees * 100 units

  int32_t get_raw_pressure();
  int32_t get_raw_temp();


  void update();
  void _calculate();
 // void _update();
  /*
  bool automaticRead();
  */
  private:
  /* Asynchronous handler functions: */ 
  static void _update(uint32_t );
  /* Asynchronous state: */
  static volatile bool _updated;
  static volatile uint8_t _d1_count;
  static volatile uint8_t _d2_count;
  static volatile uint32_t _s_D1, _s_D2;
  static uint8_t _state;
  static uint32_t _timer;
  /* Gates access to asynchronous state: */
  static bool _sync_access;

  /* Serial wrapper functions: */
  static uint8_t  _spi_read(uint8_t reg);
  static uint16_t _spi_read_16bits(uint8_t reg);
  static uint32_t _spi_read_adc();
  static void     _spi_write(uint8_t reg);



  static uint8_t		_cs_pin;
  static HardwareSPI	*_SPIx;
  int32_t Temp;
  int32_t Press;

  int32_t _raw_press;
  int32_t _raw_temp;
  // Internal calibration registers
  uint16_t C1,C2,C3,C4,C5,C6;
  uint32_t D1,D2;

  AP_PeriodicProcess * _scheduler;
};

#endif //  __AP_BARO_MS5611_H__
