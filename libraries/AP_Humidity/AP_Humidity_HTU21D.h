/*************************************************** 
  This is a library for the HTU21D-F Humidity & Temp Sensor
 ****************************************************/
#ifndef __AP_HUMIDITY_HTU21D_H__
#define __AP_HUMIDITY_HTU21D_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"


//defines are taken from the HTU21D-F Arduino library
#define HTU21DF_I2CADDR       0x40
#define HTU21DF_READTEMP      0xE3
#define HTU21DF_READHUM       0xF5
#define HTU21DF_WRITEREG      0xE6
#define HTU21DF_READREG       0xE7
#define HTU21DF_RESET         0xFE

class AP_Humidity_HTU21D
{
 private:

  void measure(void);
  void collect_humidity(void);
  void timer(void);
  // TODO make bool healty status checker
  uint32_t _last_sample_time_ms;
  uint32_t _measurement_started_ms;

 public:
  //AP_Humidity_HTU21D();       //TODO make constructor
  bool init(void);
  bool get_humidity(float &_humidity);
  void reset(void);
  float _humidity, _temperature; //TODO add temperature reading func()

  float dummy_humidity = 0;  //delete later
};

#endif
//__AP_HUMIDITY_HTU21D_H__
