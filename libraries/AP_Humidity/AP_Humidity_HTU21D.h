/*************************************************** 
  This is a library for the HTU21D-F Humidity & Temp Sensor
 ****************************************************/
#ifndef __AP_HUMIDITY_HTU21D_H__
#define __AP_HUMIDITY_HTU21D_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"


#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>


//defines are taken from the HTU21D-F Arduino library
#define HTU21DF_I2CADDR       0x80
#define HTU21DF_READTEMP      0xE3
#define HTU21DF_READHUM       0xF5
#define HTU21DF_WRITEREG      0xE6
#define HTU21DF_READREG       0xE7
#define HTU21DF_RESET         0xFE


#define PX4_I2C_BUS_DEFAULT		1
/* I2C bus address is 1010001x */
#define I2C_ADDRESS_HTU21D	0x40	/**< 7-bit address. Depends on the order code (this is for code "I") */
#define PATH_HTU21D		"/dev/htu21d"

/* Register address */
#define ADDR_READ_MR			0x00	/* write to this address to start conversion */

/* Measurement rate is 100Hz */
#define MEAS_RATE 100
#define MEAS_DRIVER_FILTER_FREQ 1.2f
#define CONVERSION_INTERVAL	(1000000 / MEAS_RATE)	/* microseconds */


class AP_Humidity_HTU21D : public device::I2C
{
 private:

  void measure(void);
  void collect_humidity(void);
  void timer(void);
  // TODO make bool healty status checker
  uint32_t _last_sample_time_ms;
  uint32_t _measurement_started_ms;

 public:
  AP_Humidity_HTU21D(int bus = PX4_I2C_BUS_DEFAULT, int address = HTU21DF_I2CADDR, const char *path = PATH_HTU21D);       //TODO make constructor

  virtual int init();
  bool get_humidity(float &_humidity);
  void reset(void);
  float _humidity, _temperature; //TODO add temperature reading func()

  float dummy_humidity = 0;  //delete later
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int humidity_main(int argc, char *argv[]);


#endif
//__AP_HUMIDITY_HTU21D_H__
