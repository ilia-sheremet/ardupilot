/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *   PX4 humidity
 */


#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 //TODO check VRbrain

#include <AP_Humidity_PX4.h>
#include <drivers/drv_humidity.h>
#include <uORB/topics/humidity.h> //TODO make uORB message
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

extern const AP_HAL::HAL& hal;

bool AP_Humidity_PX4::init()
{
    _fd = open(HTU21D_DEVICE_PATH, O_RDONLY);
    if (_fd == -1) {
        return false;
    }
    if (OK != ioctl(_fd, SENSORIOCSPOLLRATE, 100) ||
        OK != ioctl(_fd, SENSORIOCSQUEUEDEPTH, 15)) {  //TODO check the sequence
        hal.console->println("Failed to setup humidity sensor driver rate and queue");
    }

    return true;
}

// read the airspeed sensor
bool AP_Humidity_PX4::get_humidity()
{
    if (_fd == -1) {
        return false;
    }

    // read from the PX4 airspeed sensor
    float hum = 0;
    float temp = 0;
    uint16_t count = 0;

    struct humidity_s report;

//	uint64_t timestamp;
//	float humidity_percent;
//	float hum_temperature_celsius;

    while (::read(_fd, &report, sizeof(report)) == sizeof(report) &&
           report.timestamp != _last_timestamp) {
    	hum += report.humidity_percent;
    	temp += report.hum_temperature_celsius;
        count++;
        _last_timestamp = report.timestamp;
    }
    if (count == 0) {
        return false;
    }

    _htdu21_humidity = hum / count;
    _htdu21_temperature = temp / count;
    return true;
}


// read the temperature
bool AP_Humidity_PX4::get_temperature(float &temperature)
{

/* TODO make proper error handler
   if (_temperature < -80) {
        // almost certainly a bad reading. The ETS driver on PX4
        // returns -1000
        return false;
    }
*/

    temperature = _htdu21_temperature;
    return true;
}

void AP_Humidity_PX4::update(void)
{
	AP_Humidity_PX4::get_humidity();
}


#endif // CONFIG_HAL_BOARD
