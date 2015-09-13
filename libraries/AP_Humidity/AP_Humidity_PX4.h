 // -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Camera.h
/// @brief	Humidity sensor

#ifndef AP_HUMIDITY_PX4_H
#define AP_HUMIDITY_PX4_H

//#include <AP_Param.h>
//#include <AP_Common.h>
//#include <GCS_MAVLink.h>
//#include <GCS.h>
//#include <AP_Relay.h>
//#include <AP_GPS.h>
//#include <AP_AHRS.h>
//#include <AP_Mission.h>
//#include <AP_SerialManager.h>   // Serial manager library
#include <AP_HAL.h>
//#include <Plane.h>
//#include <GCS.h>

/// @class	Humidity
/// @brief	Object managing a Photo or video camera
class AP_Humidity_PX4 {

public:
    /// Constructor
    AP_Humidity_PX4(): _fd(-1) {init();}


    // init - initialise the sensor
    bool init();

    // update - read latest values of humidity and temperature
    void update(void);

    float get_htdu21d_humidity(void) const {
        return _htdu21_humidity;
    }

    float get_htdu21d_temperature(void) const {
        return _htdu21_temperature;
    }

private:
    int         _fd;                // file descriptor for sensor
    uint64_t    _last_timestamp;    // time of last update (used to avoid processing old reports)

    bool get_humidity();
    bool get_temperature(float &temperature);

    float _htdu21_humidity;
    float _htdu21_temperature;


};

#endif /* AP_HUMIDITY_PX4_H */

