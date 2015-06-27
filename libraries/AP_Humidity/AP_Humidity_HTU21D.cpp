/*************************************************** 
  This is a library for the HTU21DF Humidity & Temperature Sensor
 ****************************************************/

#include "AP_Humidity_HTU21D.h"

extern const AP_HAL::HAL& hal;

bool AP_Humidity_HTU21D::init(void){
	uint8_t read_reg = HTU21DF_READREG;

	hal.console->printf_P(PSTR("\n Initializing HTU21DF Humidity sensor \n")); //_comments

	// getting pointer to i2c bus semaphore
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

	// take i2c bus semaphore
	if (!i2c_sem->take(200))
	    return false;

    hal.i2c->write(HTU21DF_I2CADDR, 1 , NULL);
    hal.i2c->write(HTU21DF_I2CADDR, 1 , &read_reg);
    hal.scheduler->delay(10);
    hal.i2c->read(HTU21DF_I2CADDR, 3 , &read_reg);

    i2c_sem->give();
        if (_last_sample_time_ms != 0) {
            hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Humidity_HTU21D::timer, void));
            return true;
        }

    return false;
}

// start to measure humidity
void AP_Humidity_HTU21D::measure(void){
	uint8_t send_2 = 0xE5;

	hal.i2c->write(HTU21DF_I2CADDR, 1 , &send_2);

    _measurement_started_ms = 0;
    if (hal.i2c->write(HTU21DF_I2CADDR, 0xE5 , NULL) == 0) {
        _measurement_started_ms = hal.scheduler->millis();
    }
}

void AP_Humidity_HTU21D::collect_humidity(void)
{
    uint8_t data[3];

    _measurement_started_ms = 0;
    if (hal.i2c->read(HTU21DF_I2CADDR, 3, data) != 0) {
        return;
    }

    int16_t dH_raw;
    dH_raw = data[0] << 8;
    dH_raw |= (0x3 & data[1]);
//    hal.console->printf_P(PSTR("\nRaw[0] = %u\n"), data[0]); //_comments
//    hal.console->printf_P(PSTR("Raw[1] = %u\n"), data[1]);   //_comments
//    hal.console->printf_P(PSTR("Raw[2] = %u\n"), data[2]);   //_comments

    //_humidity = (dH_raw*125) / 65536 - 6;  // calculations from HTU21D datasheet
    _humidity = dH_raw;

    _last_sample_time_ms = hal.scheduler->millis();
}

// 1 KHz timer
void AP_Humidity_HTU21D::timer(void)
{
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    if (!i2c_sem->take_nonblocking())
        return;

    if (_measurement_started_ms == 0) {
        measure();
        i2c_sem->give();
        return;
    }
    if ((hal.scheduler->millis() - _measurement_started_ms) > 10) {
        collect_humidity();
        // start a new measurement
        measure();
    }
    i2c_sem->give();
}

bool AP_Humidity_HTU21D::get_humidity(float &humidity)
{
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

	measure();
	hal.scheduler->delay(50);

	collect_humidity();
	hal.scheduler->delay(10);

	i2c_sem->give();
    //humidity = _humidity;   // commented until i2c works properly

	/*****
	 * Temporary humidity for proper logging test
	 */
	dummy_humidity += 0.5;
	if (dummy_humidity >= 100){
		dummy_humidity = 0;
	}
	humidity = dummy_humidity;
    /////////////////////////////////////////////

    return true;
}
