/*************************************************** 
  This is a library for the HTU21DF Humidity & Temperature Sensor           - loop
 ****************************************************/

#include "AP_Humidity_HTU21D.h"

extern const AP_HAL::HAL& hal;

AP_Humidity_HTU21D::AP_Humidity_HTU21D(int bus, int address, const char* path):
		I2C("Humidity", path, bus, address, 100000)
{
}

int AP_Humidity_HTU21D::init(){

	hal.console->printf_P(PSTR("\n Initializing HTU21DF Humidity sensor ... ")); //_comments

	// getting pointer to i2c bus semaphore
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();
	if (i2c_sem == NULL) {
	        hal.scheduler->panic(PSTR("AP_SerialBus_I2C did not get valid I2C semaphore!"));
	    }
    //, I haven't changed anything else apart from the parameter
//	if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
//	        return false;
//	    }

	if (!i2c_sem->take(200)){
		hal.scheduler->panic(PSTR("PANIC: AP_Humidity_HTU21D: failed to take serial semaphore for init"));
	}


	uint8_t read_register = 0xE7, reset_register = 0xFE, read_hum = 0xE5, a, val;


	a = device::I2C::transfer(&reset_register, 1, nullptr, 0);
//	hal.console->printf_P(PSTR("\nA = %u\n"), a); //_comments
//	hal.scheduler->delay(30);

//	a = transfer(&read_register, 1, nullptr, 0);
//	hal.console->printf_P(PSTR("\nA2 = %u\n"), a); //_comments
//	hal.scheduler->delay(15);
//
//	a = transfer(nullptr, 0, &val, 1);

	uint8_t buf[3];
//	if (hal.i2c->readRegisters(HTU21DF_I2CADDR, HTU21DF_READREG, sizeof(buf), buf) == 0)
//	{
//		 hal.console->printf_P(PSTR("\nBuffer_before[0] = %u\n"), buf[0]); //_comments
//	}

	a = hal.i2c->write(HTU21DF_I2CADDR, 1, &read_hum);
	hal.console->printf_P(PSTR("\nA3 = %u\n"), a); //_comments
	hal.scheduler->delay(50);

	//hal.i2c->readRegisters(HTU21DF_I2CADDR , HTU21DF_READREG, sizeof(buf), buf);
	uint32_t b;
	hal.i2c->read(HTU21DF_I2CADDR, sizeof(buf), buf);
	i2c_sem->give();

	hal.console->printf_P(PSTR("\n b = %u\n"), b); //_comments

	hal.console->printf_P(PSTR("\nBuffer_after[0] = %u\n"), buf[0]); //_comments
	hal.console->printf_P(PSTR("\nBuffer_after[1] = %u\n"), buf[1]); //_comments
	hal.console->printf_P(PSTR("\nBuffer_after[2] = %u\n"), buf[2]); //_comments


//	// take i2c bus semaphore
//	if (!i2c_sem->take(200))
//	    return false;
//
//    hal.i2c->write(HTU21DF_I2CADDR, 1 , NULL);
//    hal.i2c->write(HTU21DF_I2CADDR, 1 , &read_reg);
//    hal.scheduler->delay(10);
//    hal.i2c->read(HTU21DF_I2CADDR, 3 , &read_reg);


        if (_last_sample_time_ms != 0) {
            hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Humidity_HTU21D::timer, void));
            return true;
        }

   // return false;
}

// start to measure humidity
void AP_Humidity_HTU21D::measure(void){
	uint8_t send_2 = 0xE5;
//
//	hal.i2c->write(HTU21DF_I2CADDR, 1 , &send_2);

    _measurement_started_ms = 0;
    if (hal.i2c->write(HTU21DF_I2CADDR, 1 , &send_2) == 0) {
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
    hal.console->printf_P(PSTR("\nRaw[0] = %u\n"), data[0]); //_comments
    hal.console->printf_P(PSTR("Raw[1] = %u\n"), data[1]);   //_comments
    hal.console->printf_P(PSTR("Raw[2] = %u\n"), data[2]);   //_comments

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

	if (!i2c_sem->take(400)){
			hal.scheduler->panic(PSTR("PANIC: AP_Humidity_HTU21D: failed to take serial semaphore for get_h"));
		}

	measure();
	hal.scheduler->delay(50);
	collect_humidity();

	i2c_sem->give();
    //humidity = _humidity;   // commented until i2c works properly

	/*****
	 * Temporary humidity for proper logging test
	 */
	dummy_humidity += 0.5;
	if (dummy_humidity >= 100){
		dummy_humidity = 0;
	}
	humidity = _humidity;
    /////////////////////////////////////////////

    return true;
}


