/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  simple hello world sketch
  Andrew Tridgell September 2011
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_PX4/I2CDriver.h>


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup() {
	hal.console->println("hello world");
    AP_HAL_PX4::I2CDriver::PX4I2CDriver::PX4I2CDriver dev;
}

void loop()
{
	hal.scheduler->delay(1000);
	hal.console->println("*");
    if (dev::do_transfer(0xED, 0x00, 1, 0xED, 1))
	{
		hal.console->println("Might have worked");
	}
	else
	{
		hal.console->println("Probably failed");
	}


}

AP_HAL_MAIN();
