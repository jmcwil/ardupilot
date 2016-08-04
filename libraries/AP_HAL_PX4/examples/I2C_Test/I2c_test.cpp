/* Test I2C driver for OpenROV */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
	hal.console->println("Starting AP_HAL::I2C test");

}

void loop(void)
{
	if (PX4I2CDriver::do_transfer(0xED, 0x00, 1, 0xED, 1))
	{
		hal.console->println("Might have worked");
	}
	else
	{
		hal.console->println("Probably failed");
	}

}
