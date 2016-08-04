/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   MS5837.cpp - pressure sensor example sketch
 *
 */

#include <AP_ADC/AP_ADC.h>
#include <MS5387/MS5387.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Vehicle::FixedWing aparm;

float pressure;

AP_MS5837 depth(aparm);

void setup()
{
    hal.console->println("ArduPilot Pressure Sensor library test");

}

void loop(void)
{
    static uint32_t timer;
    if((AP_HAL::millis() - timer) > 100) {
        timer = AP_HAL::millis();
        depth.read();
        depth.get_pressure(pressure);

        hal.console->printf("Depth %5.2f pressure %6.2f healthy = %u\n", depth.get_depth(), pressure, depth.healthy());
    }
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
