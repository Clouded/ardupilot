#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>

#include "RCInput_Navio.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

void LinuxRCInput_Navio::init(void*)
{
    system("killall pigpiod -q");
    hal.scheduler->delay(500);
    system("pigpiod -s 1");
    hal.scheduler->delay(1000);
    system("pigs NO NB 0 0x10 > /dev/null");
    
    pigpio = open("/dev/pigpio0", O_RDONLY);
    
    if (pigpio == -1)
        hal.scheduler->panic("No pigpio interface for RCInput");
        
    int flags = fcntl(pigpio, F_GETFL, 0);
    fcntl(pigpio, F_SETFL, flags | O_NONBLOCK);
}

void LinuxRCInput_Navio::_timer_tick()
{
    while (true) {
        int len = ::read(pigpio, reinterpret_cast<uint8_t*>(&gpioReport), 12);
        
        if (len != 12)
            break;
                                  
        if (((gpioReport.level >> 4) & 0x01) == 1) {
            _process_ppmsum_pulse(gpioReport.tick - prevtick);
            prevtick = gpioReport.tick;
        } 
    }   
}

#endif // CONFIG_HAL_BOARD
