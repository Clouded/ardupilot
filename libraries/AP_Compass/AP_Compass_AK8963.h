/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_AK8963_H
#define AP_Compass_AK8963_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"

class AP_Compass_AK8963 : public Compass
{
private:
    bool                _register_read(uint8_t address, uint8_t *value);
	void				_AK8963_registers_read(uint8_t address, uint8_t count, uint8_t *value);
    bool                _register_write(uint8_t address, uint8_t value);
	void				_AK8963_register_write(uint8_t address, uint8_t value);
	void				_print_register(uint8_t address);
	void				_dump_registers();
	bool				_calibrate();
	bool				_self_test();
	bool				_read_raw();

    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
    bool                _initialised;
	float				magnetometer_ASA[3];
	float				_mag_x_accum;
	float				_mag_y_accum;
	float				_mag_z_accum;
	float				_mag_x;
	float				_mag_y;
	float				_mag_z;
	uint32_t			_accum_count;
	uint32_t			_last_accum_time;
	uint8_t				_magnetometer_adc_resolution;

public:
    AP_Compass_AK8963();

    bool        init(void);
    bool        read(void);
    void        accumulate(void);

};
#endif
