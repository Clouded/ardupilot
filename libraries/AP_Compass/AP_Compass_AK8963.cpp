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
 *       AP_Compass_AK8963.cpp 
 *       Code by Georgii Staroselskii. Emlid.com
 *
 *       Sensor is conected to SPI port
 *
 */

#include <AP_Math.h>
#include <AP_HAL.h>
#include <cstdio>
#include <unistd.h>

#include "AP_Compass_AK8963.h"


#define READ_FLAG					0x80
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG			0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_DO			0x63


/* bit definitions for MPUREG_USER_CTRL */
#define MPUREG_USER_CTRL			0x6A
#       define BIT_USER_CTRL_I2C_MST_EN	0x20            /* Enable MPU to act as the I2C Master to external slave sensors */

/* bit definitions for MPUREG_MST_CTRL */
#define MPUREG_I2C_MST_CTRL			0x24
#		define I2C_SLV0_EN			0x80
#		define I2C_MST_CLOCK_400KHZ 0x0D


#define AK8963_I2C_ADDR             0x0c

#define AK8963_WIA                  0x00
#		define AK8963_Device_ID		0x48

#define AK8963_ST1                  0x02
#		define AK8963_DRDY			0x01
#		define AK8963_DOR			0x02

#define AK8963_HXL                  0x03

/* bit definitions for AK8963 CNTL1 */
#define AK8963_CNTL1                0x0A
#		define	AK8963_CONTINUOUS_MODE1	0x2
#		define	AK8963_SELFTEST_MODE	0x8
#		define	AK8963_POWERDOWN_MODE	0x0
#		define	AK8963_FUSE_MODE	0xf
#		define	AK8963_16BIT_ADC	0x10
#		define	AK8963_14BIT_ADC	0x00

#define AK8963_CNTL2                0x0B
#		define AK8963_RESET				0x01

#define AK8963_ASTC					0x0C
#		define AK8983_SELFTEST_MAGNETIC_FIELD_ON 0x40

#define AK8963_ASAX					0x10

extern const AP_HAL::HAL& hal;

AP_Compass_AK8963::AP_Compass_AK8963() : Compass() {
    product_id = AP_COMPASS_TYPE_MPU9250;
	_healthy[0] = true;
	_initialised = false;
	_mag_x_accum =_mag_y_accum = _mag_z_accum = 0;
	_mag_x =_mag_y = _mag_z = 0;
	_accum_count = 0;
	_magnetometer_adc_resolution = AK8963_14BIT_ADC;
}

bool AP_Compass_AK8963::_register_read(uint8_t address, uint8_t *value)
{
	uint8_t tx[2];
	uint8_t rx[2];
	
	tx[0] = address | READ_FLAG;
	tx[1] = 0;
	_spi->transaction(tx, rx, 2);
	
	*value =  rx[1];
	
	return true;
}

bool AP_Compass_AK8963::_register_write(uint8_t address, uint8_t value)
{
	uint8_t tx[2];
	uint8_t rx[2];

	tx[0] = address;
	tx[1] = value;


//	hal.console->printf("0x%x: 0x%x\n", address, value);
	_spi->transaction(tx, rx, 2);

	return true;
}

/* TODO: need to take update rate into account */
void AP_Compass_AK8963::accumulate(void)
{
	if (!_initialised) {
		return;
	}
	uint32_t tnow = hal.scheduler->micros();

	if (!_spi_sem->take(100)) {
		hal.console->printf("_spi_sem->take failed\n");
		return;
	}

   bool result = _read_raw();
   if (!result) {
		hal.console->printf("_read_raw failed\n");
	}

	_spi_sem->give();

	if (result) {
		_mag_x_accum += _mag_x;
		_mag_y_accum += _mag_y;
		_mag_z_accum += _mag_z;
		_accum_count++;
	if (_accum_count == 10) {
		 _mag_x_accum /= 2;
		 _mag_y_accum /= 2;
		 _mag_z_accum /= 2;
		 _accum_count = 5;
	}
	_last_accum_time = tnow;
	}

}

void AP_Compass_AK8963::_AK8963_register_write(uint8_t address, uint8_t value)
{
	_register_write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);  /* Set the I2C slave addres of AK8963 and set for write. */
	_register_write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
	_register_write(MPUREG_I2C_SLV0_DO, value); /* Register value to continuous measurement in 16-bit */
	_register_write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | 0x01); /* Enable I2C and set 1 byte */
	hal.scheduler->delay(10);
}

void AP_Compass_AK8963::_AK8963_registers_read(uint8_t address, uint8_t count, uint8_t *value)
{
	_register_write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  /* Set the I2C slave addres of AK8963 and set for read. */
	_register_write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
	_register_write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count); /* Enable I2C and set @count byte */

	hal.scheduler->delay(10);
	for (uint8_t i = 0; i < count; i++) {
	_register_read(MPUREG_EXT_SENS_DATA_00 + i, value + i);
	}
}

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx + 1] << 8) | v[2*idx]))
/* TODO: get rid of delays! Use DRDY bit instead. */
bool AP_Compass_AK8963::_self_test()
{
	bool success = false;
	
	_AK8963_register_write(AK8963_CNTL1, AK8963_POWERDOWN_MODE | _magnetometer_adc_resolution); /* Register value to continuous measurement in 14-bit */
	_AK8963_register_write(AK8963_ASTC, AK8983_SELFTEST_MAGNETIC_FIELD_ON); /* Turn the internal magnetic field on */
	_AK8963_register_write(AK8963_CNTL1, AK8963_SELFTEST_MODE | _magnetometer_adc_resolution); /* Register value to self-test mode in 14-bit */
	hal.scheduler->delay(10);


//	uint8_t drdy = 0x0;
//	while ( (drdy & 0x01) == 0x0) {
//		_register_read(MPUREG_EXT_SENS_DATA_00, &drdy);
//		hal.console->printf("drdy: 0x%x\n", drdy);
//	}

	uint8_t rx[14];
	_AK8963_registers_read(AK8963_HXL, 0x07, rx);

	uint8_t st2 = rx[6]; /* End data read by reading ST2 register */
	float hx, hy, hz;
	if(!(st2 & 0x08)) { 
		hx = int16_val(rx,0);
		hy = int16_val(rx,1);
		hz = int16_val(rx,2);
		hal.console->printf("SELF_TEST: %f %f %f\n", hx, hy, hz);

		switch (_magnetometer_adc_resolution) {
			bool hx_is_in_range;
			bool hy_is_in_range;
			bool hz_is_in_range;
			case AK8963_14BIT_ADC: 
				hx_is_in_range = (hx >= - 50) && (hx <= 50);
				hy_is_in_range = (hy >= - 50) && (hy <= 50);
				hz_is_in_range = (hz >= - 800) && (hz <= -200);
				if (hx_is_in_range && hy_is_in_range && hz_is_in_range) {
					success = true;
				}
				break;
			case AK8963_16BIT_ADC:
				hx_is_in_range = (hx >= -200) && (hx <= 200);
				hy_is_in_range = (hy >= -200) && (hy <= 200);
				hz_is_in_range = (hz >= -3200) && (hz <= -800);
				if (hx_is_in_range && hy_is_in_range && hz_is_in_range) {
					success = true;
				}
				break;
			default:
				success = false;
				hal.scheduler->panic(PSTR("Wrong AK8963's ADC resolution selected"));
				break;
	}
	} else {
		success = false;
	}

	_AK8963_register_write(AK8963_ASTC, 0x0); /* Turn the internal magnetic field off */
	_AK8963_register_write(AK8963_CNTL1, AK8963_POWERDOWN_MODE | _magnetometer_adc_resolution); /* Register value to continuous measurement in 14-bit */

	return success;
}

bool AP_Compass_AK8963::init()
{
	_spi = hal.spi->device(AP_HAL::SPIDevice_MPU9250_AK8963);
	if (_spi == NULL) {
		_initialised = false;
		return false;
	}
	_spi_sem = _spi->get_semaphore();

	_healthy[0] = true;

	_field[0].x = 0.0f;
	_field[0].y = 0.0f;
	_field[0].z = 0.0f;

	if (!_spi_sem->take(100)) {
		hal.console->printf("_spi_sem->take failed\n");
		return false;
	}

	_register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_MST_EN);	/* I2C Master mode */
	_register_write(MPUREG_I2C_MST_CTRL, I2C_MST_CLOCK_400KHZ);	/*  I2C configuration multi-master  IIC 400KHz */

	_AK8963_register_write(AK8963_CNTL2, AK8963_RESET); /* Reset AK8963 */

	hal.scheduler->delay(10);

	uint8_t deviceid;
	_AK8963_registers_read(AK8963_WIA, 0x01, &deviceid); /* Read AK8963's id */

	if (deviceid != AK8963_Device_ID) {
		_initialised = false;
		hal.console->printf("WRONG AK8963 DEVICE ID: 0x%x\n", (unsigned)deviceid);
	} else {
		_calibrate();
		if (_self_test()) {	
			_initialised = true;
		} else {
			_initialised = false;
		}
	}

	_AK8963_register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE1 | _magnetometer_adc_resolution);/* Register value to continuous measurement in 14bit */

//	_dump_registers();
	_spi_sem->give();

	return _initialised;
}

bool AP_Compass_AK8963::_calibrate()
{
	hal.console->printf("CALIBRATTION START\n");
	_AK8963_register_write(AK8963_CNTL1, AK8963_FUSE_MODE | _magnetometer_adc_resolution); /* Enable FUSE-mode in order to be able to read calibreation data */

	hal.scheduler->delay(10);

	uint8_t response[3];
	_AK8963_registers_read(AK8963_ASAX, 0x03, response);

	#define Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)
	for (int i = 0; i < 3; i++) {
		float data = response[i];
		magnetometer_ASA[i] = ((data-128)/256+1);//*Magnetometer_Sensitivity_Scale_Factor;
		hal.console->printf("%d: %lf\n", i, magnetometer_ASA[i]);
	}

	hal.console->printf("CALIBRATTION END\n");

	return true;
}

bool AP_Compass_AK8963::_read_raw()
{
	uint8_t rx[14] = {0};
	_register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_MST_EN);	/* I2C Master mode */
	_register_write(MPUREG_I2C_MST_CTRL, I2C_MST_CLOCK_400KHZ);	/*  I2C configuration multi-master  IIC 400KHz */

	_AK8963_register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE1 | _magnetometer_adc_resolution);/* Register value to continuous measurement in 14bit */
	_AK8963_registers_read(AK8963_HXL, 0x07, rx);

	uint8_t st2 = rx[6]; /* End data read by reading ST2 register */

	if(!(st2 & 0x08)) { 
		_mag_x = (float) int16_val(rx, 0);
		_mag_y = (float) int16_val(rx, 1);
		_mag_z = (float) int16_val(rx, 2);
		return true;
	} else {
		return false;
	}

}

bool AP_Compass_AK8963::read()
{
	if (!_initialised) {
		return false;
	}

	if (_accum_count == 0) {
	   accumulate();
	}

	/* Update */
	_field[0].x = _mag_x_accum * magnetometer_ASA[0] / _accum_count;
	_field[0].y = _mag_y_accum * magnetometer_ASA[1] / _accum_count;
	_field[0].z = _mag_z_accum * magnetometer_ASA[2] / _accum_count;
	
	_mag_x_accum = _mag_y_accum = _mag_z_accum = 0;
	_accum_count = 0;

    if (product_id == AP_COMPASS_TYPE_MPU9250) {
		;
        //_field[0].rotate(ROTATION_YAW_90);
	}

	// apply default board orientation for this compass type. This is
	// a noop on most boards
	_field[0].rotate(MAG_BOARD_ORIENTATION);

	// add user selectable orientation
	_field[0].rotate((enum Rotation)_orientation.get());

	if (!_external) {
		// and add in AHRS_ORIENTATION setting if not an external compass
		_field[0].rotate(_board_orientation);
    }
	_field[0] += _offset[0].get();

	float x = _field[0].x;
	float y = _field[0].y;
	float z = _field[0].z;

	hal.console->printf("x: %f y: %f z: %f\n", x, y, z);

	last_update = hal.scheduler->micros(); // record time of update

	return true;
}

void AP_Compass_AK8963::_print_register(uint8_t address) 
{
	uint8_t response;
	_register_read(address, &response);
	hal.console->printf("%02x:%02x\n", (unsigned)address, (unsigned)response);
}

void AP_Compass_AK8963::_dump_registers()
{
	hal.console->printf("MPU9250 CONFIG REGS:\n");
	uint8_t registers_to_read[] = {MPUREG_USER_CTRL, MPUREG_I2C_MST_CTRL, MPUREG_I2C_SLV0_ADDR};
	for (uint8_t i = 0; i < sizeof(registers_to_read); i++) {
		_print_register(registers_to_read[i]);
	}

	hal.console->printf("MPU9250 EXT_SENS:\n");
	for (uint8_t i = MPUREG_EXT_SENS_DATA_00; i < MPUREG_EXT_SENS_DATA_00 + 7; i++) {
		_print_register(i);
	}

}
