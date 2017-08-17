#include "icm.h"
#include <stdint.h>
#include "spi.h"
#include <common/timing.h>
#include "i2c_slave_interface.h"

static uint8_t icm_spi_transfer(uint16_t data) {
    spi_begin(SPI_DEVICE_ICM);

    SPI_DR(SPI3) = data;
    while (!(SPI_SR(SPI3) & SPI_SR_RXNE));
    uint16_t ret = SPI_DR(SPI3);

    spi_end();
    return ret;
}

#include "icm_defines.h"

static bool icm_initialized;

static void icm_set_user_bank(uint8_t user_bank) {
    icm_spi_transfer((ICM20948_REG_BANK_SEL<<8)|(user_bank<<4));
}

static uint8_t icm_read_reg(uint8_t user_bank, uint8_t reg) {
    icm_set_user_bank(user_bank);
    return icm_spi_transfer(((uint16_t)reg|0x80)<<8);
}

static void icm_write_reg(uint8_t user_bank, uint8_t reg, uint8_t value) {
    icm_set_user_bank(user_bank);
    icm_spi_transfer((reg<<8)|value);
}

static bool icm_i2c_slv_wait_for_completion(uint32_t timeout_us) {
    uint32_t tbegin_us = micros();
    while(!(icm_read_reg(ICM20948_I2C_MST_STATUS) & (1<<6))) {
        if (micros() - tbegin_us > timeout_us) {
            return false;
        }
    }
    return true;
}

static bool icm_i2c_slv_read(uint8_t address, uint8_t reg, uint8_t* ret) {
    icm_write_reg(ICM20948_I2C_MST_STATUS, 0);
    icm_write_reg(ICM20948_I2C_SLV4_ADDR, address|0x80);
    icm_write_reg(ICM20948_I2C_SLV4_REG, reg);
    icm_write_reg(ICM20948_I2C_SLV4_CTRL, (1<<7)|(1<<6));
    if (!icm_i2c_slv_wait_for_completion(10000)) {
        return false;
    }
    *ret = icm_read_reg(ICM20948_I2C_SLV4_DI);
    return true;

}

static bool icm_i2c_slv_write(uint8_t address, uint8_t reg, uint8_t value) {
    icm_write_reg(ICM20948_I2C_MST_STATUS, 0);
    icm_write_reg(ICM20948_I2C_SLV4_ADDR, address);
    icm_write_reg(ICM20948_I2C_SLV4_REG, reg);
    icm_write_reg(ICM20948_I2C_SLV4_DO, value);
    icm_write_reg(ICM20948_I2C_SLV4_CTRL, (1<<7)|(1<<6));
    return icm_i2c_slv_wait_for_completion(10000);

}

void icm_init(void) {
    const char* reason;
    char temp[33];

    for (uint8_t retries = 0; retries<5; retries++) {
        // Check WHOAMI
        if (icm_read_reg(ICM20948_WHO_AM_I) != 0xEA) {
            goto fail;
        }

        // Read USER_CTRL, disable MST_I2C, write USER_CTRL, and wait long enough for any active I2C transaction to complete
        icm_write_reg(ICM20948_USER_CTRL, icm_read_reg(ICM20948_USER_CTRL) & ~(1<<5));
        usleep(10000);

        // Perform a device reset, wait for completion, then wake the device
        // Datasheet is unclear on time required for wait time after reset, but mentions 100ms under "start-up time for register read/write from power-up"
        icm_write_reg(ICM20948_PWR_MGMT_1, 1<<7);
        usleep(100000);
        icm_write_reg(ICM20948_PWR_MGMT_1, 1);
        usleep(10000);

        // Disable the I2C slave interface (SPI-only) and enable the I2C master interface to the AK09916
        icm_write_reg(ICM20948_USER_CTRL, (1<<4)|(1<<5));

        // Set up the I2C master clock as recommended in the datasheet
        icm_write_reg(ICM20948_I2C_MST_CTRL, 7);

        // Configure the I2C master to delay shadowing of external sensor data until all data is received
        icm_write_reg(ICM20948_I2C_MST_DELAY_CTRL, 1<<7);


//         icm_write_reg(ICM20948_I2C_MST_STATUS, 0);
        icm_write_reg(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR|0x80);
        icm_write_reg(ICM20948_I2C_SLV0_REG, 0x00);
        icm_write_reg(ICM20948_I2C_SLV0_CTRL, (1<<7)|2);
        usleep(10000);

        // Check the AK09916 WHO_I_AM registers
        bool failed = false;
        uint8_t byte;

        failed = failed || !icm_i2c_slv_read(AK09916_I2C_ADDR, 0x00, &byte) || byte != 0x48;
        failed = failed || !icm_i2c_slv_read(AK09916_I2C_ADDR, 0x01, &byte) || byte != 0x09;


        if (failed) {
            reason = "ak09916 who_am_i";
            goto fail;
        }

        // Reset AK09916
        failed = failed || !icm_i2c_slv_write(AK09916_I2C_ADDR, 0x32, 1);

        usleep(100000);

        // Place AK09916 in continuous measurement mode 4 (100hz), readback and verify
        failed = failed || !icm_i2c_slv_write(AK09916_I2C_ADDR, 0x31, 0x08);
        failed = failed || !icm_i2c_slv_read(AK09916_I2C_ADDR, 0x31, &byte) || byte != 0x08;


        // Configure the I2C master to read the status register and measuremnt data
        icm_write_reg(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR|0x80);
        icm_write_reg(ICM20948_I2C_SLV0_REG, 0x10);
        icm_write_reg(ICM20948_I2C_SLV0_CTRL, (1<<7)|9);

        if (!failed) {
            icm_initialized = true;

            break;
        }
fail:
        usleep(10000);
        continue;
    }
}

static struct ak09916_measurement_s ak09916_measurement;

static void ak09916_update(void) {
    // TODO interrupt driven
    if (!icm_initialized || (icm_read_reg(ICM20948_EXT_SLV_SENS_DATA_00) & (1<<0)) == 0) {
        return;
    }

    uint8_t meas_bytes[6];
    for (uint8_t i=0; i<6; i++) {
        meas_bytes[i] = icm_read_reg(ICM20948_EXT_SLV_SENS_DATA_01+i);
    }

    uint8_t st2 = icm_read_reg(ICM20948_EXT_SLV_SENS_DATA_08);

    int16_t temp = 0;
    temp |= (uint16_t)meas_bytes[0];
    temp |= (uint16_t)meas_bytes[1]<<8;
    ak09916_measurement.x = temp*42120.0/32752.0;

    temp = 0;
    temp |= (uint16_t)meas_bytes[2];
    temp |= (uint16_t)meas_bytes[3]<<8;
    ak09916_measurement.y = temp*42120.0/32752.0;

    temp = 0;
    temp |= (uint16_t)meas_bytes[4];
    temp |= (uint16_t)meas_bytes[5]<<8;
    ak09916_measurement.z = temp*42120.0/32752.0;

    ak09916_measurement.hofl = (st2 & (1<<3)) != 0;

    i2c_slave_new_compass_data(ak09916_measurement.x, ak09916_measurement.y, ak09916_measurement.z, ak09916_measurement.hofl);
}

void icm_update(void) {
    ak09916_update();
}
