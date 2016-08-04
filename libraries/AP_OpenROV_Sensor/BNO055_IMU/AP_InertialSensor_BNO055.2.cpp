/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-




///CODE IS NOT FINISHED!!!!!  Copied from MPU6000. Registers correct
#include <assert.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_BNO055.h"

extern const AP_HAL::HAL& hal;

// BNO055 accelerometer scaling
#define BNO055_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#define BNO055_DRDY_PIN BBB_P8_14
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#define BNO055_DRDY_PIN RPI_GPIO_24
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
#define BNO055_DRDY_PIN MINNOW_GPIO_I2S_CLK
#endif
#endif

// BNO055 registers
#define CHIP_ID 	                0x00    // Chip ident ificat ion code, read-only fixed value 0xA0
#define ACC_ID                      0x01    // Chip ID of t he Acceleromet er device, read-only fixed value0xFB
#define MAG_ID                      0x02    // Chip ID of the MAgnetometer device, read-only fixed value 0x32
#define GYR_ID                      0x03    // Chip ID of the Gyroscope device, read-only fixed calue 0x0F
#define SW_REV_ID_LSB               0x04    // Lower byte of SW REvision ID, read-only fixed calue depending on microcontroller
#define SW_REV_ID_MSB               0x05    // Upper byte of SW Revision ID
#define BL_REV_ID                   0x06    // Identifies the version of the bootloader, read-only
#define PAGE_ID                     0x07    // Read: Number of currently selected page; Write: Change Page, 0x00 or 0x01

#define ACC_DATA_X_LSB              0x08    // X axis acceleration data (low byte)
#define ACC_DATA_X_MSB              0x09    // X axis accelerometer data (high byte)
#define ACC_DATA_Y_LSB              0x0A    // Y axis accelerometer data (low byte)
#define ACC_DATA_Y_MSB              0x0B    // Y axis accelerometer data (high byte)
#define ACC_DATA_Z_LSB              0x0C    // Z axis acceleration data (low byte)
#define ACC_DATA_Z_MSB              0x0D    // Z axis acceleration data (high byte)

#define MAG_DATA_X_LSB              0x0E    // X axis magnetometer data (low byte)
#define MAG_DATA_X_MSB              0x0F    // X axis magnetometer data (high byte)
#define MAG_DATA_Y_LSB              0x10    // Y axis magnetometer data (low byte)
#define MAG_DATA_Y_MSB              0x11    // Y axis magnetometer data (high byte)
#define MAG_DATA_Z_LSB              0x12    // Z axis magnetometer data (low byte)
#define MAG_DATA_Z_MSB              0x13    // Z axis magnetometer data (high byte)

#define GYR_DATA_X_LSB              0x14    // X axis gyroscope data (low byte)
#define GYR_DATA_X_MSB              0x15    // X axis gyroscope data (high byte)
#define GYR_DATA_Y_LSB              0x16    // Y axis gyroscope data (low byte)
#define GYR_DATA_Y_MSB              0x17    // Y axis gyroscope data (high byte)
#define GYR_DATA_Z_LSB              0x18    // Z axis gyroscope data (low byte)
#define GYR_DATA_Z_MSB              0x19    // Z axis gyroscope data (high byte)

#define EUL_DATA_X_LSB              0x1A    // X axis heading data (low byte)
#define EUL_DATA_X_MSB              0x1B    // X axis heading data (high byte)
#define EUL_DATA_Y_LSB              0x1C    // Y axis heading data (low byte)
#define EUL_DATA_Y_MSB              0x1D    // Y axis heading data (high byte)
#define EUL_DATA_Z_LSB              0x1E    // Z axis heading data (low byte)
#define EUL_DATA_Z_MSB              0x1F    // Z axis heading data (high byte)

#define QUA_DATA_W_LSB              0x20    // W axis quaternion data (low byte)
#define QUA_DATA_W_MSB              0x21    // W axis quaternion data (high byte)
#define QUA_DATA_X_LSB              0x22    // X axis quaternion data (low byte)
#define QUA_DATA_X_MSB              0x23    // X axis quaternion data (high byte)
#define QUA_DATA_Y_LSB              0x24    // Y axis quaternion data (low byte)
#define QUA_DATA_Y_MSB              0x25    // Y axis quaternion data (high byte)
#define QUA_DATA_Z_LSB              0x26    // Z axis quaternion data (low byte)
#define QUA_DATA_Z_MSB              0x27    // Z axis quaternion data (high byte)

#define LIA_DATA_X_LSB              0x28    // X axis linear acceleration data (low byte)
#define LIA_DATA_X_MSB              0x29    // X axis linear acceleration data (high byte)
#define LIA_DATA_Y_LSB              0x2A    // Y axis linear acceleration data (low byte)
#define LIA_DATA_Y_MSB              0x2B    // Y axis linear acceleration data (high byte)
#define LIA_DATA_Z_LSB              0x2C    // Z axis linear acceleration data (low byte)
#define LIA_DATA_Z_MSB              0x2D    // Z axis linear acceleration data (high byte)

#define GRV_DATA_X_LSB              0x2E    // X axis gravity vector data (low byte)
#define GRV_DATA_X_MSB              0x2F    // X axis gravity vector data (high byte)
#define GRV_DATA_Y_LSB              0x30    // Y axis gravity vector data (low byte)
#define GRV_DATA_Y_MSB              0x31    // Y axis gravity vector data (high byte)
#define GRV_DATA_Z_LSB              0x32    // Z axis gravity vector data (low byte)
#define GRV_DATA_Z_MSB              0x33    // Z axis gravity vector data (high byte)

#define TEMP                        0x34    // Temperature data
#define CALIB_STAT                  0x35    // Current system calibration status
#define ST_RESULT                   0x36    // 1 for pass, 0 for fail self tests 
#define INT_STA                     0x37    // status of acc and gyros, 1 for interrupt triggered, 0 for no interrupt triggered
#define SYS_CLK_STATUS              0x38    // 0: free to configure CLK SRC, 1: in configuration state
#define SYS_STATUS                  0x39    // 0: idle; 1: error; 2-6 refer to datasheet
#define SYS_ERR                     0x3A    // system error code, see datasheet
#define UNIT_SEL                    0x3B    
#define OPR_MODE                    0x3D    // Read: current selected operation mode, Write: select operation mode
#define PWR_MODE                    0x3E    // Read: current selected power mode, Write: select power mode
#define SYS_TRIGGER                 0x3F    
#define TEMP_SOURCE                 0x40
#define AXIS_MAP_CONFIG             0x41
#define AXIS_MAP_SIGN               0x42

#define ACC_OFFSET_X_LSB            0x55
#define ACC_OFFSET_X_MSB            0x56
#define ACC_OFFSET_Y_LSB            0x57
#define ACC_OFFSET_Y_MSB            0x58
#define ACC_OFFSET_Z_LSB            0x59
#define ACC_OFFSET_Z_MSB            0x5A
#define MAG_OFFSET_X_LSB            0x5C
#define MAG_OFFSET_X_MSB            0x5D
#define MAG_OFFSET_Y_LSB            0x5E
#define MAG_OFFSET_Y_MSB            0x5F
#define MAG_OFFSET_Z_LSB            0x60
#define MAG_OFFSET_Z_MSB            0x61
#define GYR_OFFSET_X_LSB            0x62
#define GYR_OFFSET_X_MSB            0x63
#define GYR_OFFSET_Y_LSB            0x64
#define GYR_OFFSET_Y_MSB            0x65
#define GYR_OFFSET_Z_LSB            0x66
#define GYR_OFFSET_Z_MSB            0x67

#define ACC_RADIUS_MSB              0x68
#define MAG_RADIUS_LSB              0x69
#define MAG_RADIUS_MSB              0x6A





// Product ID Description for BNO055
// high 4 bits  low 4 bits

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

/*
 *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = (0.0174532f / 16.4f);

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */


AP_InertialSensor_BNO055::AP_InertialSensor_BNO055(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     bool use_fifo)
    : AP_InertialSensor_Backend(imu)
    , _use_fifo(use_fifo)
    , _temp_filter(1000, 1)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_BNO055::~AP_InertialSensor_BNO055()
{
    delete _auxiliary_bus;
}

AP_InertialSensor_Backend *AP_InertialSensor_BNO055::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_InertialSensor_BNO055 *sensor =
        new AP_InertialSensor_BNO055(imu, std::move(dev), true);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    sensor->_id = HAL_INS_BNO055_I2C;

    return sensor;
}

/*  Not currently implementing spi
AP_InertialSensor_Backend *AP_InertialSensor_BNO055::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev)
{
    AP_InertialSensor_BNO055 *sensor;

    dev->set_read_flag(0x80);

    sensor = new AP_InertialSensor_BNO055(imu, std::move(dev), false);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    sensor->_id = HAL_INS_MPU60XX_SPI;

    return sensor;
}
*/

bool AP_InertialSensor_BNO055::_init()
{
/*#ifdef BNO055_DRDY_PIN
    _drdy_pin = hal.gpio->channel(BNO055_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif
*/
    hal.scheduler->suspend_timer_procs();
    bool success = _hardware_init();
    hal.scheduler->resume_timer_procs();

#if BNO055_DEBUG
    _dump_registers();
#endif

    return success;
}

void AP_InertialSensor_BNO055::_fifo_reset()
{
    _register_write(MPUREG_USER_CTRL, 0);
    _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_RESET);
    _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_EN);
}

void AP_InertialSensor_BNO055::_fifo_enable()
{
    _register_write(MPUREG_FIFO_EN, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN);
    _fifo_reset();
    hal.scheduler->delay(1);
}

bool AP_InertialSensor_BNO055::_has_auxiliary_bus()
{
    return _dev->bus_type != AP_HAL::Device::BUS_TYPE_I2C;
}

void AP_InertialSensor_BNO055::start()
{
    hal.scheduler->suspend_timer_procs();

    if (!_dev->get_semaphore()->take(100)) {
        AP_HAL::panic("BNO055: Unable to get semaphore");
    }
    hal.console->printf("In start()")
    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    /*// only used for wake-up in accelerometer only low power mode
    _register_write(MPUREG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);
*/
    if (_use_fifo) {
        _fifo_enable();
    }

    // disable sensor filtering
    _set_filter_register(256);

    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    // Therefore the sample rate value is 8kHz/(SMPLRT_DIV + 1)
    // So we have to set it to 7 to have a 1kHz sampling
    // rate on the gyro
   // _register_write(MPUREG_SMPLRT_DIV, 7);
   // hal.scheduler->delay(1);

    // Gyro scale 2000º/s
    _register_write(0x0A, BITS_GYRO_FS_2000DPS); //config gyro
    hal.scheduler->delay(1);

   /* // read the product ID rev c has 1/2 the sensitivity of rev d
    _product_id = _register_read(MPUREG_PRODUCT_ID);
    //Serial.printf("Product_ID= 0x%x\n", (unsigned) _BNO055_product_id);

    // TODO: should be changed to 16G once we have a way to override the
    // previous offsets
    if ((_product_id == BNO055ES_REV_C4) ||
        (_product_id == BNO055ES_REV_C5) ||
        (_product_id == BNO055_REV_C4)   ||
        (_product_id == BNO055_REV_C5)) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        _register_write(MPUREG_ACCEL_CONFIG,1<<3);
    } else {
        // Accel scale 8g (4096 LSB/g) */
        _register_write(0x08,2<<3); //accel config
   // }

    hal.scheduler->delay(1);

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);

    // now that we have initialised, we set the bus speed to high
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();

    // grab the used instances
    _gyro_instance = _imu.register_gyro(1000);
    _accel_instance = _imu.register_accel(1000);

    hal.scheduler->resume_timer_procs();

    // start the timer process to read samples
    hal.scheduler->register_timer_process(
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BNO055::_poll_data, void));
}

/*
  process any
 */
bool AP_InertialSensor_BNO055::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    _publish_temperature(_accel_instance, _temp_filtered);

    return true;
}

AuxiliaryBus *AP_InertialSensor_BNO055::get_auxiliary_bus()
{
    if (_auxiliary_bus) {
        return _auxiliary_bus;
    }

    if (_has_auxiliary_bus()) {
        _auxiliary_bus = new AP_BNO055_AuxiliaryBus(*this);
    }

    return _auxiliary_bus;
}

/*
 * Return true if the BNO055 has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_BNO055::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 0;
    }
    uint8_t status = _register_read(MPUREG_INT_STATUS);
    return (status & BIT_RAW_RDY_INT) != 0;
}

/*
 * Timer process to poll for new data from the BNO055.
 */
void AP_InertialSensor_BNO055::_poll_data()
{
    if (!_dev->get_semaphore()->take_nonblocking()) {
        return;
    }

    if (_use_fifo) {
        _read_fifo();
    } else if (_data_ready()) {
        _read_sample();
    }

    _dev->get_semaphore()->give();
}

void AP_InertialSensor_BNO055::_accumulate(uint8_t *samples, uint8_t n_samples)
{
    for (uint8_t i = 0; i < n_samples; i++) {
        uint8_t *data = samples + BNO055_SAMPLE_SIZE * i;
        Vector3f accel, gyro;
        float temp;

        accel = Vector3f(int16_val(data, 1),
                         int16_val(data, 0),
                         -int16_val(data, 2));
        accel *= BNO055_ACCEL_SCALE_1G;

        gyro = Vector3f(int16_val(data, 5),
                        int16_val(data, 4),
                        -int16_val(data, 6));
        gyro *= GYRO_SCALE;

        temp = int16_val(data, 3);
        /* scaling/offset values from the datasheet */
        temp = temp/340 + 36.53;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
        accel.rotate(ROTATION_PITCH_180_YAW_90);
        gyro.rotate(ROTATION_PITCH_180_YAW_90);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
        accel.rotate(ROTATION_YAW_270);
        gyro.rotate(ROTATION_YAW_270);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
        accel.rotate(ROTATION_PITCH_180_YAW_90);
        gyro.rotate(ROTATION_PITCH_180_YAW_90);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
        accel.rotate(ROTATION_YAW_90);
        gyro.rotate(ROTATION_YAW_90);
#endif

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);

        _temp_filtered = _temp_filter.apply(temp);
    }
}

void AP_InertialSensor_BNO055::_read_fifo()
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t rx[MAX_DATA_READ];

    static_assert(MAX_DATA_READ <= 100, "Too big to keep on stack");

    if (!_block_read(MPUREG_FIFO_COUNTH, rx, 2)) {
        hal.console->printf("MPU60x0: error in fifo read\n");
        return;
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / BNO055_SAMPLE_SIZE;

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        return;
    }

    if (n_samples > BNO055_MAX_FIFO_SAMPLES) {
        hal.console->printf("bytes_read = %u, n_samples %u > %u, dropping samples\n",
                            bytes_read, n_samples, BNO055_MAX_FIFO_SAMPLES);

        /* Too many samples, do a FIFO RESET */
        _fifo_reset();
        return;
    }

    if (!_block_read(MPUREG_FIFO_R_W, rx, n_samples * BNO055_SAMPLE_SIZE)) {
        hal.console->printf("MPU60x0: error in fifo read %u bytes\n",
                            n_samples * BNO055_SAMPLE_SIZE);
        return;
    }

    _accumulate(rx, n_samples);
}

void AP_InertialSensor_BNO055::_read_sample()
{
    /* one register address followed by seven 2-byte registers */
    struct PACKED {
        uint8_t int_status;
        uint8_t d[14];
    } rx;

    if (!_block_read(MPUREG_INT_STATUS, (uint8_t *) &rx, sizeof(rx))) {
        if (++_error_count > 4) {
            // TODO: set bus speed low for this (and only this) device
            hal.console->printf("MPU60x0: error reading sample\n");
            return;
        }
    }

    _accumulate(rx.d, 1);
}

bool AP_InertialSensor_BNO055::_block_read(uint8_t reg, uint8_t *buf,
                                            uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_BNO055::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_BNO055::_register_write(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_BNO055::_set_filter_register(uint16_t filter_hz)
{
    uint8_t filter;
    // choose filtering frequency
    if (filter_hz == 0) {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    } else if (filter_hz <= 5) {
        filter = BITS_DLPF_CFG_5HZ;
    } else if (filter_hz <= 10) {
        filter = BITS_DLPF_CFG_10HZ;
    } else if (filter_hz <= 20) {
        filter = BITS_DLPF_CFG_20HZ;
    } else if (filter_hz <= 42) {
        filter = BITS_DLPF_CFG_42HZ;
    } else if (filter_hz <= 98) {
        filter = BITS_DLPF_CFG_98HZ;
    } else {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    }
    _register_write(MPUREG_CONFIG, filter);
}


bool AP_InertialSensor_BNO055::_hardware_init(void)
{
    if (!_dev->get_semaphore()->take(100)) {
        AP_HAL::panic("BNO055: Unable to get semaphore");
    }

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        uint8_t user_ctrl = _register_read(MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _register_write(MPUREG_USER_CTRL, user_ctrl & ~BIT_USER_CTRL_I2C_MST_EN);
            hal.scheduler->delay(10);
        }

        /* reset device */
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
        if (_dev->bus_type == AP_HAL::Device::BUS_TYPE_SPI) {
            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
            _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);
        }

        // Wake up device and select GyroZ clock. Note that the
        // BNO055 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }

        hal.scheduler->delay(10);
        if (_data_ready()) {
            break;
        }

#if BNO055_DEBUG
        _dump_registers();
#endif
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();

    if (tries == 5) {
        hal.console->println("Failed to boot BNO055 5 times");
        return false;
    }

    return true;
}

#if BNO055_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_BNO055::_dump_registers(void)
{
    hal.console->println("BNO055 registers");
    if (!_dev->get_semaphore()->take(100)) {
        return;
    }

    for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
        if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();

    _dev->get_semaphore()->give();
}
#endif

AP_BNO055_AuxiliaryBusSlave::AP_BNO055_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr,
                                                         uint8_t instance)
    : AuxiliaryBusSlave(bus, addr, instance)
    , _bno055_addr(MPUREG_I2C_SLV0_ADDR + _instance * 3)
    , _bno055_reg(_bno055_addr + 1)
    , _bno055_ctrl(_bno055_addr + 2)
    , _bno055_do(MPUREG_I2C_SLV0_DO + _instance)
{
}

int AP_BNO055_AuxiliaryBusSlave::_set_passthrough(uint8_t reg, uint8_t size,
                                                  uint8_t *out)
{
    auto &backend = AP_InertialSensor_BNO055::from(_bus.get_backend());
    uint8_t addr;

    /* Ensure the slave read/write is disabled before changing the registers */
    backend._register_write(_bno055_ctrl, 0);

    if (out) {
        backend._register_write(_bno055_do, *out);
        addr = _addr;
    } else {
        addr = _addr | BIT_READ_FLAG;
    }

    backend._register_write(_bno055_addr, addr);
    backend._register_write(_bno055_reg, reg);
    backend._register_write(_bno055_ctrl, BIT_I2C_SLVX_EN | size);

    return 0;
}

int AP_BNO055_AuxiliaryBusSlave::passthrough_read(uint8_t reg, uint8_t *buf,
                                                   uint8_t size)
{
    assert(buf);

    if (_registered) {
        hal.console->println("Error: can't passthrough when slave is already configured");
        return -1;
    }

    int r = _set_passthrough(reg, size);
    if (r < 0) {
        return r;
    }

    /* wait the value to be read from the slave and read it back */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_BNO055::from(_bus.get_backend());
    backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, size);

    /* disable new reads */
    backend._register_write(_bno055_ctrl, 0);

    return size;
}

int AP_BNO055_AuxiliaryBusSlave::passthrough_write(uint8_t reg, uint8_t val)
{
    if (_registered) {
        hal.console->println("Error: can't passthrough when slave is already configured");
        return -1;
    }

    int r = _set_passthrough(reg, 1, &val);
    if (r < 0) {
        return r;
    }

    /* wait the value to be written to the slave */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_BNO055::from(_bus.get_backend());

    /* disable new writes */
    backend._register_write(_bno055_ctrl, 0);

    return 1;
}

int AP_BNO055_AuxiliaryBusSlave::read(uint8_t *buf)
{
    if (!_registered) {
        hal.console->println("Error: can't read before configuring slave");
        return -1;
    }

    auto &backend = AP_InertialSensor_BNO055::from(_bus.get_backend());
    if (!backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, _sample_size)) {
        return -1;
    }

    return _sample_size;
}

/* BNO055 provides up to 5 slave devices, but the 5th is way too different to
 * configure and is seldom used */
AP_BNO055_AuxiliaryBus::AP_BNO055_AuxiliaryBus(AP_InertialSensor_BNO055 &backend)
    : AuxiliaryBus(backend, 4)
{
}

AP_HAL::Semaphore *AP_BNO055_AuxiliaryBus::get_semaphore()
{
    return static_cast<AP_InertialSensor_BNO055&>(_ins_backend)._dev->get_semaphore();
}

AuxiliaryBusSlave *AP_BNO055_AuxiliaryBus::_instantiate_slave(uint8_t addr, uint8_t instance)
{
    /* Enable slaves on BNO055 if this is the first time */
    if (_ext_sens_data == 0) {
        _configure_slaves();
    }

    return new AP_BNO055_AuxiliaryBusSlave(*this, addr, instance);
}

void AP_BNO055_AuxiliaryBus::_configure_slaves()
{
    auto &backend = AP_InertialSensor_BNO055::from(_ins_backend);

    /* Enable the I2C master to slaves on the auxiliary I2C bus*/
    uint8_t user_ctrl = backend._register_read(MPUREG_USER_CTRL);
    backend._register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_I2C_MST_EN);

    /* stop condition between reads; clock at 400kHz */
    backend._register_write(MPUREG_I2C_MST_CTRL,
                            BIT_I2C_MST_P_NSR | BIT_I2C_MST_CLK_400KHZ);

    /* Hard-code divider for internal sample rate, 1 kHz, resulting in a
     * sample rate of 100Hz */
    backend._register_write(MPUREG_I2C_SLV4_CTRL, 9);

    /* All slaves are subject to the sample rate */
    backend._register_write(MPUREG_I2C_MST_DELAY_CTRL,
                            BIT_I2C_SLV0_DLY_EN | BIT_I2C_SLV1_DLY_EN |
                            BIT_I2C_SLV2_DLY_EN | BIT_I2C_SLV3_DLY_EN);
}

int AP_BNO055_AuxiliaryBus::_configure_periodic_read(AuxiliaryBusSlave *slave,
                                                     uint8_t reg, uint8_t size)
{
    if (_ext_sens_data + size > MAX_EXT_SENS_DATA) {
        return -1;
    }

    AP_BNO055_AuxiliaryBusSlave *mpu_slave =
        static_cast<AP_BNO055_AuxiliaryBusSlave*>(slave);
    mpu_slave->_set_passthrough(reg, size);
    mpu_slave->_ext_sens_data = _ext_sens_data;
    _ext_sens_data += size;

    return 0;
}
