
#include "sen5x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"

#define SEN5X_I2C_ADDRESS 0x69

int16_t sen5x_start_measurement(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x21);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(50000);
    return NO_ERROR;
}

int16_t sen5x_start_measurement_without_pm(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x37);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(50000);
    return NO_ERROR;
}

int16_t sen5x_stop_measurement(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x104);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(200000);
    return NO_ERROR;
}

int16_t sen5x_read_data_ready(bool* data_ready) {
    int16_t error;
    uint8_t buffer[3];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x202);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 2);
    if (error) {
        return error;
    }
    *data_ready = buffer[1];
    return NO_ERROR;
}

int16_t sen5x_read_measured_values(uint16_t* mass_concentration_pm1p0,
                                   uint16_t* mass_concentration_pm2p5,
                                   uint16_t* mass_concentration_pm4p0,
                                   uint16_t* mass_concentration_pm10p0,
                                   int16_t* ambient_humidity,
                                   int16_t* ambient_temperature,
                                   int16_t* voc_index, int16_t* nox_index) {
    int16_t error;
    uint8_t buffer[24];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3C4);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 16);
    if (error) {
        return error;
    }
    *mass_concentration_pm1p0 = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    *mass_concentration_pm2p5 = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    *mass_concentration_pm4p0 = sensirion_common_bytes_to_uint16_t(&buffer[4]);
    *mass_concentration_pm10p0 = sensirion_common_bytes_to_uint16_t(&buffer[6]);
    *ambient_humidity = sensirion_common_bytes_to_int16_t(&buffer[8]);
    *ambient_temperature = sensirion_common_bytes_to_int16_t(&buffer[10]);
    *voc_index = sensirion_common_bytes_to_int16_t(&buffer[12]);
    *nox_index = sensirion_common_bytes_to_int16_t(&buffer[14]);
    return NO_ERROR;
}

int16_t sen5x_read_measured_raw_values(int16_t* raw_humidity,
                                       int16_t* raw_temperature,
                                       uint16_t* raw_voc, uint16_t* raw_nox) {
    int16_t error;
    uint8_t buffer[12];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3D2);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 8);
    if (error) {
        return error;
    }
    *raw_humidity = sensirion_common_bytes_to_int16_t(&buffer[0]);
    *raw_temperature = sensirion_common_bytes_to_int16_t(&buffer[2]);
    *raw_voc = sensirion_common_bytes_to_uint16_t(&buffer[4]);
    *raw_nox = sensirion_common_bytes_to_uint16_t(&buffer[6]);
    return NO_ERROR;
}

int16_t sen5x_read_measured_values_sen50(uint16_t* mass_concentration_pm1p0,
                                         uint16_t* mass_concentration_pm2p5,
                                         uint16_t* mass_concentration_pm4p0,
                                         uint16_t* mass_concentration_pm10p0) {
    int16_t error;

    int16_t ambient_humidity_dummy;
    int16_t ambient_temperature_dummy;
    int16_t voc_index_dummy;
    int16_t nox_index_dummy;

    error = sen5x_read_measured_values(
        mass_concentration_pm1p0, mass_concentration_pm2p5,
        mass_concentration_pm4p0, mass_concentration_pm10p0,
        &ambient_humidity_dummy, &ambient_temperature_dummy, &voc_index_dummy,
        &nox_index_dummy);

    return error;
}

int16_t sen5x_read_measured_pm_values(
    uint16_t* mass_concentration_pm1p0, uint16_t* mass_concentration_pm2p5,
    uint16_t* mass_concentration_pm4p0, uint16_t* mass_concentration_pm10p0,
    uint16_t* number_concentration_pm0p5, uint16_t* number_concentration_pm1p0,
    uint16_t* number_concentration_pm2p5, uint16_t* number_concentration_pm4p0,
    uint16_t* number_concentration_pm10p0, uint16_t* typical_particle_size) {
    int16_t error;
    uint8_t buffer[30];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x413);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 20);
    if (error) {
        return error;
    }
    *mass_concentration_pm1p0 = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    *mass_concentration_pm2p5 = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    *mass_concentration_pm4p0 = sensirion_common_bytes_to_uint16_t(&buffer[4]);
    *mass_concentration_pm10p0 = sensirion_common_bytes_to_uint16_t(&buffer[6]);
    *number_concentration_pm0p5 =
        sensirion_common_bytes_to_uint16_t(&buffer[8]);
    *number_concentration_pm1p0 =
        sensirion_common_bytes_to_uint16_t(&buffer[10]);
    *number_concentration_pm2p5 =
        sensirion_common_bytes_to_uint16_t(&buffer[12]);
    *number_concentration_pm4p0 =
        sensirion_common_bytes_to_uint16_t(&buffer[14]);
    *number_concentration_pm10p0 =
        sensirion_common_bytes_to_uint16_t(&buffer[16]);
    *typical_particle_size = sensirion_common_bytes_to_uint16_t(&buffer[18]);
    return NO_ERROR;
}

int16_t sen5x_start_fan_cleaning(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x5607);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_set_temperature_offset_parameters(int16_t temp_offset,
                                                int16_t slope,
                                                uint16_t time_constant) {
    int16_t error;
    uint8_t buffer[11];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60B2);

    offset =
        sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, temp_offset);
    offset = sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, slope);
    offset =
        sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset, time_constant);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_get_temperature_offset_parameters(int16_t* temp_offset,
                                                int16_t* slope,
                                                uint16_t* time_constant) {
    int16_t error;
    uint8_t buffer[9];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60B2);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 6);
    if (error) {
        return error;
    }
    *temp_offset = sensirion_common_bytes_to_int16_t(&buffer[0]);
    *slope = sensirion_common_bytes_to_int16_t(&buffer[2]);
    *time_constant = sensirion_common_bytes_to_uint16_t(&buffer[4]);
    return NO_ERROR;
}

int16_t sen5x_set_warm_start_parameter(uint16_t warm_start) {
    int16_t error;
    uint8_t buffer[5];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60C6);

    offset =
        sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset, warm_start);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_get_warm_start_parameter(uint16_t* warm_start) {
    int16_t error;
    uint8_t buffer[3];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60C6);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 2);
    if (error) {
        return error;
    }
    *warm_start = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sen5x_set_voc_algorithm_tuning_parameters(
    int16_t index_offset, int16_t learning_time_offset_hours,
    int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
    int16_t std_initial, int16_t gain_factor) {
    int16_t error;
    uint8_t buffer[20];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60D0);

    offset =
        sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, index_offset);
    offset = sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset,
                                                 learning_time_offset_hours);
    offset = sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset,
                                                 learning_time_gain_hours);
    offset = sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset,
                                                 gating_max_duration_minutes);
    offset =
        sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, std_initial);
    offset =
        sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, gain_factor);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_get_voc_algorithm_tuning_parameters(
    int16_t* index_offset, int16_t* learning_time_offset_hours,
    int16_t* learning_time_gain_hours, int16_t* gating_max_duration_minutes,
    int16_t* std_initial, int16_t* gain_factor) {
    int16_t error;
    uint8_t buffer[18];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60D0);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 12);
    if (error) {
        return error;
    }
    *index_offset = sensirion_common_bytes_to_int16_t(&buffer[0]);
    *learning_time_offset_hours = sensirion_common_bytes_to_int16_t(&buffer[2]);
    *learning_time_gain_hours = sensirion_common_bytes_to_int16_t(&buffer[4]);
    *gating_max_duration_minutes =
        sensirion_common_bytes_to_int16_t(&buffer[6]);
    *std_initial = sensirion_common_bytes_to_int16_t(&buffer[8]);
    *gain_factor = sensirion_common_bytes_to_int16_t(&buffer[10]);
    return NO_ERROR;
}

int16_t sen5x_set_nox_algorithm_tuning_parameters(
    int16_t index_offset, int16_t learning_time_offset_hours,
    int16_t learning_time_gain_hours, int16_t gating_max_duration_minutes,
    int16_t std_initial, int16_t gain_factor) {
    int16_t error;
    uint8_t buffer[20];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60E1);

    offset =
        sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, index_offset);
    offset = sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset,
                                                 learning_time_offset_hours);
    offset = sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset,
                                                 learning_time_gain_hours);
    offset = sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset,
                                                 gating_max_duration_minutes);
    offset =
        sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, std_initial);
    offset =
        sensirion_i2c_add_int16_t_to_buffer(&buffer[0], offset, gain_factor);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_get_nox_algorithm_tuning_parameters(
    int16_t* index_offset, int16_t* learning_time_offset_hours,
    int16_t* learning_time_gain_hours, int16_t* gating_max_duration_minutes,
    int16_t* std_initial, int16_t* gain_factor) {
    int16_t error;
    uint8_t buffer[18];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60E1);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 12);
    if (error) {
        return error;
    }
    *index_offset = sensirion_common_bytes_to_int16_t(&buffer[0]);
    *learning_time_offset_hours = sensirion_common_bytes_to_int16_t(&buffer[2]);
    *learning_time_gain_hours = sensirion_common_bytes_to_int16_t(&buffer[4]);
    *gating_max_duration_minutes =
        sensirion_common_bytes_to_int16_t(&buffer[6]);
    *std_initial = sensirion_common_bytes_to_int16_t(&buffer[8]);
    *gain_factor = sensirion_common_bytes_to_int16_t(&buffer[10]);
    return NO_ERROR;
}

int16_t sen5x_set_rht_acceleration_mode(uint16_t mode) {
    int16_t error;
    uint8_t buffer[5];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60F7);

    offset = sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset, mode);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_get_rht_acceleration_mode(uint16_t* mode) {
    int16_t error;
    uint8_t buffer[3];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x60F7);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 2);
    if (error) {
        return error;
    }
    *mode = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sen5x_set_voc_algorithm_state(const uint8_t* state,
                                      uint8_t state_size) {
    int16_t error;
    uint8_t buffer[14];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x6181);

    offset = sensirion_i2c_add_bytes_to_buffer(&buffer[0], offset, state,
                                               state_size);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_get_voc_algorithm_state(uint8_t* state, uint8_t state_size) {
    int16_t error;
    uint8_t buffer[12];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x6181);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 8);
    if (error) {
        return error;
    }
    sensirion_common_copy_bytes(&buffer[0], state, state_size);
    return NO_ERROR;
}

int16_t sen5x_set_fan_auto_cleaning_interval(uint32_t interval) {
    int16_t error;
    uint8_t buffer[8];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x8004);

    offset = sensirion_i2c_add_uint32_t_to_buffer(&buffer[0], offset, interval);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(20000);
    return NO_ERROR;
}

int16_t sen5x_get_fan_auto_cleaning_interval(uint32_t* interval) {
    int16_t error;
    uint8_t buffer[6];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x8004);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 4);
    if (error) {
        return error;
    }
    *interval = sensirion_common_bytes_to_uint32_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sen5x_get_product_name(unsigned char* product_name,
                               uint8_t product_name_size) {
    int16_t error;
    uint8_t buffer[48];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xD014);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(50000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 32);
    if (error) {
        return error;
    }
    sensirion_common_copy_bytes(&buffer[0], product_name, product_name_size);
    return NO_ERROR;
}

int16_t sen5x_get_serial_number(unsigned char* serial_number,
                                uint8_t serial_number_size) {
    int16_t error;
    uint8_t buffer[48];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xD033);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(50000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 32);
    if (error) {
        return error;
    }
    sensirion_common_copy_bytes(&buffer[0], serial_number, serial_number_size);
    return NO_ERROR;
}

int16_t sen5x_get_version(uint8_t* firmware_major, uint8_t* firmware_minor,
                          bool* firmware_debug, uint8_t* hardware_major,
                          uint8_t* hardware_minor, uint8_t* protocol_major,
                          uint8_t* protocol_minor) {
    int16_t error;
    uint8_t buffer[12];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xD100);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 8);
    if (error) {
        return error;
    }
    *firmware_major = buffer[0];
    *firmware_minor = buffer[1];
    *firmware_debug = buffer[2];
    *hardware_major = buffer[3];
    *hardware_minor = buffer[4];
    *protocol_major = buffer[5];
    *protocol_minor = buffer[6];
    return NO_ERROR;
}

int16_t sen5x_read_device_status(uint32_t* device_status) {
    int16_t error;
    uint8_t buffer[6];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xD206);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 4);
    if (error) {
        return error;
    }
    *device_status = sensirion_common_bytes_to_uint32_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sen5x_read_and_clear_device_status(uint32_t* device_status) {
    int16_t error;
    uint8_t buffer[6];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xD210);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(20000);

    error = sensirion_i2c_read_data_inplace(SEN5X_I2C_ADDRESS, &buffer[0], 4);
    if (error) {
        return error;
    }
    *device_status = sensirion_common_bytes_to_uint32_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sen5x_device_reset(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xD304);

    error = sensirion_i2c_write_data(SEN5X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(200000);
    return NO_ERROR;
}