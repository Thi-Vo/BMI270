////https://github.com/bimalpaneru/BMI270 -> ref
////https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/K128%20CoreS3/BMI270.PDF -> ref/ page 19
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c.h"
#include "bmi2.h"
#include <esp_log.h>
#include "config_file.h"
#include <rom/ets_sys.h>

#define RAD_TO_DEGREE 		57.29577951

//REG ADDRESS
#define WHO_AM_I            0x00u
#define ACCE				0x0Cu
#define GYRO				0x12u
#define PWR_CFG				0x7Cu
#define INIT_CTRL			0x59u
#define INT_STATUS			0x21u
#define PWR_CTRL			0x7Du
#define ACCE_CFG			0x40u
#define ACCE_RANGE			0x41u

static uint16_t acce_resolution = 8096;


typedef struct {
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} bmi2_dev_t;


//Write to reg
static esp_err_t bmi2_write(bmi2_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    bmi2_dev_t *sens = (bmi2_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

//Only use for burst write config file
//(Write 2 buffer to the same address)
static esp_err_t bmi2_write_config(bmi2_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf_1, const uint8_t *const data_buf_2, const uint8_t data_len)
{
    bmi2_dev_t *sens = (bmi2_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf_1, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf_2, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

//Read data from reg
static esp_err_t bmi2_read(bmi2_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    bmi2_dev_t *sens = (bmi2_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

//Burst write config file to bmi270. The config array in config_file.h
esp_err_t load_config(bmi2_handle_t sensor, const uint8_t* input)
{
	uint8_t init_val_0 = 0;
	uint8_t init_val_1 = 0;

	uint8_t init_addr_0 = 0x5Bu;
	uint8_t init_addr_1 = 0x5Cu;
	uint8_t init_data_addr = 0x5Eu;

	esp_err_t ret;
	for (uint16_t i = 0; i < 4096; i++) {
		init_val_0 = (i & 0x0F);
		init_val_1 = (i >> 4) & 0xFF;

		//-----------------Writing the base register of INIT_ADDR_0-----------------------//

		{
			ret = bmi2_write(sensor, init_addr_0, &init_val_0, 1);
		    assert(ESP_OK == ret);
		}

		//-----------------Writing the base register of INIT_ADDR_1-----------------------//
		{
			ret = bmi2_write(sensor, init_addr_1, &init_val_1, 1);
		    assert(ESP_OK == ret);
		}
		//------------------------------------------------------------------------//

		//----------Writing to the INIT DATA REGISTER---------------//
		{
			ret = bmi2_write_config(sensor, init_data_addr, input, (input + 1), 1);
		    assert(ESP_OK == ret);
		}
		//--------------------------------------------------------------------------------//

		input = input + 2;
	}
	return ret;
}
bmi2_handle_t bmi2_create(i2c_port_t port, const uint16_t sensor_addr)
{
    bmi2_dev_t *sensor = (bmi2_dev_t *) calloc(1, sizeof(bmi2_dev_t));
    sensor->bus = port;
    sensor->dev_addr = sensor_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (bmi2_handle_t) sensor;

}

void bmi2_delete(bmi2_handle_t sensor)
{
    bmi2_dev_t *sens = (bmi2_dev_t *) sensor;
    free(sens);
}

esp_err_t bmi2_get_deviceid(bmi2_handle_t sensor, uint8_t *const deviceid)
{
    return bmi2_read(sensor, WHO_AM_I, deviceid, 1);
}

esp_err_t bmi2_begin(bmi2_handle_t sensor)
{
	//soft reset
//	uint8_t soft_reset_val = 0xB6;
//	uint8_t soft_reset_addr = 0x7Eu;
//	esp_err_t ret = bmi2_write(sensor, soft_reset_addr, &soft_reset_val, 1);
//	vTaskDelay(pdMS_TO_TICKS(500));
//    assert(ESP_OK == ret);

	//init
	esp_err_t ret;
	uint8_t pwr_conf_val = 0x00;
	uint8_t init_ctrl_val = 0x00;

	//the delay is crucial for properly working of the sensor
	ret = bmi2_write(sensor, PWR_CFG, &pwr_conf_val, 1);
    assert(ESP_OK == ret);
    ets_delay_us(500);		//minimum 450us
	ret = bmi2_write(sensor, INIT_CTRL, &init_ctrl_val, 1);
    assert(ESP_OK == ret);

    load_config(sensor, bmi270_config_file);

    init_ctrl_val = 0x01;
	ret = bmi2_write(sensor, INIT_CTRL, &init_ctrl_val, 1);
    assert(ESP_OK == ret);

	uint8_t internal_status = 0x00;
    ets_delay_us(150000);		//minimum 140ms
	ret = bmi2_read(sensor, INT_STATUS, &internal_status, 1);
    assert(ESP_OK == ret);
    assert(0x01 == internal_status);


    //set working mode
	uint8_t pwr_ctrl_val = 0x0E;
	uint8_t acc_conf_val = 0x0C;
	pwr_conf_val = 0x02;


//	uint8_t acc_range_val = 0x00;

	ret = bmi2_write(sensor, PWR_CTRL, &pwr_ctrl_val, 1);
    assert(ESP_OK == ret);
	ret = bmi2_write(sensor, ACCE_CFG, &acc_conf_val, 1);
    assert(ESP_OK == ret);
//	ret = bmi2_write(sensor, acc_range_addr, &acc_range_val, 1);
//    assert(ESP_OK == ret);
	ret = bmi2_write(sensor, PWR_CFG, &pwr_conf_val, 1);
	ets_delay_us(1000);

    assert(ESP_OK == ret);

    uint8_t acce_range = 0x00;
	ret = bmi2_read(sensor, ACCE_RANGE, &acce_range, 1);
    assert(ESP_OK == ret);
    switch(acce_range){
    case 0:
    	acce_resolution = 16384;
    	break;
    case 1:
    	acce_resolution = 8192;
    	break;
    case 2:
    	acce_resolution = 4096;
    	break;
    case 3:
    	acce_resolution = 2048;
    	break;
    default:
    	break;
    }
	return ret;
}

//float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
//{
//	float half_scale = ((float)(1 << bit_width) / 2.0f);
//
//	return (9.80665 * val * g_range) / half_scale;
//}

esp_err_t bmi2_get_raw_acce(bmi2_handle_t sensor, bmi2_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = bmi2_read(sensor, ACCE, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[1] << 8) | (data_rd[0]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[3] << 8) | (data_rd[2]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[5] << 8) | (data_rd[4]));
    return ret;
}
esp_err_t bmi2_get_acce(bmi2_handle_t sensor, bmi2_acce_value_t *const acce_value)
{
	bmi2_raw_acce_value_t raw;
	esp_err_t ret = bmi2_get_raw_acce(sensor, &raw);
    assert(ESP_OK == ret);

    acce_value->acce_x = (float)raw.raw_acce_x/(float)acce_resolution;
    acce_value->acce_y = (float)raw.raw_acce_y/(float)acce_resolution;
    acce_value->acce_z = (float)raw.raw_acce_z/(float)acce_resolution;

    return ret;
}

float bmi2_get_inclination(bmi2_acce_value_t acce_value)
{
	float acce_z = 0;
	if(acce_value.acce_z < 0) acce_z = -acce_value.acce_z;		//make it positive
	else acce_z = acce_value.acce_z;

	if(acce_z > 1){
		return 90.0 - acosf(acce_z - 1)*RAD_TO_DEGREE;
	}
	return acosf(acce_z)*RAD_TO_DEGREE;
}
//esp_err_t bmi2_get_raw_gyro(bmi2_handle_t sensor, bmi2_raw_gyro_value_t *const raw_gyro_value)
//{
//    uint8_t data_rd[6];
//    esp_err_t ret = bmi2_read(sensor, bmi2_GYRO_XOUT_H, data_rd, sizeof(data_rd));
//
//    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
//    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
//    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
//
//    return ret;
//}
//
//esp_err_t bmi2_get_acce(bmi2_handle_t sensor, bmi2_acce_value_t *const acce_value)
//{
//    esp_err_t ret;
//    float acce_sensitivity;
//    bmi2_raw_acce_value_t raw_acce;
//
//    ret = bmi2_get_acce_sensitivity(sensor, &acce_sensitivity);
//    if (ret != ESP_OK) {
//        return ret;
//    }
//    ret = bmi2_get_raw_acce(sensor, &raw_acce);
//    if (ret != ESP_OK) {
//        return ret;
//    }
//
//    acce_value->acce_x = raw_acce.raw_acce_x * acce_sensitivity;
//    acce_value->acce_y = raw_acce.raw_acce_y * acce_sensitivity;
//    acce_value->acce_z = raw_acce.raw_acce_z * acce_sensitivity;
//    return ESP_OK;
//}
