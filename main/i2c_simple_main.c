#include <bmi2.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "unity.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define bmi2_I2C_ADDRESS         0x68u /*!< I2C address with AD0 pin low */
#define bmi2_WHO_AM_I_VAL        0x24u

static bmi2_handle_t bmi2 = NULL;


static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

static void bmi2_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    bmi2 = bmi2_create(I2C_MASTER_NUM, bmi2_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(bmi2, "bmi2 create returned NULL");

    ret = bmi2_begin(bmi2);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

}

void app_main(void)
{

	esp_err_t ret;
	uint8_t bmi2_deviceid;
	bmi2_acce_value_t acce;
	float inclination;

	//init
	bmi2_init();

	ret = bmi2_get_deviceid(bmi2, &bmi2_deviceid);
	TEST_ASSERT_EQUAL(ESP_OK, ret);
	TEST_ASSERT_EQUAL_UINT8_MESSAGE(bmi2_WHO_AM_I_VAL, bmi2_deviceid, "Who Am I register does not contain expected data");
	printf("%" PRIu8 " \n", bmi2_deviceid);

	while(1)
	{
		ret = bmi2_get_acce(bmi2, &acce);
		TEST_ASSERT_EQUAL(ESP_OK, ret);
//		ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
//		printf("acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);
//		printf("acce_x:%.5f, acce_y:%.5f, acce_z:%.5f\n", acce.raw_acce_x/resolution, acce.raw_acce_y/resolution, acce.raw_acce_z/resolution);
		printf("acce_x:%.5f, acce_y:%.5f, acce_z:%.5f\n", acce.acce_x, acce.acce_y, acce.acce_z);

		inclination = bmi2_get_inclination(acce);
		printf("DO nghieng: %.2f*\n\n", inclination);
		vTaskDelay(pdMS_TO_TICKS(5000));
	}



	bmi2_delete(bmi2);
	ret = i2c_driver_delete(I2C_MASTER_NUM);
	TEST_ASSERT_EQUAL(ESP_OK, ret);
}
