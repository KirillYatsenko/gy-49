#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_types.h"
#include "math.h"

#define SDA_IO 18
#define SCL_IO 19
#define MAX44009 0x4b
#define REGISTER_ADDRESS 0x03

void read_lux_task(void *args);
double read_lux_i2c();

void app_main(void)
{
  i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = SDA_IO,
      .scl_io_num = SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000};

  i2c_param_config(I2C_NUM_0, &i2c_config);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

  xTaskCreate(&read_lux_task, "lux", 8000, NULL, 1, NULL);
}

void read_lux_task(void *args)
{
  while (true)
  {
    double lux = read_lux_i2c();
    printf("lux = %f\n", lux);

    vTaskDelay(2000 / portTICK_RATE_MS);
  }
}

double read_lux_i2c()
{
  uint8_t raw = 0;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MAX44009 << 1), true);
  i2c_master_write_byte(cmd, REGISTER_ADDRESS, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (MAX44009 << 1) | I2C_MASTER_READ, true);

  i2c_master_read_byte(cmd, &raw, I2C_MASTER_ACK);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  uint8_t exponent = raw >> 4;
  uint8_t mantissa = raw & 0xf;
  double lux = pow(2, exponent) * mantissa * 0.72;

  return lux;
}