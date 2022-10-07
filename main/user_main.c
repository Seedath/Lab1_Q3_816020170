/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"


static const char *TAG = "main";


#define I2C_MASTER_SCL_IO           2                	/*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO			0					/*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        	/*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0                	/*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                	/*!< I2C master do not need buffer */

// define ADS1115 pin addresses
#define ADS1115_GND 						0x48
#define ADS1115_VDD 						0x49
#define ADS1115_SDA 						0x4A
#define ADS1115_SCL 						0x4B

#define WRITE_BIT                           I2C_MASTER_WRITE 	/*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  	/*!< I2C master read */
#define ACK_CHECK_EN                        0x1              	/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              	/*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              	/*!< I2C ack value */
#define NACK_VAL                            0x1              	/*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              	/*!< I2C last_nack value */

// define ASD1115 reg addresses
#define ADS1115_CONV 						0x00				//conversion reg
#define ADS1115_CONFIG 						0x01				//configuration reg
#define ADS1115_LOTHRESH 					0x02				//low threshold value
#define ADS1115_HITHRESH 					0x03				//high threshold values


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write mpu6050
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_master_ads1115_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_GND<< 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief test code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_master_ads1115_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_GND<< 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
		ESP_LOGI(TAG, "Cannot read data from ADS1115 \n");
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADS1115_GND<< 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t data_write(i2c_port_t i2c_num, uint8_t reg_address, uint16_t data)
{
    int ret;
    uint8_t temp[2];
    temp[0] = (data >> 8) & 0xFF;
    temp[1] = (data >> 0) & 0xFF;

    ret = i2c_master_ads1115_write(i2c_num, reg_address, temp, 2);
    return ret;
}

static esp_err_t data_read(i2c_port_t i2c_num, uint8_t reg_address, uint16_t *data)
{
    int ret;
    uint8_t temp[2];

    ret = i2c_master_ads1115_read(i2c_num, reg_address, temp, 2);
    *data = (temp[0] << 8) | temp[1];
    return ret;
}

static esp_err_t i2c_master_ads1115_init(i2c_port_t i2c_num)
{
    uint16_t conf_data;
    vTaskDelay(100 / portTICK_RATE_MS);

    i2c_master_init();

    uint8_t OS = 0x00;			// NULL
    uint8_t MUX = 0x04;       	// AINp = AIN0 and AINn = GND
    uint8_t PGA = 0x01;       	// FS = 4.096 V
    uint8_t MODE = 0x00;      	// Continuous-Conversion Mode
    uint8_t DR = 0x04;        	// 128SPS
    uint8_t COMP_MODE = 0x00; 	// Traditional Comparator
    uint8_t COMP_POL = 0x00;  	// Active Low
    uint8_t COMP_LAT = 0x00;  	// Non-latching Comparator
    uint8_t COMP_QUE = 0x02;  	// Assert After Four Conversions

    conf_data = (OS << 3) | MUX;
    conf_data = (conf_data << 3) | PGA;
    conf_data = (conf_data << 1) | MODE;
    conf_data = (conf_data << 3) | DR;
    conf_data = (conf_data << 1) | COMP_MODE;
    conf_data = (conf_data << 1) | COMP_POL;
    conf_data = (conf_data << 1) | COMP_LAT;
    conf_data = (conf_data << 2) | COMP_QUE;

    // Output Configuration Bits
    ESP_LOGI(TAG, "Configuration Bits: %d\n", (int)conf_data);

    // Writing to CONFIG Register
    ESP_ERROR_CHECK(data_write(i2c_num, ADS1115_CONFIG, conf_data));

   return ESP_OK;
}

static void i2c_task(void *arg)
{
    uint16_t adc_data;
    double voltage;
    esp_err_t ret;

    ret  = i2c_master_ads1115_init(I2C_MASTER_NUM);

    while(1){
        ret = data_read(I2C_MASTER_NUM, ADS1115_CONV, &adc_data);
        if(ret == ESP_OK){
            ESP_LOGI(TAG, "ADS1115 Read!\n");
            voltage = (double)adc_data * 1.25e-4;
            ESP_LOGI(TAG, "Voltage = %d.%d V\n", (uint16_t)voltage, (uint16_t)(voltage * 100) % 100);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        else{
            ESP_LOGI(TAG, "Cannot read ADS1115\n");
        }
    }
}

void app_main(void)
{
    //start i2c task
    xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, NULL);
}
