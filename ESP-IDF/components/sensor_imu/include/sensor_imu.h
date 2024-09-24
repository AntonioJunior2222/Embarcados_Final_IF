#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H
/* Includes*/
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "math.h"
#include "driver/i2c.h"

/*Protótipo das funções*/
// Endereço e registradores do MPU6050
#define mpu_6050_addr 0x68
#define MPU6050_WHO_AM_I_REG_ADDR 0x75        /*!< "Who Am I" register address */
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B      /*!< Power management register address */
#define MPU6050_RESET_BIT 7                   /*!< Bit de reset do MPU6050 */

// Registradores de leitura dos dados de sensores
#define ACCEL_XOUT_H 0x3B        // Registrador de leitura do eixo X do acelerômetro
#define GYRO_XOUT_H 0x43         // Registrador de leitura do eixo X do giroscópio
#define PI 3.14159265358979323846

// Parâmetros de comunicação I2C
#define I2C_MASTER_SCL_IO 22        // Pino para o SCL
#define I2C_MASTER_SDA_IO 21        // Pino para o SDA
#define I2C_MASTER_FREQ_HZ 100000   // Frequência do I2C (100 kHz)
#define I2C_MASTER_NUM I2C_NUM_0    // Número da interface I2C
#define I2C_MASTER_TX_BUF_DISABLE 0 // Desabilitar buffer de transmissão
#define I2C_MASTER_RX_BUF_DISABLE 0 // Desabilitar buffer de recepção
#define I2C_MASTER_TIMEOUT_MS 1000  // Tempo limite de timeout em milissegundos
#define RAD_TO_DEG             (180.0 / M_PI)       /**< Conversão de radianos para graus. */
#define RAD_PER_DEG            (3.14159265358979 / 180.0) /**< Conversão de graus para radianos. */
// Parâmetros de cálculo de ângulos e sensores
#define MADGWICK_BETA 0.1f          // Parâmetro de ganho (ajustável)
#define ACCEL_SCALE 16384.0f        // Escala do acelerômetro para ±2g
#define GYRO_SCALE 131.0f           // Escala do giroscópio para ±250°/s
#define DEG_TO_RAD (M_PI / 180.0f)  // Conversão de graus para radianos
#define GYRO_SCALE_t            65.5              /**< Fator de escala para ±250°/s. */
#define ACCEL_SCALE_t          16384.0            /**< Fator de escala para ±2g. */

typedef struct {
    // Leituras do acelerômetro
    float accel_x;
    float accel_y;
    float accel_z;
} AccelerationData;
typedef struct {
    // Leituras do giroscópio
    float gyro_x;
    float gyro_y;
    float gyro_z;
} GyroscopeData;

esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin);
esp_err_t imu_get_acceleration_data(AccelerationData *data);
esp_err_t imu_get_gyroscope_data(GyroscopeData *data);
esp_err_t imu_deinit();
esp_err_t i2c_master_write_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_master_read_slave(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, size_t len);
#endif