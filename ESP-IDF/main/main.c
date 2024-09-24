#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "imu_tools.h"
#include "servo_tools.h"

void app_main(void)
{
  // Inicializa o IMU com o endereço especificado e os pinos SDA/SCL
  while (imu_init(mpu_6050_addr, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO) != ESP_OK) {
    printf("Tentando inicializar o IMU.\n");
    vTaskDelay(pdMS_TO_TICKS(3000));

  }
  esp_err_t teste = 0;
  IMUData dados_sensor;
  Quaternion dados_quaternion;
  EulerAngle dados_eulerangle;

  ServoConfig servo_1 = {
      .gpio_num = 27,
      .pwm_freq = 50,
      .min_angle = 0,
      .max_angle = 180
  };

  ServoConfig servo_2 = {
      .gpio_num = 13,
      .pwm_freq = 50,
      .min_angle = 0,
      .max_angle = 180
  };

  esp_err_t ret = servo_init(&servo_1);
  if (ret != ESP_OK)
  {
    printf("Falha ao inicializar o servo 1: %s", esp_err_to_name(ret));
    return;
  }

  ret = servo_init(&servo_2);
  if (ret != ESP_OK)
  {
    printf("Falha ao inicializar o servo 2: %s", esp_err_to_name(ret));
    return;
  }


  ServoAngle angle_1 = rand() % 181;
  ret = servo_set_angle(&servo_1, angle_1);
  if (ret == ESP_OK)
  {
    printf("Servo 1 movido para %d graus.\n", angle_1);
  }
  else
  {
    printf("Falha ao definir o ângulo do servo 1: %s\n", esp_err_to_name(ret));
  }

  ServoAngle angle_2 = rand() % 181;
  ret = servo_set_angle(&servo_2, angle_2);
  if (ret == ESP_OK)
  {
    printf("Servo 2 movido para %d graus.\n", angle_2);
  }
  else
  {
    printf("Falha ao definir o ângulo do servo 2: %s\n", esp_err_to_name(ret));
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

  while (1)
  {
    //MPU
    teste = imu_read_data(&dados_sensor);
    printf("%d\n",teste);
    teste = imu_calculate_quaternion(&dados_sensor,&dados_quaternion);
    printf("quaternion: q_w=%.2f q_x=%.2f q_y=%.2f q_z=%.2f\n\n",dados_quaternion.w,dados_quaternion.x,dados_quaternion.y,dados_quaternion.z);
    printf("%d\n",teste);
    teste = imu_calculate_euler_angles(&dados_quaternion,&dados_eulerangle);
    printf("%d\n",teste);
    printf("euler angle: roll=%.2f pitch=%.2f yaw=%.2f\n\n",dados_eulerangle.roll*(180.0f / PI),dados_eulerangle.pitch*(180.0f / PI),dados_eulerangle.yaw);

    // Delay de 1 segundo
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Movimenta o servo 1 para 90 graus
    if (servo_set_angle(&servo_1, dados_eulerangle.roll*(180.0f / PI)) != ESP_OK) {
      printf("Falha ao definir o ângulo do servo 1.\n");
    }

    // Movimenta o servo 2 para 45 graus
    if (servo_set_angle(&servo_2, dados_eulerangle.pitch*(180.0f / PI)) != ESP_OK) {
      printf("Falha ao definir o ângulo do servo 2.\n");
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }

}
